#include "Copter.h"

/* Integrated fault tolerant control - variables and functions */
AP_HAL::OwnPtr<AP_HAL::SPIDevice> _teensy;

void check_variable(void);

/* Initialize the FTC functions  */
void Copter::ftc_init()
{
    int i;
    int j;

    Q_rank = 0;
    F_loc = 0;
    F_mag = 0;

    J_p = 0;
    J_q = 0;
    J_r = 0;

    p_norm = 0;
    q_norm = 0;
    r_norm = 0;

    rfc_counter = 0;

    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL

    for (i=0; i<4; i++)
    {
        NN_init[i] = 0;
        NN_train[i] = 0;
        NN_predict[i] = 0;
        NN_sample[i] = 0;
    }

    // zero initialize parameters
    for (i=0; i<4; i++)
    {
        for (j=0; j<4; j++)
        {
            Q_matrix[i][j] = 1;
        }
    }

    #else
        // initiliaze the SPI bus
        TeensySPI_init();
    #endif

    rfc_init();
}

/* Execute the FTC functions */
// This function should update both Teensy and Pixhawk with FTC-related data
void Copter::ftc_exe()
{

    /// execute FDD algorithm
    fdd_exe();

    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        int i;
        int j;
        float temp1 = 0.0;
        float num_var[4] = {0};


        if (sysid_man_flag)
        {
            start_fdd = 1;
        }

        if (NN_sample[sysid_active_rotor] < 353)
        NN_sample[sysid_active_rotor] = sysid_counter;

        // initialize NN ONCE
        if ((NN_sample[sysid_active_rotor] > 352) && (!NN_init[sysid_active_rotor]))
        {
            init_counter++;

            if (init_counter > 5) // number of executions until network has initialized ~ 5*2500 = 12500 ms
            {
                NN_init[sysid_active_rotor] = 1;
            }
        }

        // perform emulated training once NN is initialized
        if ((NN_init[sysid_active_rotor]) && (!NN_train[sysid_active_rotor]))
        {
            train_counter++; // start training network

            if (train_counter > 180) // number of executions until network has trained ~
            {
                NN_train[sysid_active_rotor] = 1;
                train_counter = 0;
                NN_sample[sysid_active_rotor] = 0; // reset data sample for prediction
            }
        }

        // perform emulated prediction once NN are gathered validation data
        if ((NN_sample[sysid_active_rotor] > 352) && (NN_train[sysid_active_rotor]) && (!NN_predict[sysid_active_rotor]))
        {
            predict_counter++; // start prediction network

            if (predict_counter > 5) // number of executions until network has predicted
            {
                NN_predict[sysid_active_rotor] = 1;
                predict_counter = 0;
            }
        }

        for (i=0; i<sysid_num_motor; i++)
        {
            if ((sysid_active_rotor == 0) && (NN_predict[i]))
            {
                Q_matrix[sysid_active_rotor][i] = -25;
                if ((i+1) > Q_rank){Q_rank = i+1;}
            }
            else if ((i == 0) && (NN_predict[sysid_active_rotor]))
            {
                Q_matrix[sysid_active_rotor][i] = 25;
                if ((i+1) > Q_rank){Q_rank = i+1;}
            }
            else if ((NN_predict[sysid_active_rotor]) && (NN_predict[i]))
            {
                Q_matrix[sysid_active_rotor][i] = 3;
                if ((i+1) > Q_rank){Q_rank = i+1;}
            }

        }

        // compute the Q_matrix variance
        if (sysid_num_motor > 0)
        {
            for (i=0; i<sysid_num_motor; i++)
            {
                for (j=0; j<sysid_num_motor; j++) // compute average across the column
                {
                    temp1 += (float)Q_matrix[j][i]/(float)sysid_num_motor;
                }

                // for loop here to compute the variance
                for (j=0; j<sysid_num_motor; j++)
                {
                    num_var[i] += powf((float)Q_matrix[j][i] - temp1,2)/((float)sysid_num_motor - 1);
                }
            }
        }

        /// Compute fault location through finding column with highest variance
        // initialize values
        temp1 = num_var[0];
        F_loc = 0;

        for (i=1; i<sysid_num_motor; i++)
        {
            if (num_var[i] >= temp1)
            {
                F_loc = (uint8_t)i; /// This is passed via SPI
                temp1 = num_var[i];
            }
        }

        /// Compute fault magnitude
        // This is computed based on the rotor
        F_mag = abs((float)Q_matrix[F_loc][F_loc]); /// This is passed via SPI


    #else
        // collect Q_matrix data (F_mag, F_loc and Q_rank) via SPI bus
        TeensySPI_update();
    #endif

    rfc_exe();
}

void Copter::fdd_exe()
{
    /// Introduce fault based on the inertial speed - This is controlled by RFC
    if ((inertial_nav.get_velocity_xy() > 300) && (J_r < 3.5) && (!start_rfc)) // 3 m/s
    {
        /// Fault emulation initialized
        motors.set_fault_state(0); //randNum);  // Fault #1 on any/all motors

        /// Fault injection started...
        if (inertial_nav.get_velocity_xy() > 500) // 5m/s
        {
            motors.set_perc_loss(0.6);
            motors.set_fault_type(1); // 0 - Thrust loss 1 - Slippage condition
        }

        /// Fault identification activation - ONLY in AUTO mode
        if (( control_mode == AUTO))
        {
            /*	mode    Dt_in  Dt_out D_step  Amp     Rotor   NumOfRotors       Repeat	Dt_repeat */
            motors.set_sysid_state(3,   100,    100,    100,    1.10,   0,      sysid_num_motor,  1,      1200);
            sysid_active_rotor = motors.get_rotor_num();
            sysid_man_flag = motors.get_man_state();

            // This is a debug to see how how many values the NN has to learn with vs what's been logged
            if (sysid_man_flag)
            {
                sysid_counter++;
            }
            else
            {
                sysid_counter = 0;
            }

        }
        else
        {
            motors.set_sysid_state(0,0,0,0,0.0,0,sysid_num_motor,0,0);
            sysid_counter = 0;
        }

    }
    else{
        /// Fault emulation
        if (!start_rfc)
        {
            motors.set_fault_state(0);
            motors.set_fault_type(0);
            motors.set_perc_loss(0.0);
        }

        /// Fault identification - stopped
        motors.set_sysid_state(0,0,0,0,0.0,0,sysid_num_motor,0,0);
        sysid_counter = 0;
    }

}


void Copter::TeensySPI_init()
{
    uint8_t reg = 0x0C;
    uint8_t val = 0x0D;
    uint8_t buf[2] = { reg, val };

    /// SPI BUS initialization
    hal.scheduler->suspend_timer_procs();

    // set the speed low before
    _teensy->set_speed(AP_HAL::Device::SPEED_LOW);

    /// This sends a data to MOSI line - to be used by Logic analyzer for debugging
    _teensy->transfer(buf, sizeof(buf), nullptr, 0);

    /// This is to receive an array bytes without sending anything from the MISO line
    // _teensy->transfer(nullptr, 0, buf, sizeof(buf));

    _teensy->set_speed(AP_HAL::Device::SPEED_HIGH);

    hal.scheduler->resume_timer_procs();

    // hal.scheduler->delay(5); // Can be used if needed

}

void Copter::TeensySPI_start()
{
    // enter code here
}

void Copter::TeensySPI_update()
{
    // enter code here
}

void Copter::rfc_init()
{
    J_the = 0;
}

void Copter::rfc_exe()
{

    // float amp = 0.5; // amplitude for Extremum sinusoidal input
    float p_meas;
    float q_meas;
    float r_meas;
    float p_cmd;
    float q_cmd;
    float r_cmd;

    //Vector3f cmdAng = attitude_control.get_att_target_euler_cd();
    //wp_nav.get_roll these are the values used in AUTO mode to set euler angle command. These should be manipulated jusr like sys-id maneuvers

    p_meas = ins.get_gyro(0).x;
    q_meas = ins.get_gyro(0).y;
    r_meas = ins.get_gyro(0).z;

    p_cmd = attitude_control.get_rate_roll_pid().get_pid_info().desired; /// to be checked with SITL data on the sign
    q_cmd = attitude_control.get_rate_pitch_pid().get_pid_info().desired;
    r_cmd = attitude_control.get_rate_yaw_pid().get_pid_info().desired;

    /// start reconfiguration once FDD has located fault
    if (Q_rank >= 2)
    {
        start_rfc = 1;
    }


    /// Enable reconfigurable control ONLY after FDD has begun
    if (start_rfc)
    {
        if (sysid_man_flag == 0)
        {
            rfc_counter++;
        }

        if ((p_norm < 1e-3) && (rfc_counter == 50)) // replace p_norm with std-deviation
        {
            p_norm = (float)sqrtf(pow(p_cmd,2));
            q_norm = (float)sqrtf(pow(q_cmd,2));
            r_norm = (float)sqrtf(pow(r_cmd,2));
        }

        // execute the cost function once the sys_id flag is LOW and counter has exceeded
        if ((sysid_man_flag == 0) && (rfc_counter > 50) && (rfc_counter < 600))
        {
            J_the = J_the + (powf(p_meas - p_cmd,2) + powf(q_meas - q_cmd,2) + powf(r_meas - r_cmd,2))*G_Dt;
            J_p = powf(p_meas/p_norm - p_cmd/p_norm,2)*G_Dt;
            J_q = powf(q_meas/q_norm - q_cmd/q_norm,2)*G_Dt;
            J_r = powf(r_meas/r_norm - r_cmd/r_norm,2)*G_Dt;
        }

        if (sysid_man_flag)
        {
            J_the = 0;

            J_p = 0;
            J_q = 0;
            J_r = 0;

            p_norm = 0;
            q_norm = 0;
            r_norm = 0;

            rfc_counter = 0;
        }

        /// start extremum seeking control during FDD
        if ((Q_rank >= 2) && (sysid_man_flag))
        {
            // enter code
        }
    }

}
