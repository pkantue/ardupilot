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
        // collect Q_matrix data via SPI bus
        TeensySPI_update();
    #endif

    rfc_exe();
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
    float p_meas;
    float q_meas;
    float r_meas;
    float p_cmd;
    float q_cmd;
    float r_cmd;

    p_meas = ins.get_gyro(0).x;
    q_meas = ins.get_gyro(0).y;
    r_meas = ins.get_gyro(0).z;

    p_cmd = attitude_control.get_rate_roll_pid().get_pid_info().desired; /// to be checked with SITL data on the sign
    q_cmd = attitude_control.get_rate_pitch_pid().get_pid_info().desired;
    r_cmd = attitude_control.get_rate_yaw_pid().get_pid_info().desired;

    /// compute Ojective function using error and integration timestep
    if (sysid_man_flag)
    {
        start_rfc = 1;
    }

    // execute the cost function once the sys_id flag is LOW
    if ((start_rfc) && (sysid_man_flag == 0))
    {
        J_the = J_the + (powf(p_meas - p_cmd,2) + powf(q_meas - q_cmd,2) + powf(r_meas - r_cmd,2))*G_Dt;
    }
    else
    {
        // compute the actuation gain values based on the FDD has concluded.
    }
}
