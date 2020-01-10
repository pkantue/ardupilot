#include "Copter.h"

/* Integrated fault tolerant control - variables and functions */
AP_HAL::OwnPtr<AP_HAL::SPIDevice> _teensy;

void check_variable(void);
#define MIN_PERIOD 50
#define MAX_PERIOD 200
#define PERT_AMP 0.01f // pertubation amplitude
#define ES_LOOP_GAIN 0.1f  // loop gain prior to signal modulation

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

    p_mean = 0;
    q_mean = 0;
    r_mean = 0;

    p_var = 0;
    q_var = 0;
    r_var = 0;

    p_std = 0;
    q_std = 0;
    r_std = 0;

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

    // zero initialize rfc streams
    for (i=0; i<MAX_STREAM_PERIOD; i++)
    {
        rfc_pitch_str[i] = 0.0;
        rfc_roll_str[i] = 0.0;
        rfc_yaw_str[i] = 0.0;
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

            if (init_counter > 5) /// number of executions until network has initialized ~ 5*2500 = 12500 us
            {
                NN_init[sysid_active_rotor] = 1;
            }
        }

        // perform emulated training once NN is initialized
        if ((NN_init[sysid_active_rotor]) && (!NN_train[sysid_active_rotor]))
        {
            train_counter++; // start training network

            if (train_counter > 180) /// number of executions until network has trained ~ 180*2500 = 450000 us
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

            if (predict_counter > 5) /// number of executions until network has predicted ~ 5*2500 = 12500 us
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
        /// collect Q_matrix data (F_mag, F_loc and Q_rank) via SPI bus
        TeensySPI_update();
    #endif

    /// Execute the reconfigurable controller
    rfc_exe();
}

void Copter::fdd_exe()
{
    float yaw_rate_meas = ins.get_gyro(0).z;
    float yaw_rate_cmd = attitude_control.get_rate_yaw_pid().get_pid_info().desired;

    /// Introduce fault based on the inertial speed - This is controlled by RFC
    if ((inertial_nav.get_velocity_xy() > 300) && ( sqrtf(powf(yaw_rate_meas - yaw_rate_cmd,2)) < 3.5) && (!start_rfc)) // 3 m/s
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

        //motors.set_fault_state(0);
        //motors.set_fault_type(0);
        //motors.set_perc_loss(0.0);

        /// Fault identification - stopped
        motors.set_sysid_state(0,0,0,0,0.0,0,sysid_num_motor,0,0);
        sysid_counter = 0;
        sysid_man_flag = 0;
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
    int i;

    J_the = 0;

    J_the_hold = 0;

    rfc_counter = 0;
    rfc_man = 0;
    rfc_period = 0;

    roll_cmd = 0;
    pitch_cmd = 0;

    rfc_speed = 0;
    rfc_speed1 = 0;

    for(i=0;i<MAX_NUMBER_ROTORS;i++)
    {
        rfc_gains[i] = 1.0;
    }

    /// initialize filters - LPF = 4/(s+4)
    ES_LPF.a[0] = 1; // denominator
    ES_LPF.a[1] = 4;

    ES_LPF.b[0] = 0; // numerator
    ES_LPF.b[1] = 4;

    for (i=0; i < 2; i++){
        ES_LPF.X[i] = 0;
        ES_LPF.dX[i] = 0;
        ES_LPF.dX1[i] = 0;
    }

    ES_LPF.in = 0;
    ES_LPF.out = 0;
    ES_LPF.init_state = 0;

    /// initialize filters - HPF = s/(s+2)
    ES_HPF.a[0] = 1; // denominator
    ES_HPF.a[1] = 2;

    ES_HPF.b[0] = 1; // numerator
    ES_HPF.b[1] = 0;

    for (i=0; i < 2; i++){
        ES_HPF.X[i] = 0;
        ES_HPF.dX[i] = 0;
        ES_HPF.dX1[i] = 0;
    }

    ES_HPF.in = 0;
    ES_HPF.out = 0;
    ES_HPF.init_state = 0;

    filter_dtime = 0;

    pert_sign = 1; // pertubation sign for stimulating objective function

    delt_gain = 0;
    int_hold = 0;
}

void Copter::rfc_exe()
{

    float amp = 0.7; // amplitude for objective function step command
    float p_meas;
    float q_meas;
    float r_meas;
    float p_cmd;
    float q_cmd;
    float r_cmd;

    float delt_p;
    float delt_q;
    float delt_r;

    int i;
    int8_t scale_dir = 1;

    /// holding values
    float aside0 = 0;
    float aside1 = 0;
    float cside0 = 0;
    float cside1 = 0;

    //Vector3f cmdAng = attitude_control.get_att_target_euler_cd();
    //wp_nav.get_roll these are the values used in AUTO mode to set euler angle command. These should be manipulated jusr like sys-id maneuvers

    p_meas = (float)ahrs.roll_sensor/100.0f; //ins.get_gyro(0).x;
    q_meas = (float)ahrs.pitch_sensor/100.0f; //ins.get_gyro(0).y;
    r_meas = (float)ahrs.yaw_sensor/100.0f; //ins.get_gyro(0).z;

    p_cmd = (float)wp_nav.get_roll()/100.0f; // attitude_control.get_rate_roll_pid().get_pid_info().desired; /// to be checked with SITL data on the sign
    q_cmd = (float)wp_nav.get_pitch()/100.0f; //attitude_control.get_rate_pitch_pid().get_pid_info().desired;
    r_cmd = (float)wp_nav.get_yaw()/100.0f; //attitude_control.get_rate_yaw_pid().get_pid_info().desired;

    /// window created as a circular buffer
    uint8_t ndx = rfc_period % MAX_STREAM_PERIOD;

    /// start reconfiguration once FDD has located fault
    if (Q_rank >= 2)
    {
        start_rfc = 1;

        /// update increment
        rfc_period++;
    }

    /// get old values to be removed from statistics
    float p_meas1 = rfc_roll_str[ndx];
    float q_meas1 = rfc_pitch_str[ndx];
    float r_meas1 = rfc_yaw_str[ndx];

    float p_mean1 = p_mean;
    float q_mean1 = q_mean;
    float r_mean1 = r_mean;

    /// replace old value with new observation
    rfc_roll_str[ndx] = p_meas;
    rfc_pitch_str[ndx] = q_meas;
    rfc_yaw_str[ndx] = r_meas;

    /// Collect data for the RFC activation of ES logic
    if ((start_rfc) && (rfc_period <= MAX_STREAM_PERIOD))
    {
        // compute mean and variance and deviation - roll
        delt_p = p_meas - p_mean1;
        p_mean += delt_p/(float)rfc_period;
        p_var += delt_p*(p_meas - p_mean);
        if (p_var >= 0.0){p_std = sqrtf(p_var/(float)rfc_period);}

        // compute mean and variance and deviation - pitch
        delt_q = q_meas - q_mean1;
        q_mean += delt_q/(float)rfc_period;
        q_var += delt_q*(q_meas - q_mean);
        if (q_var >= 0.0){q_std = sqrtf(q_var/(float)rfc_period);}

        // compute mean and variance and deviation - yaw
        delt_r = r_meas - r_mean1;
        r_mean += delt_r/(float)rfc_period;
        r_var += delt_r*(r_meas - r_mean);
        if (r_var >= 0.0){r_std = sqrtf(r_var/(float)rfc_period);}
    }
    else if (start_rfc) // reset period and store first values
    {
        // compute mean and variance and deviation - roll
        delt_p = p_meas - p_meas1;
        p_mean += delt_p/(float)MAX_STREAM_PERIOD;
        p_var += delt_p*((p_meas - p_mean) + (p_meas1 - p_mean1));
        if (p_var >= 0.0){p_std = sqrtf(p_var/(float)MAX_STREAM_PERIOD);}

        // compute mean and variance and deviation - pitch
        delt_q = q_meas - q_meas1;
        q_mean += delt_q/(float)MAX_STREAM_PERIOD;
        q_var += delt_q*((q_meas - q_mean) + (q_meas1 - q_mean1));
        if (q_var >= 0.0){q_std = sqrtf(q_var/(float)MAX_STREAM_PERIOD);}

        // compute mean and variance and deviation - yaw
        delt_r = r_meas - r_meas1;
        r_mean += delt_r/(float)MAX_STREAM_PERIOD;
        r_var += delt_r*((r_meas - r_mean) + (r_meas1 - r_mean1));
        if (r_var >= 0.0){r_std = sqrtf(r_var/(float)MAX_STREAM_PERIOD);}

    }

    /// Enable reconfigurable control ONLY after FDD has begun
    if (start_rfc)
    {
        if (!rfc_man)
        {
            /// stability condition
            if ((p_std < 0.02) && (q_std < 0.02) && (r_std < 0.05))
            {
                rfc_counter++;
            }
            else /// reset if continous condition fails
            {
                rfc_counter = 0;
            }
        }
        else{
            rfc_counter++;
        }

        /// only do this once
        if (rfc_counter == MIN_PERIOD)
        {
            /// retrieve target lean angles values
            roll_cmd = amp*(float)wp_nav.get_roll();
            pitch_cmd = amp*(float)wp_nav.get_pitch();

            /// construct step commands
            wp_nav.set_rfc_roll(roll_cmd);
            wp_nav.set_rfc_pitch(pitch_cmd);

            /// start RFC maneuver
            rfc_man = 1;

            /// capture speed of RFC command
            if (rfc_speed1 < 1){rfc_speed1 = rfc_speed;} // do it once

            rfc_speed = gps.ground_speed();

            /// engage RFC commands
            wp_nav.set_rfc_cmd(rfc_man);

            /// compute scale direction due to pairing of motor spin direction
            /// faulty motor odd number - reverse scaling of n+1 motor
            /// faulty motor even number - reverse scaling of n-1 motor
            if ((F_loc+1)%2 == 0){scale_dir = -1;}
            else {scale_dir = 1;}

            /// compute the geometric sides of affected rotor - based on unit circle

            delt_gain = int_hold + PERT_AMP*pert_sign;

            compute_geom_sides(1,0,&aside0,&cside0);
            compute_geom_sides(1,delt_gain,&aside1,&cside1);

            delt_gain = constrain_float(delt_gain,0,0.5);

            for (i=0; i<sysid_num_motor; i++)
            {
                if (i == F_loc)
                {
                    rfc_gains[i] = cside0/cside1;
                }
                else if (i == F_loc + scale_dir)
                {
                    rfc_gains[i] = cside1/cside0;
                }
                else
                {
                    rfc_gains[i] = aside1/aside0;
                }
            }

            /// send RFC control allocation commands
            // motors.get_rfc_gain(rfc_gains);
        }

        // execute the cost function once the sys_id flag is LOW and counter has exceeded
        if ((rfc_counter > MIN_PERIOD) && (rfc_counter < MAX_PERIOD) && (rfc_speed > 0))
        {
            // J_the = J_the + (powf(p_meas - p_cmd,2) + powf(q_meas - q_cmd,2) + powf(r_meas - r_cmd,2))*G_Dt;
            J_the = J_the + (rfc_speed1/rfc_speed)*(powf((p_meas - p_cmd),2) + powf((q_meas - q_cmd),2))*G_Dt;
            J_p = J_p + (rfc_speed1/rfc_speed)*powf(p_meas - p_cmd,2)*G_Dt;
            J_q = J_q + (rfc_speed1/rfc_speed)*powf(q_meas - q_cmd,2)*G_Dt;
            J_r = J_r + (rfc_speed1/rfc_speed)*powf(r_meas - r_cmd,2)*G_Dt;

            filter_dtime += G_Dt; // increment filter time
        }

        /// reset RFC conditions
        if (rfc_counter == MAX_PERIOD)
        {
            wp_nav.set_rfc_cmd(0);

            J_the_hold = J_the;

            J_the = 0;

            J_p = 0;
            J_q = 0;
            J_r = 0;

            rfc_counter = 0;
            rfc_man = 0;

            /// demodulation signal - sinewave
            if (pert_sign == 1){pert_sign = -1;}
            else if (pert_sign == -1){pert_sign = 1;}

            /// reset filter sample time
            filter_dtime = 0;
        }

        /// compute filter output based on the hold value objective function
        exe_rfc_filter(&ES_HPF,J_the_hold,G_Dt);

        /// compute filter ouput based demodulated objective function
        exe_rfc_filter(&ES_LPF,ES_HPF.out*pert_sign*PERT_AMP,G_Dt);

        /// compute integrated filtered output
        int_hold += ES_LOOP_GAIN*(ES_LPF.out);

    }
}

void Copter::exe_rfc_filter(struct rfc_filter *filter, float input, float dt)
{
    float beta[2];

    uint8_t n;
    n = sizeof(filter->a) / sizeof(filter->a[0]);


    for (int i=0; i < n; i++)
    {
        // beta = b - b(1)*a;
        beta[i] = filter->b[i] - filter->b[0]*filter->a[i];
    }

    if (!filter->init_state)
    {
        // X(end) = u/beta(end);
        filter->X[1] = input/beta[1];
        filter->init_state = 1;
    }

    for (int i=1; i < n; i++)
    {
        if(i == 1)
        {
            filter->dX[i] = input;
            for (int j=1; j < n; j++)
            {
                filter->dX[i] = filter->dX[i] - filter->a[j]*filter->X[j];
            }
        }
        else
        {
            filter->dX[i] = filter->X[i - 1]; // state first derivative
        }
    }

    // Feedthrough term
    filter->out = filter->b[0]*input;

    for (int i=1; i<n; i++)
    {
        filter->out += beta[i]*filter->X[i];
    }

    // Update the state vector
    for (int i=1; i < n; i++)
    {
        filter->X[i] += (float)0.5*(filter->dX[i] + filter->dX1[i])*(dt);
        filter->dX1[i] = filter->dX[i];
    }
}

void Copter::compute_geom_sides(float delta, float gamma, float *a_side, float *c_side)
{
    float b_side;

    *a_side = (delta*(2*delta*delta - gamma*gamma + gamma*sqrtf(2*delta*delta - gamma*gamma)))/(delta*delta - gamma*gamma);

    b_side = (delta*(2*delta*delta - gamma*gamma - gamma*sqrtf(2*delta*delta - gamma*gamma)))/(delta*delta - gamma*gamma);

    *c_side = b_side; //sqrtf(*a_side * *a_side + b_side * b_side);
}
