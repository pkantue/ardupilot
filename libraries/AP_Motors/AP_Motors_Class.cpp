/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_Motors.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */

#include "AP_Motors_Class.h"
#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

// Constructor
AP_Motors::AP_Motors(uint16_t loop_rate, uint16_t speed_hz) :
    _loop_rate(loop_rate),
    _speed_hz(speed_hz),
    _roll_in(0.0f),
    _pitch_in(0.0f),
    _yaw_in(0.0f),
    _throttle_in(0.0f),
    _throttle_avg_max(0.0f),
    _throttle_filter(),
    _spool_desired(DESIRED_SHUT_DOWN),
    _batt_voltage(0.0f),
    _batt_current(0.0f),
    _air_density_ratio(1.0f),
    _motor_map_mask(0),
    _motor_fast_mask(0),
    _faulty_motor(0),
    _fault_type(0),
    _perc_loss(0),
    _sysid_mode(NO_SYSID)
{
    // init other flags
    _flags.armed = false;
    _flags.frame_orientation = AP_MOTORS_X_FRAME;
    _flags.interlock = false;

    // setup throttle filtering
    _throttle_filter.set_cutoff_frequency(0.0f);
    _throttle_filter.reset(0.0f);

    // init limit flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = true;
    limit.throttle_upper = true;
};

void AP_Motors::armed(bool arm)
{
    if (_flags.armed != arm) {
        _flags.armed = arm;
        AP_Notify::flags.armed = arm;
        if (!arm) {
            save_params_on_disarm();
        }
    }
};

// pilot input in the -1 ~ +1 range for roll, pitch and yaw. 0~1 range for throttle
void AP_Motors::set_radio_passthrough(float roll_input, float pitch_input, float throttle_input, float yaw_input)
{
    _roll_radio_passthrough = roll_input;
    _pitch_radio_passthrough = pitch_input;
    _throttle_radio_passthrough = throttle_input;
    _yaw_radio_passthrough = yaw_input;
}

/*
  write to an output channel
 */
void AP_Motors::rc_write(uint8_t chan, uint16_t pwm)
{
    if (_motor_map_mask & (1U<<chan)) {
        // we have a mapped motor number for this channel
        chan = _motor_map[chan];
    }
    if (_pwm_type == PWM_TYPE_ONESHOT125 && (_motor_fast_mask & (1U<<chan))) {
        // OneShot125 uses a PWM range from 125 to 250 usec
        pwm /= 8;
        /*
          OneShot125 ESCs can be confused by pulses below 125 or above
          250, making them fail the pulse type auto-detection. This
          happens at least with BLHeli
        */
        if (pwm < 125) {
            pwm = 125;
        } else if (pwm > 250) {
            pwm = 250;
        }
    }
    hal.rcout->write(chan, pwm);
}

/*
  set frequency of a set of channels
 */
void AP_Motors::rc_set_freq(uint32_t mask, uint16_t freq_hz)
{
    mask = rc_map_mask(mask);
    if (freq_hz > 50) {
        _motor_fast_mask |= mask;
    }
    hal.rcout->set_freq(mask, freq_hz);
    if ((_pwm_type == PWM_TYPE_ONESHOT ||
         _pwm_type == PWM_TYPE_ONESHOT125) &&
        freq_hz > 50 &&
        mask != 0) {
        // tell HAL to do immediate output
        hal.rcout->set_output_mode(AP_HAL::RCOutput::MODE_PWM_ONESHOT);
    }
}

void AP_Motors::rc_enable_ch(uint8_t chan)
{
    if (_motor_map_mask & (1U<<chan)) {
        // we have a mapped motor number for this channel
        chan = _motor_map[chan];
    }
    hal.rcout->enable_ch(chan);
}

/*
  map an internal motor mask to real motor mask
 */
uint32_t AP_Motors::rc_map_mask(uint32_t mask) const
{
    uint32_t mask2 = 0;
    for (uint8_t i=0; i<32; i++) {
        uint32_t bit = 1UL<<i;
        if (mask & bit) {
            if ((i < AP_MOTORS_MAX_NUM_MOTORS) && (_motor_map_mask & bit)) {
                // we have a mapped motor number for this channel
                mask2 |= (1UL << _motor_map[i]);
            } else {
                mask2 |= bit;
            }
        }
    }
    return mask2;
}

// convert input in -1 to +1 range to pwm output
int16_t AP_Motors::calc_pwm_output_1to1(float input, const RC_Channel& servo)
{
    int16_t ret;

    input = constrain_float(input, -1.0f, 1.0f);

    if (servo.get_reverse()) {
        input = -input;
    }

    if (input >= 0.0f) {
        ret = ((input * (servo.get_radio_max() - servo.get_radio_trim())) + servo.get_radio_trim());
    } else {
        ret = ((input * (servo.get_radio_trim() - servo.get_radio_min())) + servo.get_radio_trim());
    }

    return constrain_int16(ret, servo.get_radio_min(), servo.get_radio_max());
}

// convert input in 0 to +1 range to pwm output
int16_t AP_Motors::calc_pwm_output_0to1(float input, const RC_Channel& servo)
{
    int16_t ret;

    input = constrain_float(input, 0.0f, 1.0f);

    if (servo.get_reverse()) {
        input = 1.0f-input;
    }

    ret = input * (servo.get_radio_max() - servo.get_radio_min()) + servo.get_radio_min();

    return constrain_int16(ret, servo.get_radio_min(), servo.get_radio_max());
}

/*
  add a motor, setting up _motor_map and _motor_map_mask as needed
 */
void AP_Motors::add_motor_num(int8_t motor_num)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {
        uint8_t chan;
        if (RC_Channel_aux::find_channel((RC_Channel_aux::Aux_servo_function_t)(RC_Channel_aux::k_motor1+motor_num),
                                         chan)) {
            _motor_map[motor_num] = chan;
            _motor_map_mask |= 1U<<motor_num;
        } else {
            // disable this channel from being used by RC_Channel_aux
            RC_Channel_aux::disable_aux_channel(motor_num);
        }
    }
}

void AP_Motors::set_sysid_state(uint8_t mode, uint16_t Dt_in,
                                        uint16_t Dt_out, uint16_t Dt_step, float Amp,
                                        uint8_t rotor_loc, uint8_t num_rotor, uint8_t repeat, uint16_t Dt_rep)
{
    switch (mode){
        case (0):
            _sysid_mode = NO_SYSID;
        break;
        case (1):
            _sysid_mode = DOUBLET_SINGLE;
            break;
        case (2):
            _sysid_mode = MULTI_SINGLE;
            break;
        case (3):
            _sysid_mode = DOUBLET_ALL;
            break;
        case (4):
            _sysid_mode = MULTI_ALL;
            break;
    }

    _sysid_dtin = Dt_in;
    _sysid_dtout = Dt_out;
    _sysid_dtstep = Dt_step;
    _sysid_Amp = Amp;
    _sysid_rotor = rotor_loc; // rotor location
    _sysid_num_rotor = num_rotor; // airframe number of rotors (speed optimization)

    switch (repeat){
        case (0):
            _sysid_repeat = NO_REPEAT;
            break;
        case (1):
            _sysid_repeat = DO_REPEAT;
            break;
    }

    _sysid_dtrep = Dt_rep;
    _rotor_delay = Dt_rep/2; // it was half the timestep
}

void AP_Motors::sysid_exe(const float *cur_mot_values, float *new_mot_values)
{
    uint8_t num_steps;
    uint32_t steps_vec[10] = {0};
    float amp_vec[10] = {0.0};

    int i;

    _man_flag = 0;

    if (_sysid_mode == NO_SYSID)
    {
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++)
        {
            _hold_motor[i] = cur_mot_values[i];
            new_mot_values[i] = _hold_motor[i];
        }

        return; // exit function
    }

    if ((_sysid_mode == DOUBLET_SINGLE) || (_sysid_mode == DOUBLET_ALL))
    {
        num_steps = 4;

        steps_vec[0] = _sysid_dtin;
        steps_vec[1] = _sysid_dtin + (1)*_sysid_dtstep;
        steps_vec[2] = _sysid_dtin + (1 + 1)*_sysid_dtstep;
        steps_vec[3] = _sysid_dtin + (1 + 1)*_sysid_dtstep + _sysid_dtout;
        steps_vec[4] = 0;

        amp_vec[0] = 1;
        amp_vec[1] = 1*_sysid_Amp;
        amp_vec[2] = 1/_sysid_Amp;
        amp_vec[3] = 1;

    }

    if ((_sysid_mode == MULTI_SINGLE) || (_sysid_mode == MULTI_ALL))
    {

        num_steps = 7;

        steps_vec[0] = _sysid_dtin;
        steps_vec[1] = _sysid_dtin + (1)*_sysid_dtstep;
        steps_vec[2] = _sysid_dtin + (1 + 3)*_sysid_dtstep;
        steps_vec[3] = _sysid_dtin + (1 + 3 + 2)*_sysid_dtstep;
        steps_vec[4] = _sysid_dtin + (1 + 3 + 2 + 1)*_sysid_dtstep;
        steps_vec[5] = _sysid_dtin + (1 + 3 + 2 + 1 + 1)*_sysid_dtstep;
        steps_vec[6] = _sysid_dtin + (1 + 3 + 2 + 1 + 1)*_sysid_dtstep + _sysid_dtout;
        steps_vec[7] = 0;

        amp_vec[0] = 1;
        amp_vec[1] = 1*_sysid_Amp;
        amp_vec[2] = 1/_sysid_Amp;
        amp_vec[3] = 1*_sysid_Amp;
        amp_vec[4] = 1/_sysid_Amp;
        amp_vec[5] = 1*_sysid_Amp;
        amp_vec[6] = 1;
    }

    if ((_sysid_mode == DOUBLET_SINGLE) || (_sysid_mode == MULTI_SINGLE))
    {
        _num_rotor_counter = _sysid_rotor;
        _sysid_num_rotor = _num_rotor_counter;
    }

    // hold current value ONCE
    if (_hold_val_flag < 1)
    {
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++)
        {
            _hold_motor[i] = cur_mot_values[i];
        }
        _hold_val_flag = 1;
        _repeat_counter = 0;
        _man_counter = 0;
    }

    _num_rotor_counter = constrain_float(_num_rotor_counter,0,AP_MOTORS_MAX_NUM_MOTORS);
    _num_steps_counter = constrain_float(_num_steps_counter,0,num_steps);

    if (_man_counter < steps_vec[_num_steps_counter])
    {
        // output the holding values for all rotors
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++)
        {
            new_mot_values[i] = _hold_motor[i];
        }

        new_mot_values[_num_rotor_counter] = amp_vec[_num_steps_counter]*_hold_motor[_num_rotor_counter];

        _man_counter++;
        _man_flag = 1; // maneuver is active
    }
    else if (_num_steps_counter < num_steps)
    {
		// output the holding values for all rotors
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++)
        {
            new_mot_values[i] = _hold_motor[i];
        }

        new_mot_values[_num_rotor_counter] = amp_vec[_num_steps_counter]*_hold_motor[_num_rotor_counter];

        _num_steps_counter++;
        _man_flag = 1; // maneuver is active
    }
    else if (_sysid_repeat)
    {
        if (_num_rotor_counter < (_sysid_num_rotor-1))
        {
            _rotor_delay_counter++; // this is done to ensure the controller regains stability/control

            if (_rotor_delay_counter > _rotor_delay)
            {
                _num_rotor_counter++;
                _man_counter = 0;
                _hold_val_flag = 0;
                _rotor_delay_counter = 0;
                _num_steps_counter = 0;

				// passthrough controller values
				for (i = 0; i<AP_MOTORS_MAX_NUM_MOTORS; i++)
				{
					new_mot_values[i] = cur_mot_values[i];
				}

            }
            else
            {
                // passthrough controller values
                for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++)
                {
                    new_mot_values[i] = cur_mot_values[i];
                }
            }
        }
        else if (_repeat_counter < _sysid_dtrep)
        {
            // passthrough controller values
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++)
            {
                new_mot_values[i] = cur_mot_values[i];
            }
            _repeat_counter++;
        }
        else // repeat delay period has lapsed reset all counters
        {
            // passthrough controller values
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++)
            {
                new_mot_values[i] = cur_mot_values[i];
            }
            _repeat_counter = 0;
            _hold_val_flag = 0;
            _man_counter = 0;
            _rotor_delay_counter = 0;
            _num_steps_counter = 0;
			_num_rotor_counter = 0;
        }
    }
    else
    {
        if (_num_rotor_counter < (_sysid_num_rotor-1))
        {
            _rotor_delay_counter++; // this is done to ensure the controller regains stability/control

            if (_rotor_delay_counter > _rotor_delay)
            {
                _num_rotor_counter++;
                _man_counter = 0;
                _hold_val_flag = 0;
                _rotor_delay_counter = 0;
                _num_steps_counter = 0;

				// passthrough controller values
				for (i = 0; i<AP_MOTORS_MAX_NUM_MOTORS; i++)
				{
					new_mot_values[i] = cur_mot_values[i];
				}
            }
            else
            {
                // passthrough controller values
                for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++)
                {
                    new_mot_values[i] = cur_mot_values[i];
                }
            }
        }
        else
        {
            // passthrough controller values
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++)
            {
                new_mot_values[i] = cur_mot_values[i];
            }
        }
    }
}


// This function is to initialize the servo fault dynamics required
void AP_Motors::ini_servo_fault(void)
{
    float wn = 0;
    float zeta = (float)0.89; // damping ratio

    float wn_fault = (float)6.4;
    float wn_healthy = 192;

    if (_fault_type)
    {
        wn = wn_fault*_perc_loss + wn_healthy*(1 - _perc_loss);
    }
    else {
        wn = wn_healthy;
    }

    servo_param.a[0] = 1;
    servo_param.a[1] = 2*zeta*wn;
    servo_param.a[2] = wn*wn;

    servo_param.b[0] = 0*wn*wn;
    servo_param.b[1] = 0*wn*wn;
    servo_param.b[2] = 1*wn*wn;

    for (int i=0; i < 3; i++){
        servo_param.X[i] = 0;
        servo_param.dX[i] = 0;
        servo_param.dX1[i] = 0;
    }

    servo_param.in = 0;
    servo_param.out = 0;
    servo_param.init_state = 0;

}

// This function is to execute the servo fault emulation algorithm
float AP_Motors::exe_servo_fault(float input)
{
    float wn = 0;
    float zeta = (float)0.89; // damping ratio

    float wn_fault = (float)6.4;
    float wn_healthy = 192;
    float beta[3];

    uint8_t n;
    n = sizeof(servo_param.a) / sizeof(servo_param.a[0]);

    if (_fault_type)
    {
        wn = wn_fault*_perc_loss + wn_healthy*(1 - _perc_loss);
    }
    else {
        wn = wn_healthy;
    }

    servo_param.a[0] = 1;
    servo_param.a[1] = 2*zeta*wn;
    servo_param.a[2] = wn*wn;

    servo_param.b[0] = 0*wn*wn;
    servo_param.b[1] = 0*wn*wn;
    servo_param.b[2] = 1*wn*wn;

    for (int i=0; i < n; i++)
    {
        // beta = b - b(1)*a;
        beta[i] = servo_param.b[i] - servo_param.b[0]*servo_param.a[i];
    }

    if (!servo_param.init_state)
    {
        // X(end) = u/beta(end);
        servo_param.X[2] = input/beta[2];
        servo_param.init_state = 1;
    }

    /*
     * for i = 2:length(a)
     * if (i == 2)
     * dX(i) = u; % state second derivative
     * for j = 2:length(a)
     * dX(i) = dX(i) - a(j)*X(j);
     * end
     * else
     * dX(i) = X(i - 1); % state first derivative
     * end
     * end
    */
    for (int i=1; i < n; i++)
    {
        if(i == 1)
        {
            servo_param.dX[i] = input;
            for (int j=1; j < n; j++)
            {
                servo_param.dX[i] = servo_param.dX[i] - servo_param.a[j]*servo_param.X[j];
            }
        }
        else
        {
            servo_param.dX[i] = servo_param.X[i - 1]; // state first derivative
        }
    }
    /*
    y = b(1)*u; % feed through term
    for (i = 2:length(a))
        y = y + beta(i)*X(i);
    end
    output(k) = y;
    */

    // Feedthrough term
    servo_param.out = servo_param.b[0]*input;

    for (int i=1; i<n; i++)
    {
        servo_param.out += beta[i]*servo_param.X[i];
    }

    /*
    % update state vector
    for i = 2:length(a)
        X(i)   = X(i) + 0.5*(dX(i) + dX1(i))*dt;
        dX1(i) = dX(i);
    end
    */

    // Update the state vector
    for (int i=1; i < n; i++)
    {
        servo_param.X[i] += (float)0.5*(servo_param.dX[i] + servo_param.dX1[i])*(1.0f/_speed_hz);
        servo_param.dX1[i] = servo_param.dX[i];
    }

    return servo_param.out;
}
