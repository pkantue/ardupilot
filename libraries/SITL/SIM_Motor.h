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
  simple electric motor simulation class
*/

#pragma once

/** local variables */
#define INTERP_ROWS		4
#define INTERP_COLS		8

#include "SIM_Aircraft.h"
#include "h1_quad_definitions.h"

namespace SITL {

/*
  class to describe a motor position
 */
class Motor {
public:
    float angle;
    float yaw_factor;
    uint8_t servo;
    uint8_t display_order;

    // support for tilting motors
    int8_t roll_servo = -1;
    float roll_min, roll_max;
    int8_t pitch_servo = -1;
    float pitch_min, pitch_max;

	# include "struct_def.h"

	aero_struct motor_aero;				// data structure for each rotor
    kine_struct motor_kine;			// kinematic structure for each rotor

    // support for servo slew rate
    enum {SERVO_NORMAL, SERVO_RETRACT} servo_type;
    float servo_rate = 0.24; // seconds per 60 degrees
    uint64_t last_change_usec;
    float last_roll_value, last_pitch_value;

    Motor(uint8_t _servo, float _angle, float _yaw_factor, uint8_t _display_order) :
        servo(_servo), // what servo output drives this motor
        angle(_angle), // angle in degrees from front
        yaw_factor(_yaw_factor), // positive is clockwise
        display_order(_display_order) // order for clockwise display
    {}

    /*
      alternative constructor for tiltable motors
     */
    Motor(uint8_t _servo, float _angle, float _yaw_factor, uint8_t _display_order,
          int8_t _roll_servo, float _roll_min, float _roll_max,
          int8_t _pitch_servo, float _pitch_min, float _pitch_max) :
        servo(_servo), // what servo output drives this motor
        angle(_angle), // angle in degrees from front
        yaw_factor(_yaw_factor), // positive is clockwise
        display_order(_display_order), // order for clockwise display
        roll_servo(_roll_servo),
        roll_min(_roll_min),
        roll_max(_roll_max),
        pitch_servo(_pitch_servo),
        pitch_min(_pitch_min),
        pitch_max(_pitch_max)
    {}

    void calculate_forces(const Aircraft::sitl_input &input,
                          float thrust_scale,
                          uint8_t motor_offset,
                          Vector3f &rot_accel, // rad/sec
                          Vector3f &body_thrust, // Z is down
                          aero_struct *quad_lan,
                          kine_struct *kine_lan,
                          const float dTime);

    uint16_t update_servo(uint16_t demand, uint64_t time_usec, float &last_value);

    // additional functions
    double data_interp(const double data[INTERP_ROWS][INTERP_COLS],
	                   const double row_array[INTERP_ROWS],
	                   const double col_array[INTERP_COLS],
	                   double row_input,
	                   double col_input);

    void calc_rotor(aero_struct *quad_lan, kine_struct *kine_lan, const double Ts);
	void calc_rotorpseed(aero_struct *quad_lan, kine_struct *kine_lan, const double Ts);
	void calc_rotor_flap_dynamics(aero_struct *quad_lan, const kine_struct *kine_lan, const double Ts);
	void TransferFunc(const double input, aero_struct *rotor, const double dT);
	void TransformKine(kine_struct * kine_lan, const Vector3f arm);


private:
    // variables internal to each rotor
    const double adv_ratio[INTERP_COLS] = { 0.00f, 0.09f, 0.17f, 0.26f, 0.35f, 0.43f, 0.52f, 0.61f};
    const double rpm[INTERP_ROWS] = { 4000.00f, 5000.00f, 6000.00f, 6500.00f };
    const double CT_data[INTERP_ROWS][INTERP_COLS] = {
		{ 0.1156f, 0.1057f, 0.0949f, 0.0819f, 0.0677f, 0.0509f, 0.0317f, 0.0093f },
		{ 0.1229f, 0.1114f, 0.0990f, 0.0858f, 0.0712f, 0.0543f, 0.0344f, 0.0114f },
		{ 0.1279f, 0.1180f, 0.1040f, 0.0895f, 0.0748f, 0.0571f, 0.0370f, 0.0140f },
		{ 0.1298f, 0.1203f, 0.1063f, 0.0906f, 0.0760f, 0.0583f, 0.0384f, 0.0149f } };

    const double CQ_data[INTERP_ROWS][INTERP_COLS] = {
		{ 0.0074f, 0.0073f, 0.0071f, 0.0068f, 0.0063f, 0.0056f, 0.0048f, 0.0037f },
		{ 0.0079f, 0.0077f, 0.0074f, 0.0070f, 0.0066f, 0.0059f, 0.0049f, 0.0038f },
		{ 0.0083f, 0.0081f, 0.0078f, 0.0073f, 0.0069f, 0.0061f, 0.0051f, 0.0039f },
		{ 0.0084f, 0.0083f, 0.0080f, 0.0074f, 0.0070f, 0.0062f, 0.0052f, 0.0040f } };

};
}
