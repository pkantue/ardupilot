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
  simple electric motor simulator class
*/

#include "SIM_Motor.h"
#include <AP_Motors/AP_Motors.h>

using namespace SITL;

// calculate rotational accel and thrust for a motor
void Motor::calculate_forces(const Aircraft::sitl_input &input,
                             const float thrust_scale,
                             uint8_t motor_offset,
                             Vector3f &rot_accel,
                             Vector3f &thrust,
                             aero_struct *quad_lan,
                             kine_struct *kine_lan,
                             const float dTime)
{
    // fudge factors
    //const float arm_scale = radians(5000);
    //const float yaw_scale = radians(400);
    const float arm_scale = sqrtf(ARM_XCG*ARM_XCG + ARM_YCG*ARM_YCG);

    // get motor speed from 0 to 1
    float motor_speed = constrain_float((input.servos[motor_offset+servo]-1100)/900.0, 0, 1);

    // assign motor_speed to aero_struct
    quad_lan->dmotor = 0.5*motor_speed;

    // define the arm position relative to center of mass
    Vector3f arm(arm_scale * cosf(radians(angle)), arm_scale * sinf(radians(angle)), 0);

    //transform kinematics to each rotor position
    TransformKine(kine_lan,arm);

    calc_rotor(quad_lan,kine_lan,dTime);

    // the yaw torque of the motor
    //Vector3f rotor_torque(0, 0, yaw_factor * motor_speed * yaw_scale);
    Vector3f rotor_torque(0, 0, yaw_factor * quad_lan->Q_mr);

    // get thrust for untilted motor
    //thrust(0, 0, -motor_speed);
    //thrust(0, 0, -quad_lan->T_mr); // *cosf(quad_lan->a_rad)*cosf(quad_lan->b_rad));
    thrust(-quad_lan->T_mr*sinf(quad_lan->a_rad), -quad_lan->T_mr*sinf(quad_lan->b_rad), -quad_lan->T_mr*cosf(quad_lan->a_rad)*cosf(quad_lan->b_rad));

    // work out roll and pitch of motor relative to it pointing straight up
    float roll = 0, pitch = 0;

    uint64_t now = AP_HAL::micros64();

    // possibly roll and/or pitch the motor
    if (roll_servo >= 0) {
        uint16_t servoval = update_servo(input.servos[roll_servo+motor_offset], now, last_roll_value);
        if (roll_min < roll_max) {
            roll = constrain_float(roll_min + (servoval-1000)*0.001*(roll_max-roll_min), roll_min, roll_max);
        } else {
            roll = constrain_float(roll_max + (2000-servoval)*0.001*(roll_min-roll_max), roll_max, roll_min);
        }
    }
    if (pitch_servo >= 0) {
        uint16_t servoval = update_servo(input.servos[pitch_servo+motor_offset], now, last_pitch_value);
        if (pitch_min < pitch_max) {
            pitch = constrain_float(pitch_min + (servoval-1000)*0.001*(pitch_max-pitch_min), pitch_min, pitch_max);
        } else {
            pitch = constrain_float(pitch_max + (2000-servoval)*0.001*(pitch_min-pitch_max), pitch_max, pitch_min);
        }
    }
    last_change_usec = now;

    // possibly rotate the thrust vector and the rotor torque
    if (!is_zero(roll) || !is_zero(pitch)) {
        Matrix3f rotation;
        rotation.from_euler(radians(roll), radians(pitch), 0);
        thrust = rotation * thrust;
        rotor_torque = rotation * rotor_torque;
    }

    // calculate total rotational acceleration
    rot_accel = (arm % thrust) + rotor_torque;

    // scale the thrust
    // thrust = thrust * thrust_scale;
}

/*
  update and return current value of a servo. Calculated as 1000..2000
 */
uint16_t Motor::update_servo(uint16_t demand, uint64_t time_usec, float &last_value)
{
    if (servo_rate <= 0) {
        return demand;
    }
    if (servo_type == SERVO_RETRACT) {
        // handle retract servos
        if (demand > 1700) {
            demand = 2000;
        } else if (demand < 1300) {
            demand = 1000;
        } else {
            demand = last_value;
        }
    }
    demand = constrain_int16(demand, 1000, 2000);
    float dt = (time_usec - last_change_usec) * 1.0e-6f;
    // assume servo moves through 90 degrees over 1000 to 2000
    float max_change = 1000 * (dt / servo_rate) * 60.0f / 90.0f;
    last_value = constrain_float(demand, last_value-max_change, last_value+max_change);
    return uint16_t(last_value+0.5);
}

/**
 *  \brief Compute Main Rotor Forces & Moments
 *
 *  \param [in] quad_lan Parameter_Description
 *  \param [in] kine_lan Parameter_Description
 *  \param [in] Ts sample time [s]
 *  \return None
 *
 *  \details Compute Main Rotor Forces & Moments
 */
void Motor::calc_rotor(aero_struct *quad_lan, kine_struct *kine_lan, const double Ts)
{

     double omega;
     double mu_z;
     double mu_mr;

     double temp_hj;
     double CT_mr, CQ_mr;
     // double T_max;
     double LAMBDA;
     double sig_mr;

     calc_rotorpseed(quad_lan,kine_lan,Ts);

	 // set final value before ESC
	 omega = quad_lan->omega;

	 if (omega != 0.0){
		 mu_mr = (double)sqrt((kine_lan->vibb.x)*(kine_lan->vibb.x) + (kine_lan->vibb.y)*(kine_lan->vibb.y)) / (omega*R_mr);
		 mu_z = (kine_lan->vibb.z) / (omega*R_mr);
	 }
	 else{
		 mu_mr = 0.0;
		 mu_z = 0.0;
	 }

     sig_mr = (2*c_mr)/(PI*R_mr);

     // interpolate to get wind tunnel propeller data
	 CT_mr = data_interp(CT_data,rpm,adv_ratio,omega,mu_z);
     CQ_mr = data_interp(CQ_data,rpm,adv_ratio,omega,mu_z);

	 //CT_mr = 0.5*a_mr*sig_mr*(delt_col*(1.0 / 3.0 + 0.5*mu_mr*mu_mr) + 0.5*(mu_z - quad_lan->lambda0));
	 LAMBDA = mu_mr*mu_mr + (quad_lan->lambda0 - mu_z)*(quad_lan->lambda0 - mu_z);

	 /// should be executed only once
	 if (LAMBDA == 0)
		 LAMBDA = sqrt(CT_mr / (2 * eta_w));

	 /// No update of inflow velocity if thrust coefficient is below threshold value
	 if (fabs(CT_mr) > 1e-5)
	 {
		 temp_hj = -((2 * eta_w*quad_lan->lambda0*sqrt(LAMBDA) - CT_mr)*LAMBDA) / (2 * eta_w*pow(LAMBDA, 3.0 / 2.0) + 0.25*sig_mr*a_mr*LAMBDA - CT_mr*(mu_z - quad_lan->lambda0));
		 quad_lan->lambda0 = quad_lan->lambda0 + 0.6*temp_hj;
	 }

     /// limiting CT_mr - quadcopter propeller can't produce negative lift through change in angle of attack
     //CT_mr = CSlimit(-C_mr_Tmax,C_mr_Tmax,CT_mr);
     CT_mr = constrain_float(CT_mr,-C_mr_Tmax,C_mr_Tmax);

     /// calculate T_mr
     quad_lan->T_mr = kine_lan->rho*(omega*R_mr)*(omega*R_mr)*PI*R_mr*R_mr*CT_mr;

     /// Calculate induced velocity
	 quad_lan->V_imr = sqrt(quad_lan->T_mr/ (2 * kine_lan->rho*PI*R_mr*R_mr));

     // calculate Q_mr
	 //CQ_mr = CT_mr*(quad_lan->lambda0 - mu_z) + (C_mr_Do*sig_mr / 8)*(1 + (7 / 3)*mu_mr*mu_mr);

     quad_lan->Q_mr = CQ_mr*kine_lan->rho*(omega*R_mr)*(omega*R_mr)*PI*pow(R_mr,3);

     calc_rotor_flap_dynamics(quad_lan,kine_lan,Ts);

     //fptr = fopen("main_rotor.txt", "a");
     //fprintf(fptr, "%5.3f %5.3f %5.3f \n",);

 }

void Motor::calc_rotorpseed(aero_struct *quad_lan, kine_struct *kine_lan, const double Ts)
{

    //double Q_e_mr;
    double defl_t; /// put into pointer structure
    // double omega;
	double omega_cmd;
	double omega_max;
	double alph = (tf_wn/1200.0)/(tf_wn/1200.0 + 1);
	double temp;
	//double mu_z;

	//dmotor = CSlimit(0, MAX_THRUST, quad_lan->dmotor);

	// dyn_lim = MAX_THRUST - dmotor;
	// low_lim = dmotor*dmotor*dmotor; // cubic continuous dynamic limit

    // dlat = CSlimit(-ROLL_LIM, ROLL_LIM, quad_lan->dlat);
    // dlon = CSlimit(-PITCH_LIM, PITCH_LIM, quad_lan->dlon);
    // drudd = CSlimit(-YAW_LIM, YAW_LIM, quad_lan->drudd);

	// tot_defl = dlat + dlon + drudd;

	// defl_t = dmotor + dlat + dlon + drudd; // value between 0 and 1
	defl_t = quad_lan->dmotor; //dmotor*(1 + dlat)*(1 + dlon)*(1 + drudd); // value between 0 and 1

	omega_max = motor_KV*max_volt; // / (gear_mr*9.5493);
	omega_cmd = motor_KV*defl_t*max_volt;// / (gear_mr*9.5493); // conversion from RPM to rad/s (main rotor)

	// limit omega within range
	// omega_cmd = CSlimit(0,omega_max,omega_cmd);
	omega_cmd = constrain_float(omega_cmd,0,omega_max);

	// Compute rotor dynamics based on rotor/propeller mechanics (including faults)
	// TransferFunc(omega_cmd, quad_lan, Ts);

    temp = alph*omega_cmd + (1.0 - alph)*quad_lan->omega_out;
	quad_lan->omega_out = temp;

	// protection against negative rotation (potentially)
	if (quad_lan->omega_out < 0.0){
		quad_lan->omega_out = 0.0;
	}

	quad_lan->omega = omega_cmd/9.5493;
	// quad_lan->omega = quad_lan->omega_out/9.5493;

}

/**
 *  \brief Compute Rotor flapping angles
 *
 *  \param [in] quad_lan Parameter_Description
 *  \param [in] kine_lan Parameter_Description
 *  \param [in] Ts sample time [s]
 *  \return xx
 *
 *  \details Compute Rotor flapping angles
 */
 void Motor::calc_rotor_flap_dynamics(aero_struct *quad_lan, const kine_struct *kine_lan, const double Ts)
 {
     double mu_mr, sig_mr;
     double da1_dmu;
     double db1_muv;
     double da1_muz;
     //double dlat, dlon,
     double dcol;
     double tau_e;
     // double Blat, Alon;
     //double b_dot1, a_dot1;
     double omega;

     //dlat = quad_lan->dlat;
     //dlon = quad_lan->dlon;
     dcol = quad_lan->dcol;
     omega = quad_lan->omega;

	 if (omega != 0.0){
		 mu_mr = (double)sqrt((kine_lan->vibb.x)*(kine_lan->vibb.x) + (kine_lan->vibb.y)*(kine_lan->vibb.y)) / (omega*R_mr);
		 //mu_z = (kine_lan->vibb.z) / (omega*R_mr);
	 }
	 else{
		 mu_mr = 0.0;
		 //mu_z = 0.0;
	 }

     sig_mr = (2*c_mr)/(PI*R_mr);

	 da1_dmu = 2 * K_u*(4 * dcol / 3 - quad_lan->lambda0);
     db1_muv = -da1_dmu;

     /// There's no flybar lag this doesn't apply
     /// tau_e = 16/(gam_fb*quad_lan->omega);
     tau_e = 0.9;

     if (mu_mr > 0)
        da1_muz = K_u*((16*mu_mr*mu_mr)/((1 - mu_mr*mu_mr/2)*(8*fabs(mu_mr) + sig_mr*a_mr)));
     //else if (mu_mr < 0)
     else
        da1_muz = -K_u*((16*mu_mr*mu_mr)/((1 - mu_mr*mu_mr/2)*(8*fabs(mu_mr) + sig_mr*a_mr)));

     //Blat = B_nom_lat*(quad_lan->omega/Omeg_nom)*(quad_lan->omega/Omeg_nom);
     //Alon = A_nom_lon*(quad_lan->omega/Omeg_nom)*(quad_lan->omega/Omeg_nom);

     /// save the last value prior to update
	 quad_lan->a_dot1 = quad_lan->a_dot;
	 quad_lan->b_dot1 = quad_lan->b_dot;

	 /// only execute if motor started turning
	 if (quad_lan->omega != 0.0){
		 quad_lan->b_dot = -kine_lan->wibb.x - quad_lan->b_rad / tau_e - (1 / tau_e)*db1_muv*kine_lan->vibb.y / (quad_lan->omega);// + Blat*dlat / tau_e;
		 quad_lan->a_dot = -kine_lan->wibb.y - quad_lan->a_rad / tau_e - (1 / tau_e)*(da1_dmu*kine_lan->vibb.x / (quad_lan->omega) + da1_muz*kine_lan->vibb.z / (quad_lan->omega));// + Alon*dlon / tau_e;
	 }

	 // Moved to kine_lan for modularity between simulink and script
	 quad_lan->b_rad = quad_lan->b_rad + 0.5*Ts*(3.0*quad_lan->b_dot - quad_lan->b_dot1);
	 quad_lan->a_rad = quad_lan->a_rad + 0.5*Ts*(3.0*quad_lan->a_dot - quad_lan->a_dot1);

 }

/*
    interpolation function for torque and thrust coefficients
*/
double Motor::data_interp(const double data[INTERP_ROWS][INTERP_COLS],
	                  const double row_array[INTERP_ROWS],
	                  const double col_array[INTERP_COLS],
	                  double row_input,
	                  double col_input)
{

	unsigned int row_index;
	unsigned int col_index;
	unsigned int i;

	double row_offset;
	double col_offset;

	// interpolation vertices
	double row1, row2;
	double col1;

    row_index   = 0u;
    col_index   = 0u;

	// Limit the input values to within the coefficient database
	// row_input = CSlimit(row_array[0], row_array[INTERP_ROWS - 1],row_input);
	row_input = constrain_float(row_input, row_array[0], row_array[INTERP_ROWS - 1]);
	col_input = constrain_float(col_input, col_array[0], col_array[INTERP_COLS - 1]);

	// compute offsets within the various arrays
	for (i = 0; i < (INTERP_ROWS - 1); i++)
	{
		if ((row_input >= row_array[i]) && (row_input <= row_array[i + 1]))
		{
			row_index = i;
			break;
		}
	}

	// compute offsets within the various arrays
	for (i = 0; i < (INTERP_COLS - 1); i++)
	{
		if ((col_input >= col_array[i]) && (col_input <= col_array[i + 1]))
		{
			col_index = i;
			break;
		}
	}

	// compute amount of offset from the identified array index
	row_offset = (row_input - row_array[row_index]) / (row_array[row_index + 1] - row_array[row_index]);
	col_offset = (col_input - col_array[col_index]) / (col_array[col_index + 1] - col_array[col_index]);

	// perform first interpolation along the row arrays
	row1 = data[row_index][col_index] * (1 - row_offset) + data[row_index+1][col_index] * row_offset;

	row2 = data[row_index][col_index+1] * (1 - row_offset) + data[row_index+1][col_index+1] * row_offset;

	// perform second interpolation along the column array
	col1 = row1*(1 - col_offset) + row2*col_offset;

	return(col1);
}


/**
*  \brief This function transforms kinematics to rotor location
*
*  \param [in] input value
*  \param [in/out] structure holding transfer function parameters
*  \param [in] sample time
*  \return None
*
*/
void Motor::TransformKine(kine_struct * kine_lan, const Vector3f arm)
{
    Vector3f vibb(kine_lan->vibb.x,kine_lan->vibb.y,kine_lan->vibb.z);
    Vector3f wibb(kine_lan->wibb.x,kine_lan->wibb.y,kine_lan->wibb.z);

    vibb = vibb - arm % wibb; //cross product

    //allocate kine_struct
    kine_lan->vibb.x = vibb.x;
    kine_lan->vibb.y = vibb.y;
    kine_lan->vibb.z = vibb.z;
}

/**
*  \brief This function computes the continuous transfer function (constant coeff)
*
*  \param [in] input value
*  \param [in/out] structure holding transfer function parameters
*  \param [in] sample time
*  \return None
*
*/
void Motor::TransferFunc(const double input, aero_struct *rotor, const double dT)
{
	int i,j,n;

	double a[10] = {0.0};
	double b[10] = {0.0};
	double beta[10] = {0.0};
	double dX[10] = {0.0};
	double dX1[10] = {0.0};
	double X[10] = {0.0};
	double u,y;

	n = rotor->nIn;

	beta[0] = 0.0;

	// assign from external structure
	for (i = 0; i <= n; i++)
	{
		X[i] = rotor->Xs[i];
		dX[i] = rotor->dXs[i];
		dX1[i] = rotor->dX1s[i];
		b[i] = rotor->bIn[i];
		a[i] = rotor->aIn[i];
	}

	// calculate long division coefficients
	for (i = 1; i <= n; i++)
	{
		beta[i] = b[i] - b[0]*a[i];
	}

	// save input
	u = input;

	// calculate state derivatives
	for (i = 1; i <= n; i++)
	{
		if (i == 1)
		{
			dX[i] = u;
			for (j = 1; j <= n; j++)
			{
				dX[i] = dX[i] - a[j] * X[j];
			}
		}
		else
		{
			dX[i] = X[i - 1];
		}
	}

	// calculate output
	y = b[0] * u; // feed through term
	for (i = 1; i <= n; i++)
	{
		y = y + beta[i] * X[i];
	}

	// update state with trapezoidal integration method
	for (i = 1; i <= n; i++)
	{
		X[i] = X[i] + 0.5*(dX[i] + dX1[i])*dT;
		dX1[i] = dX[i]; // memory
	}

	rotor->omega_out = y;

	// assign to external structure
	for (i = 0; i <= n; i++)
	{
		rotor->Xs[i] = X[i];
		rotor->dXs[i] = dX[i];
		rotor->dX1s[i] = dX1[i];
	}
}

