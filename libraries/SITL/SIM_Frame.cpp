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
  multicopter frame simulator class
*/

#include "SIM_Frame.h"
#include <AP_Motors/AP_Motors.h>

#include <stdio.h>

using namespace SITL;

static Motor quad_plus_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_3,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
};

static Motor quad_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
};

static Motor hexa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_2, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_3,-120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_4,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_5, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_6, 120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)
};

static Motor hexax_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_3, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_4, 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_5,  30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_6,-150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4)
};

static Motor octa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,    0,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_2,  180,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_3,   45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_4,  135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_5,  -45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8),
    Motor(AP_MOTORS_MOT_6, -135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_7,  -90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7),
    Motor(AP_MOTORS_MOT_8,   90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)
};

static Motor octa_quad_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
    Motor(AP_MOTORS_MOT_5,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8),
    Motor(AP_MOTORS_MOT_6,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_7,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_8, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6)
};

static Motor tri_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, AP_MOTORS_MOT_7, 60, -60, -1, 0, 0),
};

static Motor tilttri_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, -1, 0, 0, AP_MOTORS_MOT_8, 0, -90),
    Motor(AP_MOTORS_MOT_2,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, AP_MOTORS_MOT_8, 0, -90),
    Motor(AP_MOTORS_MOT_4,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, AP_MOTORS_MOT_7, 60, -60, -1, 0, 0),
};

static Motor y6_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_2, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_3, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_6, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3)
};

/*
  FireflyY6 is a Y6 with front motors tiltable using servo on channel 9 (output 8)
 */
static Motor firefly_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, -1, 0, 0, 8, 0, -90),
    Motor(AP_MOTORS_MOT_2, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5, -1, 0, 0, 8, 0, -90),
    Motor(AP_MOTORS_MOT_3, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6, -1, 0, 0, 8, 0, -90),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1, -1, 0, 0, 8, 0, -90),
    Motor(AP_MOTORS_MOT_6, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3)
};

/*
  table of supported frame types. String order is important for
  partial name matching
 */
static Frame supported_frames[] =
{
    Frame("+",         4, quad_plus_motors),
    Frame("quad",      4, quad_plus_motors),
    Frame("copter",    4, quad_plus_motors),
    Frame("x",         4, quad_x_motors),
    Frame("hexax",     6, hexax_motors),
    Frame("hexa",      6, hexa_motors),
    Frame("octa-quad", 8, octa_quad_motors),
    Frame("octa",      8, octa_motors),
    Frame("tri",       3, tri_motors),
    Frame("tilttri",   3, tilttri_motors),
    Frame("y6",        6, y6_motors),
    Frame("firefly",   6, firefly_motors)
};

void Frame::init(float _mass, float hover_throttle, float _terminal_velocity, float _terminal_rotation_rate)
{
    mass = _mass;
    inertia(IXX,IYY,IZZ); //initialize the inertia tensor (diagonal only)

    /*
       scaling from total motor power to Newtons. Allows the copter
       to hover against gravity when each motor is at hover_throttle
    */
    thrust_scale = (mass * GRAVITY_MSS) / (num_motors * hover_throttle);

    terminal_velocity = _terminal_velocity;
    terminal_rotation_rate = _terminal_rotation_rate;

    // initialize the aero structures
    for (uint8_t i = 0; i < num_motors; i++)
    {
        initStruct(&motors[i].motor_aero,&motors[i].motor_kine);
    }
}

/*
  find a frame by name
 */
Frame *Frame::find_frame(const char *name)
{
    for (uint8_t i=0; i < ARRAY_SIZE(supported_frames); i++) {
        // do partial name matching to allow for frame variants
        if (strncasecmp(name, supported_frames[i].name, strlen(supported_frames[i].name)) == 0) {
            return &supported_frames[i];
        }
    }
    return NULL;
}

// calculate rotational and linear accelerations
void Frame::calculate_forces(const Aircraft &aircraft,
                             const Aircraft::sitl_input &input,
                             Vector3f &rot_accel,
                             Vector3f &body_accel)
{
    Vector3f thrust; // newtons
    const Matrix3f &dcm = aircraft.get_dcm();
    const Vector3f &vibb = dcm.transposed() * aircraft.get_velocity_air_ef();
    const Vector3f &gyro = aircraft.get_gyro();
    float rho;
    Vector3f vec_sign;

    dT = 1/aircraft.get_rate_hz(); //seconds

    for (uint8_t i=0; i<num_motors; i++) {
        Vector3f mraccel, mthrust;
        updateStruct(aircraft,&motors[i].motor_kine);
        motors[i].calculate_forces(input, thrust_scale, motor_offset, mraccel, mthrust,&motors[i].motor_aero,&motors[i].motor_kine,dT);
        rot_accel += mraccel;
        thrust += mthrust;
    }

    body_accel = thrust/mass;


    // compute full nonlinear dynamics on translational and rotational dynamics
    vec_sign = getSign(vibb);
    rho = motors[0].motor_kine.rho;


    body_accel.x = vibb.y*gyro.z - vibb.z*gyro.y + (thrust.x - 0.5*rho*vec_sign.x*vibb.x*vibb.x*A_FRONT*CD_0)/mass;
    body_accel.y = vibb.z*gyro.x - vibb.x*gyro.z + (thrust.y - 0.5*rho*vec_sign.y*vibb.y*vibb.y*A_SIDE*CD_0)/mass;
    body_accel.z = vibb.x*gyro.y - vibb.y*gyro.x + (thrust.z - 0.5*rho*vec_sign.z*vibb.z*vibb.z*A_BACK*CD_0)/mass;

    rot_accel.x = (rot_accel.x + CL_p*gyro.x)/inertia.x + gyro.y*gyro.z*(inertia.y - inertia.z)/inertia.x;
    rot_accel.y = (rot_accel.y + CM_q*gyro.y)/inertia.y + gyro.z*gyro.x*(inertia.z - inertia.x)/inertia.y;
    rot_accel.z = (rot_accel.z + CN_r*gyro.z)/inertia.z + gyro.x*gyro.y*(inertia.x - inertia.y)/inertia.z;

    /*
    if (terminal_rotation_rate > 0) {
        // rotational air resistance
        //const Vector3f &gyro = aircraft.get_gyro();
        rot_accel.x -= gyro.x * radians(400.0) / terminal_rotation_rate;
        rot_accel.y -= gyro.y * radians(400.0) / terminal_rotation_rate;
        rot_accel.z -= gyro.z * radians(400.0) / terminal_rotation_rate;
    }

    if (terminal_velocity > 0) {
        // air resistance
        Vector3f air_resistance = -aircraft.get_velocity_air_ef() * (GRAVITY_MSS/terminal_velocity);
        body_accel += aircraft.get_dcm().transposed() * air_resistance;
    }
    */

    // add some noise
    const float gyro_noise = radians(0.1);
    const float accel_noise = 0.3;
    const float noise_scale = thrust.length() / (thrust_scale * num_motors);
    rot_accel += Vector3f(aircraft.rand_normal(0, 1),
                          aircraft.rand_normal(0, 1),
                          aircraft.rand_normal(0, 1)) * gyro_noise * noise_scale;
    body_accel += Vector3f(aircraft.rand_normal(0, 1),
                           aircraft.rand_normal(0, 1),
                           aircraft.rand_normal(0, 1)) * accel_noise * noise_scale;
}

Vector3f Frame::getSign(Vector3f input)
{
    Vector3f temp;

    if (input.x != 0)
        temp.x = input.x/fabsf(input.x);
    else
        temp.x = 0;

    if (input.y != 0)
        temp.y = input.y/fabsf(input.y);
    else
        temp.y = 0;

    if (input.z != 0)
        temp.z = input.z/fabsf(input.z);
    else
        temp.z = 0;

    return temp;
}

void Frame::updateStruct(const Aircraft &aircraft, Motor::kine_struct *kine_lan)
{
    const Vector3f &gyro = aircraft.get_gyro();
    const Matrix3f &dcm = aircraft.get_dcm();
    const Vector3f &vibb = dcm.transposed() * aircraft.get_velocity_air_ef();
    Motor::Atmos_struct atmos;

    // kine_struct update
    kine_lan->Alt = aircraft.get_loc_alt();
    atmosphere(kine_lan->Alt,&atmos);
    kine_lan->rho = atmos.rho;

    kine_lan->vibb.x = vibb.x;
    kine_lan->vibb.y = vibb.y;
    kine_lan->vibb.z = vibb.z;

    kine_lan->wibb.x = gyro.x;
    kine_lan->wibb.y = gyro.y;
    kine_lan->wibb.y = gyro.z;

}

void Frame::initStruct(Motor::aero_struct *quad_lan, Motor::kine_struct *kine_lan)
{
    // aero_struct initialization
    int i;
	double a[] = {1.0, 2*tf_zeta*tf_wn, tf_wn*tf_wn};
	double b[] = {0, -tf_w0*tf_wn*tf_wn, 1.0*tf_wn*tf_wn};

	quad_lan->w_tr = 0;
	quad_lan->lambda0 = 0;
	quad_lan->lambda0_tr = 0;
	quad_lan->omega_dot = 0.0;
	quad_lan->omega_err_state = 0.0;
	quad_lan->yaw_rate_state = 0;
	quad_lan->a_dot = 0;
	quad_lan->b_dot = 0;

    // initialize quad_lan structure
    quad_lan->fibb.x = 0;
    quad_lan->fibb.y = 0;
    quad_lan->fibb.z = 0;

    quad_lan->mibb.x = 0;
    quad_lan->mibb.y = 0;
    quad_lan->mibb.z = 0;

    quad_lan->dcol = 0.0;
    quad_lan->dlat = 0;
	quad_lan->dlon = 0;

	quad_lan->drudd = 0;
	quad_lan->drudd_gyro = 0;
	quad_lan->dmotor = 0.0;
	quad_lan->gov_switch = 0;
	quad_lan->a_rad = 0;
	quad_lan->b_rad = 0;

	quad_lan->mass = MASS;
	quad_lan->Ixx = IXX;
	quad_lan->Iyy = IYY;
	quad_lan->Izz = IZZ;

	quad_lan->Pdyn = 0;

	quad_lan->T_mr = 0;
	quad_lan->T_tr = 0;
	quad_lan->Q_mr = 0;
	quad_lan->Q_tr = 0;
	quad_lan->Q_eng = 0;
	quad_lan->Y_vf = 0;
	quad_lan->Z_ht = 0;

	quad_lan->omega = 0;
	quad_lan->omega_out = 0;
	quad_lan->omega_tr = 0;
	quad_lan->V_imr = 0;
	quad_lan->V_itr = 0;
	quad_lan->K_lam = 0;

	// transfer function
	quad_lan->nIn = coeff;

	for (i = 0; i < coeff; i++)
	{
		quad_lan->aIn[i] = a[i];
		quad_lan->bIn[i] = b[i];
		quad_lan->Xs[i] = 0;
		quad_lan->dXs[i] = 0;
		quad_lan->dX1s[i] = 0;
	}

	quad_lan->Kfactor = 1.0;
    quad_lan->Kfactor1 = 1.0;
    quad_lan->Kfmax = 1.0;
    quad_lan->Kfmin = 0.5;
    quad_lan->omega_out = 0.0;
    quad_lan->PrevInp = 0.0;

    // kine_struct initialization
    kine_lan->vibb.x = 0.0;
    kine_lan->vibb.y = 0.0;
    kine_lan->vibb.z = 0.0;

    kine_lan->vibb1.x = 0.0;
    kine_lan->vibb1.y = 0.0;
    kine_lan->vibb1.z = 0.0;

    // inertial airspeeds
    kine_lan->vibi.x = 0.0;
    kine_lan->vibi.y = 0.0;
    kine_lan->vibi.z = 0.0;

    kine_lan->wibb.x = 0.0;
    kine_lan->wibb.y = 0.0;
    kine_lan->wibb.y = 0.0;

    kine_lan->rho = 1.228;
}


void Frame::atmosphere(const double  kine_z, Motor::Atmos_struct *Atm)
{
	//const double g0 = 9.80665;			/// gravitational acceleration at sea level [m/s^2]
	const double r = 287.05287;			/// gas constant for air [N.m/kg.K]
	const double re = 6.356766e+06;		/// earth radius [m]
	//const double s = 110.4;				/// Sutherland coefficient [K]
	const double temp0 = 288.15;		/// ISA temperature at sea level [K]
	//const double betas = 1.458e-06;		/// Sutherland coefficient [N.s/m^2.K^0.5]
	const double rho0 = 1.225;			/// density for ISA at sea level [kg/m^3]

	/// standard atmosphere temperature gradients (ESDU 77022 Table II)
	const double lt = -6.5e-03;			/// troposphere temperature gradient [K/m]

	const double SHR = 1.4;				/// specific heat ratio [-]
	//const double SHR_PLUS_ONE = 1.5;
	//const double HDEFAULT = 11000.0;

	double Ts;
	//double Tdh;
	double rho;
	double Ps;
	//double deltat = 0.0;
	double h;

	//output protection
	if (kine_z < 0.0)
	{
		h = -kine_z;
	}
	else
	{
		h = kine_z;
	}


	Ts = temp0 + lt*(re*h) / (re + h);
	//Tdh = Ts + deltat;
	rho = rho0*pow((Ts / temp0), 4.25588);
	Ps = rho*r*Ts;

	Atm->Alt = h;
	Atm->Ts = Ts;
	Atm->rho = rho;
	Atm->Ps = Ps;
	Atm->Vsound = sqrt(SHR*r*Ts);
}
