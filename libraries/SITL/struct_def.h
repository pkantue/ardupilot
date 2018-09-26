/**
	*  \file struct_def.h
	*  \brief Structure definitions used throughout
*/
//#ifndef _STRUCT_DEF_H
//	#define _STRUCT_DEF_H

	/// Vector structure definition
	typedef struct
	{
		double x;
		double y;
		double z;
	}vec_struct;

	/// Inertial Euler angles
	typedef struct
	{
		double phi;
		double the;
		double psi;
	}Euler_struct;

	/// Quaternion transformation
	typedef struct
	{
		double q1;
		double q2;
		double q3;
		double q4;
	}Quat_struct;

	/// DCM elements struct
	typedef struct
	{
		double C11;
		double C12;
		double C13;
		double C21;
		double C22;
		double C23;
		double C31;
		double C32;
		double C33;
	}DCM_struct;

	/// aerodynamics data structure due to control deflections
	typedef struct
	{
		double CLMde;		///< Pitching moment increment due to elevator
		double CLMdt;		///< Pitching moment increment due to thrust

		double CLde;		///< Lift force increment due to elevator
		double CLdt;		///< Lift force increment due to thrust

		double CAdt;		///< Axial force due to thrust

		double CYdr;		///< Side force due to rudder
		double CLNdr;		///< Yawing moment due to rudder

		double CLLdr;		///< Roll moment due to rudder deflection
		double CLLdt;		///< Roll moment due to thrust

	}aero_defl_struct;

	/// aerodynamics data structure due to aero angles
	typedef struct
	{
		double CL;			///< Lift force coefficient
		double CD;			///< Drag force coefficient
		double CM;			///< Pitching Moment coefficient
		double CN;			///< Normal force coefficient
		double CA;			///< Axial force coefficient
		double CLA;			///< Lift curve slope coefficient
		double CMA;			///< Pitching moment curve slope coefficient
		double CYB;			///< Side Force due to sideslip angle coefficient
		double CNB;			///< Yawing moment due to sidesip angle coefficient
		double CLB;			///< Rolling moment due to sideslip angle coefficient

		/* Damping coefficients*/
		double CLQ;			///< Lift force due to pitch rate coefficient
		double CMQ;			///< Pitching moment due to pitch rate coefficient
		double CLAD;		///< Lift force due to rate of angle of attack coefficient
		double CLP;			///< Rolling moment due to roll rate coefficient
		double CYP;			///< Side force due to roll rate coefficient
		double CNP;			///< Yawing moment due to roll rate coefficient
		double CNR;			///< Yawing moment due to yaw rate coefficient
		double CLR;			///< Rolling moment due to yaw rate coefficient

	}aero_coef_struct;


    /// Rotor aerodynamics - assigned for simulation outputs ONLY
    typedef struct
    {
        double a_rad;
		double a_dot;
		double a_dot1; // previous value
		double b_rad;
		double b_dot;
		double b_dot1; // previous value
		double omega;
		double omega_out; // measured value of the propeller
		double T_mr;
		double Q_mr;
		double defl;

    }rotor_struct;

	typedef struct
	{
		aero_defl_struct defl_lan;
		aero_coef_struct coef_lan;

		vec_struct fibb;	///< Body forces
		vec_struct mibb;	///< Body moments

		double alpha;
		double beta;

        // for aircraft
		double dele;
		double daile;
		double drudd;

		// for rotorcraft only
		double dcol;
		double dlat;
		double dlon;
		double drudd_gyro;
		double dmotor; //[0 to 1]
		int gov_switch;
		double a_rad;
		double b_rad;

		// global variables
		double w_tr;
		double lambda0;
		double lambda0_tr;
		double omega_dot;
		double omega_err_state;
		double yaw_rate_state;
		double a_dot;
		double a_dot1; // previous value
		double b_dot;
		double b_dot1; // previous value

		double mass;
		double Ixx;
		double Iyy;
		double Izz;
		double Pdyn;

		double T_mr;
		double T_tr;
		double Q_mr;
		double Q_tr;
		double Q_eng;
		double Y_vf;
		double Z_ht;

		double omega;
		double omega_tr;
		double V_imr;
		double V_itr;
		double K_lam;

		// quadcopter - for logging purposes
		rotor_struct rotor[4];

		// Propeller transmission dynamics [used for fault emulation]
		double Xs[10];              ///< State vector current value
		double dXs[10];             ///< State vector derivative - current value
		double dX1s[10];            ///< State vector derivative - previous value

		double aIn[10];             ///< Transfer function denominator coefficients
		double bIn[10];             ///< Transfer function numerator coefficients
		int nIn;                    ///< Number of poles/zeros
		double Kfactor;             ///< Bandwidth scaling factor (unity gain)
		double Kfactor1;             ///< Bandwidth scaling factor (unity gain) - previous value
		double Kfmax;               ///< Bandwidth scaling factor - maximum
		double Kfmin;               ///< Bandwidth scaling factor - minimum

		double omega_out;           ///< measured value of the propeller
		double PrevInp;             ///< Previous control input
		double Kvec[2];             ///< Pole/zero scaling factor vector

	}aero_struct;


	// Atmosphere structure
	typedef struct
	{
		double Alt;					///< Altitude [m]
		double Vsound;				///< Speed of sound [m/s]
		double rho;					///< Air density [kg/m^3]
		double Pdyn;				///< Dynamic pressure [Pa]
		double Ps;					///< Static Pressure [Pa]
		double Ts;					///< Static Pressure [Pa]
		double Mach;				///< Mach number [-]

	}Atmos_struct;

	typedef struct
	{
		vec_struct	ribi;			///< Inertial position vector [m]

		vec_struct	vibi;			///< Inertial velocity vector [m/s]
		vec_struct	vibi1;			///< Inertial velocity vector - previous [m/s]

		vec_struct	vibb;			///< Body velocity vector [m/s]
		vec_struct	vibb1;			///< Body velocity vector - previous [m/s]

		vec_struct	gibb;			///< Body acceleration vector [m/s/s]
		vec_struct	gibb1;			///< Body acceleration vector - previous [m/s/s]

		vec_struct	gibi;			///< Inertial acceleration vector [m/s/s]
		vec_struct	gibi1;			///< Inertial acceleration vector - previous [m/s/s]

		vec_struct	aibb;			///< Body angular acceleration [rad/s/s]
		vec_struct	aibb1;			///< Body angular acceleration - previous [rad/s/s]

		vec_struct	wibb;			///< Body angular velocity [rad/s]
		Euler_struct  Eib;			///< Body Euler angles [rad]

		Quat_struct Quatb;			///< Quaternion rotation
		Quat_struct dQuat;			///< Quaternion rotation rate
		Quat_struct dQuat1;			///< Quaternion rotation rate - previous
		DCM_struct  DCMb;			///< DCM Matrix of quaternions

		double alpha;				///< Angle of attack [rad]
		double beta;				///< Angle of side-slip [rad]

		double rho;					///< Air density
		double Alt;					///< Altitude [m]
		double Vtotal;				///< Body resultant speed [m/s]
		double Pdyn;				///< Dynamic pressure [Pa]

	}kine_struct;

	// setup structure
	typedef struct
	{
		vec_struct vibi0;			///< Initial inertial velocity [m/s]
		vec_struct ribi0;			///< Initial inertial position [m]
		Euler_struct Eib0;			///< Initial inertial Euler angles [rad]
		vec_struct Org0;			///< Initial inerital position of Origin [m]

	}setup_struct;

	/// GPS data structures
	typedef struct
	{
		int gps_fix;	    		///< GPS Satellite fix [1] true [0] false
		double gps_time;			///< GPS time [sec]
		double lat_deg;				///< GPS latitude position [deg]
		double long_deg;			///< GPS longitude position [deg]
		double alt;					///< GPS altitude ASL [m]
		double course; 				///< GPS course over ground [deg]
		double speed;				///< GPS ground speed [m/s]

	}gps_struct;

	/// IMU data structures
	typedef struct
	{
		double abbx; 				///< measured body acceleration along the body x-axis [m/s2]
		double abby;				///< measured body acceleration along the body y-axis [m/s2]
		double abbz; 				///< measured body acceleration along the body z-axis [m/s2]

		double wbbx;  	 			///< measured body rate about the body x-axis [rad/s]
		double wbby;   				///< measured body rate about the body z-axis [rad/s]
		double wbbz;   				///< measured body rate about the body y-axis [rad/s]

		double imu_dT;				///< Elapsed time since last IMU measurement [sec]
	}imu_struct;

	/// Drift Controller data structure
	typedef struct
	{
		double wbbx_err;			///< Estimated body roll error of the DCM matrix [rad]
		double wbby_err;			///< Estimated body pitch error of the DCM matrix [rad]
		double wbbz_err;			///< Estimated body yaw error of the DCM matrix [rad]

		double KI_err[3]; 			///< Integral control error [-]
		double KP_err[3]; 			///< Proportional control error [-]

	}navDrift_struct;

	/// Navigation data structure
	typedef struct
	{
		double vbbx; 				///< estimated body velocity along the body x-axis [m/s]
		double vbby;				///< estimated body velocity along the body y-axis [m/s]
		double vbbz; 				///< estimated body velocity along the body z-axis [m/s]

		double pbbx;   				///< estimated body inertial position coordinated in the body x-axis [m]
		double pbby;   				///< estimated body inertial position coordinated in the body y-axis [m]
		double pbbz;   				///< estimated body inertial position coordinated in the body z-axis [m]

		double Ebbx;				///< Estimated body inertial Euler angle of the body x-axis [deg]
		double Ebby;				///< Estimated body inertial Euler angle of the body y-axis [deg]
		double Ebbz;				///< Estimated body inertial Euler angle of the body z-axis [deg]

		double dcm_eb[3][3];		///< DCM transformation matrix from earth/inertial to body axis [-]

		navDrift_struct drift_lan; 	///< Navigation drift Controller instance [-]
		imu_struct imu_lan;			///< IMU data structure [-]
		gps_struct gps_lan; 		///< GPS data structure for current valid instance  [-]
		gps_struct gps_first_fix;	///< GPS data structure for first valid instance 	[-]
	}nav_struct;

	/// flight control data structures
	typedef struct
	{
		int elevator_pwm;			///< PWM signal from elevator servo command signal [-]
		int aileron_pwm;			///< PWM from the aileron servomotor signal [-]
		int rudder_pwm;	     		///< PWM from the rudder servomotor signal [-]
		int engine_pwm;				///< PWM from the engine servomotor signal [-]

		int dlat_pwm;				///< PWM from the dlat servomotor signal [-]
		int dlon_pwm;				///< PWM from the dlon servomotor signal [-]
		int dcol_pwm;				///< PWM from the dcol servomotor signal [-]
		int dmotor_pwm;				///< PWM from the dmotor servomotor signal [-]

		double elevator_deg;		///< estimated elevator deflection [deg]
		double aileron_deg; 		///< estimated aileron deflection [deg]
		double rudder_deg;			///< estimated rudder deflection [deg]

		double dlon_deg;			///< estimated dlon deflection [deg]
		double dlat_deg;			///< estimated dlat deflection [deg]
		double dcol_deg;			///< estimated dcol deflection [deg]
		double dmotor;			    ///< estimated dmotor deflection [-]

	}fc_struct;

	/// Mission control data structure
	typedef struct
	{
		int mode;

	}mc_struct;

	/// system transmitter Dx4e structure
	typedef struct
	{
		int valid;					///< transmitter valid flag [-]
		unsigned int motor;	  		///< transmitter motor PWM signal [microseconds]
		unsigned int elevator;		///< transmitter elevator PWM signal [microseconds]
		unsigned int rudder;		///< transmitter rudder PWM signal [microseconds]
		unsigned int aileron;		///< transmitter aileron PWM signal [microseconds]
		unsigned int aux;			///< transmitter auxiliary PWM signal [microseconds]

	}dx4e_struct;

	/// system transmitter Dx6e structure
	typedef struct
	{
		int valid;					///< transmitter valid flag [-]
		unsigned int motor;	  		///< transmitter motor PWM signal [microseconds]
		unsigned int elevator;		///< transmitter elevator PWM signal [microseconds]
		unsigned int rudder;		///< transmitter rudder PWM signal [microseconds]
		unsigned int aileron;		///< transmitter aileron PWM signal [microseconds]
		unsigned int aux;			///< transmitter auxiliary PWM signal toggle [microseconds]
		unsigned int aux2;			///< transmitter auxiliary 2 PWM signal toggle [microseconds]

	}dx6e_struct;

	/// Status data structure
	typedef struct
	{
		unsigned int imu_dT;		///< Elapsed time since last IMU measurement [millisecond]
		unsigned int nav_dT;		///< Elapsed time since last navigation measurement [millisecond]
		unsigned int gps_dT;		///< Elapsed time since last GPS computation [millisecond]
		unsigned int wp_dT; 		///< Elapsed time since last waypoint [millisecond]

		unsigned int imu_counter;	///< IMU execution counter [-]
		unsigned int gps_counter;	///< GPS execution counter [-]

		int nav_valid;				///< Navigation valid flag [-]
		int tx_valid;				///< Transmitter valid flag [-]
		int rx_valid;				///< Receiver valid flag [-]
		int gps_valid;				///< GPS valid flag [-]
		int imu_valid;				///< IMU valid flag [-]
		int fc_valid; 				///< Flight Control Valid (AP and Servo Commands) [-]
		int mc_valid;				///< Mission control valid flag [-]

	}status_struct;

    /* Definition of Simulation MASTER structure */
    //simulation overall structure
    typedef struct
    {
        kine_struct   kine_lan;     ///< Kinematics lan structure
        aero_struct   aero_lan;     ///< Aerodynamics lan structure
        setup_struct  setup_lan;    ///< Setup lan structure
        Atmos_struct  atmos_lan;    ///< Atmosphere lan structure
		mc_struct     mc_lan;		///< Mission control structure
    }sim_struct;

//#endif
