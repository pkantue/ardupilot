/**
	*  \file h1_quad_definitions.h
	*  \brief This is the definitions file for the H1 quadcopter
	*  \date September 2018
	*  \author P. Kantue
*/

#ifndef H1_QUAD_DEF
#define H1_QUAD_DEF

/** Mass and inertia characteristics H-1 quadcopter */
#define MASS 		1.75
#define IXX  		0.03274
#define IYY  		0.04202
#define IZZ  		0.07327

/** Geometry characteristics H-1 Quadcopter */
#define R2D		    (180/3.1416)
#define D2R		    (3.1416/180)
#define PI          3.141592653589793
#define A_FRONT     0.0250 /* 25cm x 2 x 5cm */
#define A_BACK		0.0560 /* 35cm x 2 x 8cm */
#define A_SIDE		0.0210 /* 35cm x 2 x 3cm */
#define CD_0		1.08

/// Aerodynamic damping coefficients (resistance to rate attitude change)
#define CL_p		-0.45
#define CM_q		-0.51
#define CN_r		-0.8

/** Helicopter specific parameters */
#define K_beta 		5.4 // To be estimated from data
#define gam_fb 		0.8
#define B_nom_lat 	4.2
#define A_nom_lon 	4.2
#define K_u 		0.2
#define R_mr 		0.127
#define c_mr 		0.035
#define a_mr 		5.5

#define C_mr_Tmax   0.15
#define I_beta_mr   0.01

/*
Turnigy 1/8th Scale 4 Pole Brushless Motor - 2100KV

Turns: 2.0D
Poles: 4
RPM/v: 2100
Voltage: 3-5s Lipoly
Continuous Current: 80A
Peak Current: 140A
Resistance: 4.7mOhms
No Load Current: 3.7A
Dimension: 43 x 63.4mm
Shaft Length: 20mm
Shaft Diameter: 5mm
Weight: 308g

*/

/*
H1 Quadcopter layout as per ArduCopter 3.4

% /---\           /---\
% | 3 |===========| 1 |
% \---/    | |    \---/     x^
%          | |               |
%          |o|               |______>
%          | |                      y
% /---\    | |    /---\
% | 2 |===========| 4 |
% \---/           \---/

*/

#define P_eng_idl 	0
#define P_eng_max 	2000
#define motor_KV    625     /* Rctimer 5010-620KV Multicopter Brushless Motor - http://rctimer.com/product-911.html */
#define Q_stall     2.42    /* calculated from http://lancet.mit.edu/motors/motors3.html */
#define max_volt	14.8    // 4-Cell battery
#define gear_mr		1       /* Propeller applied straight to the shaft */

#define ROLL_LIM    0.1     /* Roll angular acceleration */
#define PITCH_LIM   0.1     /* Pitch angular acceleration */
#define YAW_LIM     0.1     /* Yaw angular acceleration */
#define MAX_THRUST  0.8     /* Max Thrust coefficient  */
#define MIN_THRUST  0.1     /* Min Thrust coefficient for deflection application */
#define K_p 		0.01
#define K_i 		0.02

#define f_s_p 		12.5
#define f_s_q 		9
#define f_s_r 		9.6
#define eta_s 		0.05
#define S_fus_x 	0.1
#define S_fus_y 	0.22
#define S_fus_z 	0.15
#define h_mr 		0.235
#define l_tr 		0.91
#define h_tr 		0.08
#define l_ht 		0.71

#define Omeg_nom 	167
#define eta_w 		0.9
#define C_tr_Do 	0.024
#define GK_p 		0.004
#define GK_i 		0.09

#define tf_wn		6.2     /* natural frequency */
#define tf_zeta 	0.89    /* damping ratio */
#define tf_w0		0.0     /* non-minimum zero - 0.0005 */
#define coeff       3       /* Transfer function no. of coeffients */

#define ARM_XCG     0.35
#define ARM_YCG     0.25
#define ARM_ZCG     0.05

#define GRAVITY     9.81

#endif // H1_QUAD_DEF
