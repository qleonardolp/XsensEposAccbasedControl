#ifndef CURRENT_CONTROL_H
#define CURRENT_CONTROL_H

#include "AXIS.h"
#include "EPOS_NETWORK.h"


// CONSTANTES

#define		GEAR_RATIO		150.0		    // Redu��o do Sistema
#define		ENCODER_IN		4096		    // Resolu��o do encoder do motor
#define		ENCODER_OUT		2048		    // Resolu��o do encoder de sa�da
#define		STIFFNESS		104.0			// Constante da mola SEA [N.m/rad]

//	Torque Constant: 0.0603 N.m/A = 60.3 N.m/mA
//	Speed Constant: 158 rpm/V
//	Max current (@ 48 V)  ~3.1 A
//	Stall current (@ 48 V)  42.4 A

#define		CURRENT_MAX		3.1000		// Max corrente nominal no motor Maxon RE40 [A]
#define		TORQUE_CONST	60.300		// Constante de torque do motor RE40	[N.m/mA]


#define     GRAVITY         9.8066      // [m/s^2]
#define     INERTIA_EXO     0.0655      // [Kg.m^2], +- 0.0006, estimado em 2019-08-21
#define		MTW_DIST_LIMB	0.2500		// [m]
#define		MTW_DIST_EXO	0.0700		// [m]
#define		L_CG			0.3500		// [m]

// Feedback PI acc-based controller:
#define     KP_A			2.1205      // [Kg.m^2]
#define     KI_A			1.4069      // [Kg.m^2/s]

// Feedback PD force (SEA) controller:
#define     KP_F			13.131      // [dimensionless]
#define     KD_F			2.4846      // [s]


#define     RATE            120.00      // [Hz]		  ?? Ts = 0.005 -> 200 Hz ??
#define     LPF_FC          15.000      // [Hz] Low Pass Filter Frequency Cutoff
#define		MY_PI			3.141592653	// Pi value
#define		LPF_SMF         ( (2*MY_PI / RATE) / (2*MY_PI / RATE + 1 / LPF_FC) )    // Low Pass Filter Smoothing Factor


class accBasedControl
{
public:
	// constructor
	accBasedControl(EPOS_NETWORK* epos, AXIS* eixo_in, AXIS* eixo_out)
	{
		vel_hum =		0;
		vel_exo =		0;
		vel_hum_ant =	0;
		vel_exo_ant =	0;
		torque_sea =	0;
		setpoint =		0;
		setpoint_filt = 0;

		m_epos = epos;
		m_eixo_in = eixo_in;
		m_eixo_out = eixo_out;

		pos0_out = -m_eixo_out->PDOgetActualPosition();
		pos0_in = m_eixo_in->PDOgetActualPosition();
	}

	// numdiff
	void FiniteDiff(float velHum, float velExo);

	// acc-gravity
	void Acc_Gravity(float accHum_X, float accHum_Y, float accExo_X, float accExo_Y, float velHum_Z, float velExo_Z);

	// destructor
	~accBasedControl()
	{
		m_eixo_in->PDOsetCurrentSetpoint(0);
		m_eixo_in->WritePDO01();
	}

private:

	EPOS_NETWORK* m_epos;
	AXIS* m_eixo_in;
	AXIS* m_eixo_out;

	float acc_hum;			// [rad/s^2]
	float acc_exo;			// [rad/s^2]

	float vel_hum;			// [rad/s]
	float vel_exo;			// [rad/s]
	float vel_hum_ant;		// [rad/s]
	float vel_exo_ant;		// [rad/s]

	float setpoint;			// [mA]
	float setpoint_filt;

	float theta_l;			// [rad]
	float theta_c;			// [rad]
	float torque_sea;		// [N.m]
	float d_torque_sea;		// [N.m/s]
	float accbased_comp;	// [N.m]
	float grav_comp;		// [N.m]
	int pos0_out;
	int pos0_in;
};

#endif // !CURRENT_CONTROL_H