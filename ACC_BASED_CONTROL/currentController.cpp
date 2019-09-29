#include "currentController.h"
#include <math.h>

// Control Functions Descriptions //

void accBasedControl::FiniteDiff(float velHum, float velExo)
{
	m_epos->sync();	//Sincroniza a CAN

	vel_hum = vel_hum - LPF_SMF*( vel_hum - velHum);
	acc_hum = (vel_hum - vel_hum_ant)*RATE;
	vel_hum_ant = vel_hum;

	vel_exo = vel_exo- LPF_SMF*( vel_exo - velExo);
	acc_exo = (vel_exo - vel_exo_ant)*RATE;
	vel_exo_ant = vel_exo;

	m_eixo_out->ReadPDO01();
	theta_l = ( (float) (-m_eixo_out->PDOgetActualPosition()- pos0_out)/ENCODER_OUT )*2*MY_PI;				// [rad]

	m_eixo_in->ReadPDO01();
	theta_c = ( (float) (m_eixo_in->PDOgetActualPosition()-pos0_in )/(ENCODER_IN * GEAR_RATIO) )*2*MY_PI;	// [rad]
	d_torque_sea = ( STIFFNESS*(theta_c - theta_l) - torque_sea )*RATE;
	torque_sea = STIFFNESS * (theta_c - theta_l);

	//grav_comp = (INERTIA_EXO + 0.038)*GRAVITY*(0.50)*sin(theta_l);
    accbased_comp = (5 * INERTIA_EXO)*acc_hum + KP_A*(acc_hum - acc_exo) + KI_A*(vel_hum - vel_exo);

    setpoint = (1/TORQUE_CONST) * (1/GEAR_RATIO) * ( accbased_comp - KP_F*torque_sea - KD_F*d_torque_sea );
    setpoint = 60000 * setpoint;

	printf("setpt: %7.3f", setpoint);

	if ( (setpoint >= - CURRENT_MAX*1000) && (setpoint <= CURRENT_MAX*1000) )
	{
		m_eixo_in->PDOsetCurrentSetpoint( (int)setpoint );	// esse argumento � em mA
	}
	m_eixo_in->WritePDO01();

    m_eixo_in->ReadPDO01();
    printf(" %5d mA theta_l: %5.3f deg theta_c: %5.3f deg T_sea: %5.3f N.m T_acc: %-5.3f N.m \n", m_eixo_in->PDOgetActualCurrent(), theta_l * (180/MY_PI), theta_c * (180/MY_PI), torque_sea, accbased_comp);
}


void accBasedControl::Acc_Gravity(float accHum_X, float accHum_Y, float accExo_X, float accExo_Y, float velHum_Z, float velExo_Z)
{
	m_epos->sync();
	
	float sin_thetag = (velExo_Z*velExo_Z*MTW_DIST_EXO - accExo_X) / GRAVITY;
	if (sin_thetag <= 1)
	{
		acc_exo = ( (-1)*accExo_Y + GRAVITY*sqrt(1 - sin_thetag) ) / MTW_DIST_EXO;
	}

	sin_thetag = (velHum_Z*velHum_Z*MTW_DIST_LIMB - accHum_X) / GRAVITY;
	if (sin_thetag <= 1)
	{
		acc_hum = ( accHum_Y + GRAVITY*sqrt(1 - sin_thetag) ) / MTW_DIST_LIMB;
	}

	accbased_comp = (5 * INERTIA_EXO)*acc_hum + KP_A*(acc_hum - acc_exo) + KI_A*(velHum_Z - velExo_Z);


	m_eixo_out->ReadPDO01();
	theta_l = ( (float) (-m_eixo_out->PDOgetActualPosition()- pos0_out)/ENCODER_OUT )*2*MY_PI;				// [rad]

	m_eixo_in->ReadPDO01();
	theta_c = ( (float) (m_eixo_in->PDOgetActualPosition()-pos0_in )/(ENCODER_IN * GEAR_RATIO) )*2*MY_PI;	// [rad]
	d_torque_sea = ( STIFFNESS*(theta_c - theta_l) - torque_sea )*RATE;
	torque_sea = STIFFNESS * (theta_c - theta_l);

	setpoint = (1/TORQUE_CONST) * (1/GEAR_RATIO) * ( accbased_comp - KP_F*torque_sea - KD_F*d_torque_sea );
	setpoint_filt = setpoint_filt - LPF_SMF*( setpoint_filt - setpoint);

	printf("setpt: %7.3f", setpoint);

	if ( (setpoint >= - CURRENT_MAX*1000) && (setpoint <= CURRENT_MAX*1000) )
	{
		m_eixo_in->PDOsetCurrentSetpoint( (int)setpoint );	// esse argumento � em mA
	}
	m_eixo_in->WritePDO01();

	m_eixo_in->ReadPDO01();
	printf(" %5d mA theta_l: %5.3f deg theta_c: %5.3f deg T_sea: %5.3f N.m T_acc: %-5.3f N.m \n", m_eixo_in->PDOgetActualCurrent(), theta_l * (180/MY_PI), theta_c * (180/MY_PI), torque_sea, accbased_comp);
}

