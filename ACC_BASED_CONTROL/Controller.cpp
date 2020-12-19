///////////////////////////////////////////////////////
// Leonardo Felipe Lima Santos dos Santos, 2019     ///
// leonardo.felipe.santos@usp.br	_____ ___  ___   //
// github/bitbucket qleonardolp		| |  | . \/   \  //
////////////////////////////////	| |   \ \   |_|  //
////////////////////////////////	\_'_/\_`_/__|    //
///////////////////////////////////////////////////////

/*
Copyright 2019-2023 Leonardo Felipe Lima Santos dos Santos

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include "Controller.h"

using namespace std;
using namespace chrono;
using namespace Eigen;

// accBasedControl Paramount Variables
EPOS_NETWORK* accBasedControl::m_epos;
AXIS* accBasedControl::m_eixo_in;
AXIS* accBasedControl::m_eixo_out;
float accBasedControl::m_seconds;
int   accBasedControl::pos0_out;
int   accBasedControl::pos0_in;
atomic<bool> accBasedControl::Run(true);
atomic<bool> accBasedControl::logging(false);
system_clock::time_point accBasedControl::control_t_begin;
system_clock::time_point accBasedControl::timestamp_begin;
float accBasedControl::control_t_Dt = 0.001;				// [s]
float accBasedControl::timestamp = 0.0;

// Speed Control [s]
float accBasedControl::Kp_V = 0;
float accBasedControl::Ki_V = 0;
float accBasedControl::Kd_V = 0;
float accBasedControl::Kff_V = 0;

//		CACu Kalman Filter						//
Matrix<float, 5, 1> accBasedControl::CAC_xk;	// State Vector				[vel_hum vel_adm vel_motor theta_c theta_l torque_sea d_torque_sea]
Matrix<float, 4, 1> accBasedControl::CAC_zk;	// Sensor reading Vector	[vel_hum vel_motor theta_c theta_l]
Matrix<float, 5, 5> accBasedControl::CAC_Pk;	// State Covariance Matrix
Matrix<float, 5, 5> accBasedControl::CAC_Fk;	// Prediction Matrix
Matrix<float, 5, 1> accBasedControl::CAC_Bk;	// Control Matrix (is a vector but called matrix)
Matrix<float, 1, 1> accBasedControl::CAC_uk;	// Control Vector
Matrix<float, 5, 5> accBasedControl::CAC_Qk;	// Process noise Covariance
Matrix<float, 4, 4> accBasedControl::CAC_Rk;	// Sensor noise Covariance
Matrix<float, 4, 5> accBasedControl::CAC_Hk;	// Sensor Expectations Matrix
Matrix<float, 5, 4> accBasedControl::CAC_KG;	// Kalman Gain Matrix

//accBasedPostion & OmegaControl Kalman Filter	//
Matrix<float, 5, 1> accBasedControl::xk_omg;	// State Vector				[vel_h vel_e theta_l theta_c pid_e]
Matrix<float, 5, 1> accBasedControl::zk_omg;	// Sensor reading Vector	[vel_h vel_e theta_l theta_c pid_e]
Matrix<float, 5, 5> accBasedControl::Pk_omg;	// State Covariance Matrix
Matrix<float, 5, 5> accBasedControl::Fk_omg;	// Prediction Matrix
Matrix<float, 5, 1> accBasedControl::Bk_omg;	// Control Matrix (is a vector but called matrix)
Matrix<float, 5, 5> accBasedControl::Qk_omg;	// Process noise Covariance
Matrix<float, 5, 5> accBasedControl::Rk_omg;	// Sensor noise Covariance
Matrix<float, 5, 5> accBasedControl::Hk_omg;	// Sensor Expectations Matrix
Matrix<float, 5, 5> accBasedControl::KG_omg;	// Kalman Gain Matrix


// Adimittance Control [a,u] Variables
float accBasedControl::Ki_adm = 0;
float accBasedControl::Kp_adm = 0;
float accBasedControl::torque_m;
float accBasedControl::IntInnerC = 0;
float accBasedControl::vel_inner = 0;
float accBasedControl::IntTorqueM = 0;
int   accBasedControl::resetInt = 0;
float accBasedControl::torque_ref = 0;
float accBasedControl::Adm_In;
float accBasedControl::IntAdm_In = 0;
float accBasedControl::stiffness_d = STIFFNESS / 2;
float accBasedControl::damping_d = 0.001;
float accBasedControl::k_bar = 1 - stiffness_d / STIFFNESS;
float accBasedControl::vel_adm = 0;
float accBasedControl::vel_adm_last = 0;
float accBasedControl::kd_min = damping_d*(Ki_adm / Kp_adm - 1/J_EQ*(damping_d / k_bar - Kp_adm));
float accBasedControl::kd_max = STIFFNESS;
float accBasedControl::torque_u = 0;
float accBasedControl::IntTsea = 0;
float accBasedControl::Int2Tsea = 0;

// State Variables
float accBasedControl::vel_hum = 0;			// [rad/s]
float accBasedControl::vel_exo = 0;			// [rad/s]
float accBasedControl::acc_hum = 0;			// [rad/s^2]
float accBasedControl::acc_exo = 0;			// [rad/s^2]
float accBasedControl::jerk_hum = 0;		// [rad/s^3]
float accBasedControl::jerk_exo = 0;		// [rad/s^3]
float accBasedControl::theta_l = 0;			// [rad]
float accBasedControl::theta_c = 0;			// [rad]
float accBasedControl::vel_hum_last = 0;
float accBasedControl::vel_exo_last = 0;

// Auxiliar Variables
float accBasedControl::setpoint = 0;		// [A]
float accBasedControl::setpoint_filt = 0;	// [A]
int   accBasedControl::actualCurrent = 0;	// [mA] Read from the EPOS
float accBasedControl::accbased_comp = 0;	// [N.m]
float accBasedControl::d_accbased_comp = 0;	// [N.m/s]
float accBasedControl::des_tsea_last = 0;   // [N.m]
float accBasedControl::torque_sea = 0;		// [N.m]
float accBasedControl::torque_sea_last = 0;		// [N.m]
float accBasedControl::d_torque_sea = 0;	// [N.m/s]
float accBasedControl::grav_comp = 0;		// [N.m]
float accBasedControl::vel_leg = 0;			// [rpm ?]
float accBasedControl::acc_motor = 0;		// [rad/s^2]
float accBasedControl::vel_motor = 0;		// [rad/s]
float accBasedControl::vel_motor_filt = 0;	// [rad/s]
float accBasedControl::voltage = 0;			// [V]
float accBasedControl::theta_m = 0;			// [rad]
float accBasedControl::theta_m_last = 0;	//
int	  accBasedControl::actualVelocity = 0;	// [rpm]
int   accBasedControl::exoVelocity = 0;		// [rpm]
float accBasedControl::diffCutoff = CUTOFF;	// [Hz] ?
float accBasedControl::IntegratorHum = 0;
float accBasedControl::IntegratorExo = 0;
float accBasedControl::IntAccMotor = 0;
float accBasedControl::IntAccHum = 0;
float accBasedControl::IntAccExo = 0;


// Control Functions //

void accBasedControl::accBasedPosition(std::vector<float> &ang_vel, std::condition_variable &cv, std::mutex &m)
{
	while (Run.load())
	{
		unique_lock<mutex> Lk(m);
		cv.notify_one();

		control_t_begin = steady_clock::now();

		// try with low values until get confident 1000 Hz
		this_thread::sleep_for(nanoseconds(150));

		m_epos->sync();	// CAN Synchronization

		// Positions Measurement:
		m_eixo_out->ReadPDO01(); theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]
		m_eixo_in->ReadPDO01();  theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]

		// Assigning the measured states to the Sensor reading Vector
		zk_omg << ang_vel[0], ang_vel[1], theta_l, theta_c, (ang_vel[0] - ang_vel[1]);

		// Predicting	//
		xk_omg = Fk_omg * xk_omg;
		Pk_omg = Fk_omg * Pk_omg * Fk_omg.transpose() + Qk_omg;

		// Kalman Gain	//

		// !!! WARNING: DECLARATION MUST MATCH THE SENSOR DIM !!!
		static FullPivLU< Matrix<float, 5, 5> > TotalCovariance(Hk_omg * Pk_omg * Hk_omg.transpose() + Rk_omg);
		if (TotalCovariance.isInvertible())
			KG_omg = Pk_omg * Hk_omg.transpose() * TotalCovariance.inverse();

		// Updating		//
		xk_omg = xk_omg + KG_omg * (zk_omg - Hk_omg*xk_omg);
		Pk_omg = Pk_omg - KG_omg * Hk_omg*Pk_omg;

		// Controlling using the state estimations:
		acc_hum = (xk_omg(0, 0) - vel_hum) / control_t_Dt;
		vel_hum = xk_omg(0, 0);
		vel_exo = xk_omg(1, 0);
		theta_l = xk_omg(2, 0);
		theta_c = xk_omg(3, 0);
		torque_sea = STIFFNESS*(theta_c - theta_l);

		// Position Set	//
		// Using the same gains variables from Speed controller, but loading then from the gains file of the Position controller
		theta_m = theta_l + (Kff_V*INERTIA_EXO*acc_hum + Kd_V / C_DT*xk_omg(4, 0) + Kp_V*xk_omg(4, 0)) / STIFFNESS;
		// using setpoint_filt as encoder steps:
		setpoint_filt = (ENCODER_IN * GEAR_RATIO) * theta_m / (2 * MY_PI) + pos0_in;

		//if ((theta_m >= -1.5708) && (theta_m <= 0.5236)) //(caminhando)
			m_eixo_in->PDOsetPositionSetpoint((int)setpoint_filt);
		m_eixo_in->WritePDO01();

		// Motor Velocity to log
		m_eixo_in->ReadPDO02();
		vel_motor = 1 / GEAR_RATIO * m_eixo_in->PDOgetActualVelocity();

		auto control_t_end = steady_clock::now();
		control_t_Dt = (float)duration_cast<microseconds>(control_t_end - control_t_begin).count();
		control_t_Dt = 1e-6*control_t_Dt;

		Run_Logger();
	}
}

void accBasedControl::OmegaControl(std::vector<float> &ang_vel, std::condition_variable &cv, std::mutex &m)
{
	while (Run.load())
	{
		unique_lock<mutex> Lk(m);
		cv.notify_one();

		control_t_begin = steady_clock::now();

		// try with low values until get confident 1000 Hz
		this_thread::sleep_for(nanoseconds(1500));

		m_epos->sync();	// CAN Synchronization

    vel_hum = HPF_SMF*vel_hum + HPF_SMF*(ang_vel[0] - vel_hum_last);	// HPF on gyroscopes
		vel_hum_last = ang_vel[0];
		vel_exo = HPF_SMF*vel_exo + HPF_SMF*(ang_vel[1] - vel_exo_last);	// HPF on gyroscopes
		vel_exo_last = ang_vel[1];

		// velocities derivative using a Gain and a Discrete Integrator in loop
		acc_exo = 24 * (vel_exo - IntegratorExo);
		IntegratorExo += acc_exo*C_DT;
		acc_hum = 24 * (vel_hum - IntegratorHum);
		IntegratorHum += acc_hum*C_DT;

		// accelerations derivative using a Gain and a Discrete Integrator in loop
		jerk_hum = 15 * (acc_hum - IntAccHum);
		IntAccHum += jerk_hum*C_DT;
		jerk_exo = 15 * (acc_exo - IntAccExo);
		IntAccExo += jerk_exo*C_DT;

		// SEA Torque:

		m_eixo_out->ReadPDO01();
		theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]
		m_eixo_in->ReadPDO01();
		theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]
		torque_sea += LPF_SMF*(STIFFNESS*(theta_c - theta_l) - torque_sea);

		// Jerk Feedforward:
		m_eixo_out->ReadPDO02();
		vel_motor = vel_exo + (Kff_V*INERTIA_EXO*jerk_hum + Kp_V*(jerk_hum - jerk_exo) + Ki_V*(acc_hum - acc_exo)) / STIFFNESS;

		vel_motor = RADS2RPM * GEAR_RATIO * vel_motor;
		vel_motor_filt += LPF_SMF*(vel_motor - vel_motor_filt);

		SetEposVelocityLimited(vel_motor_filt);

		auto control_t_end = steady_clock::now();
		control_t_Dt = (float) duration_cast<microseconds>(control_t_end - control_t_begin).count();
		control_t_Dt = 1e-6*control_t_Dt;
		
		Run_Logger();
	}
}

void accBasedControl::OmegaControlKF(std::vector<float> &ang_vel, std::condition_variable &cv, std::mutex &m)
{
	while (Run.load())
	{
		unique_lock<mutex> Lk(m);
		cv.notify_one();

		control_t_begin = steady_clock::now();

		// try with low values until get confident 1000 Hz
		this_thread::sleep_for(nanoseconds(150));

		m_epos->sync();	// CAN Synchronization

		// Positions Measurement:
		m_eixo_out->ReadPDO01(); theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]
		m_eixo_in->ReadPDO01();  theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]

		// Assigning the measured states to the Sensor reading Vector
		zk_omg << ang_vel[0], ang_vel[1], theta_l, theta_c, (ang_vel[0] - ang_vel[1]);

		// Motor Velocity
		m_eixo_in->ReadPDO02();
		vel_motor = RPM2RADS / GEAR_RATIO * m_eixo_in->PDOgetActualVelocity();

		// Predicting	//
		xk_omg = Fk_omg * xk_omg + Bk_omg * vel_motor;
		Pk_omg = Fk_omg * Pk_omg * Fk_omg.transpose() + Qk_omg;

		// Kalman Gain	//

		// !!! WARNING: DECLARATION MUST MATCH THE SENSOR DIM !!!
		static FullPivLU< Matrix<float, 5, 5> > TotalCovariance(Hk_omg * Pk_omg * Hk_omg.transpose() + Rk_omg);
		if (TotalCovariance.isInvertible())
			KG_omg = Pk_omg * Hk_omg.transpose() * TotalCovariance.inverse();

		// Updating		//
		xk_omg = xk_omg + KG_omg * (zk_omg - Hk_omg*xk_omg);
		Pk_omg = Pk_omg - KG_omg * Hk_omg*Pk_omg;

		// Controlling using the state estimations:
		acc_hum = (xk_omg(0, 0) - vel_hum) / control_t_Dt;
		vel_hum = xk_omg(0, 0);
		vel_exo = xk_omg(1, 0);
		theta_l = xk_omg(2, 0);
		theta_c = xk_omg(3, 0);
		torque_sea = STIFFNESS*(theta_c - theta_l);

		// Position Set	//
		theta_m = theta_l + (Kff_V*INERTIA_EXO*acc_hum + Kd_V / C_DT*xk_omg(4, 0) + Kp_V*xk_omg(4, 0)) / STIFFNESS;

		vel_motor = (theta_m - theta_m_last) / C_DT;
		theta_m_last = theta_m;

		vel_motor = RADS2RPM*GEAR_RATIO*vel_motor;

		SetEposVelocityLimited(vel_motor);

		auto control_t_end = steady_clock::now();
		control_t_Dt = (float)duration_cast<microseconds>(control_t_end - control_t_begin).count();
		control_t_Dt = 1e-6*control_t_Dt;

		Run_Logger();
	}
}

void accBasedControl::CAdmittanceControl(std::vector<float> &ang_vel, std::condition_variable &cv, std::mutex &m)
{
	while (Run.load())
	{
		unique_lock<std::mutex> Lk(m);
		cv.notify_one();

		control_t_begin = steady_clock::now();

		// try with low values until get confident 1000 Hz
		this_thread::sleep_for(nanoseconds(1500));

		m_epos->sync();	// CAN Synchronization

		// Positions
		m_eixo_out->ReadPDO01(); theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]
		m_eixo_in->ReadPDO01();  theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]
		// Motor Velocity
		m_eixo_in->ReadPDO02();
		vel_motor = RPM2RADS / GEAR_RATIO * m_eixo_in->PDOgetActualVelocity();

		// Putting Dt from (Tsea_k - Tsea_k-1)/Dt
		// into the old C2 = (1 - stiffness_d / STIFFNESS) / (stiffness_d + damping_d / C_DT)
		static float C2 = (1 - stiffness_d / STIFFNESS) / (C_DT*stiffness_d + damping_d);
		static float C1 = damping_d / (damping_d + stiffness_d*C_DT);

    vel_hum += LPF_SMF*(ang_vel[0] - vel_hum);
    acc_hum = (vel_hum - vel_hum_last)/C_DT;
		accbased_comp = J_EQ*acc_hum;		// human disturbance input

		grav_comp = -(LOWERLEGMASS*GRAVITY*L_CG)*sin(theta_l);	// inverse dynamics, \tau_W = -M g l sin(\theta_e)
    torque_sea += LPF_SMF*(STIFFNESS*(theta_c - theta_l) - torque_sea_last);

		//vel_adm = C1*vel_adm + C2*(grav_comp + accbased_comp - des_tsea_last - (torque_sea - torque_sea_last));   // C2*(Tsea_d_k - Tsea_d_k-1 - (Tsea_k - Tsea_k-1))
    vel_adm = C1*vel_adm + C2*(0 - (torque_sea - torque_sea_last));

		des_tsea_last = grav_comp + accbased_comp;
		torque_sea_last = torque_sea; 	// Tsea_k-1 <- Tsea_k
		vel_hum_last = vel_hum;		// VelHum_k-1 <- VelHum_k
    vel_exo = ang_vel[1];

		vel_motor = RADS2RPM*GEAR_RATIO*(vel_adm+vel_hum);

		SetEposVelocityLimited(vel_motor);

		auto control_t_end = steady_clock::now();
		control_t_Dt = (float)duration_cast<microseconds>(control_t_end - control_t_begin).count();
		control_t_Dt = 1e-6*control_t_Dt;

		Run_Logger();
	}
}

void accBasedControl::CAdmittanceControlKF(float &velHum, std::condition_variable &cv, std::mutex &m)
{
	while (Run.load())
	{
		unique_lock<std::mutex> Lk(m);
		cv.notify_one();

		control_t_begin = steady_clock::now();

		// try with low values until get confident 1000 Hz
		this_thread::sleep_for(nanoseconds(15));

		resetInt++;	// counter to periodically reset the Inner Loop Integrator 
		m_epos->sync();	// CAN Synchronization

		// Positions
		m_eixo_out->ReadPDO01(); theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]
		m_eixo_in->ReadPDO01();  theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]
		// Motor Velocity
		m_eixo_in->ReadPDO02();
		vel_motor = RPM2RADS / GEAR_RATIO * m_eixo_in->PDOgetActualVelocity();

		// Assigning the measured states to the Sensor reading Vector
		CAC_zk << velHum, vel_motor, theta_c, theta_l;

		m_eixo_in->ReadPDO01();
		actualCurrent = m_eixo_in->PDOgetActualCurrent();
		CAC_uk << (float)(0.001*actualCurrent * TORQUE_CONST - STIFFNESS*(theta_c - theta_l)) / J_EQ;
		// Predicting (test without the Control Mtx)	//
		CAC_xk = CAC_Fk * CAC_xk + CAC_Bk * CAC_uk;
		CAC_Pk = CAC_Fk * CAC_Pk * CAC_Fk.transpose() + CAC_Qk;

		// Kalman Gain	//

		// !!! WARNING: DECLARATION MUST MATCH THE SENSOR DIM !!!
		static FullPivLU<Matrix4f> TotalCovariance(CAC_Hk * CAC_Pk * CAC_Hk.transpose() + CAC_Rk);
		if (TotalCovariance.isInvertible())
			CAC_KG = CAC_Pk * CAC_Hk.transpose() * TotalCovariance.inverse();

		// Updating		//
		CAC_xk = CAC_xk + CAC_KG * (CAC_zk - CAC_Hk*CAC_xk);
		CAC_Pk = CAC_Pk - CAC_KG * CAC_Hk*CAC_Pk;

		// Controlling using the state estimations
		vel_hum = CAC_xk(0, 0);
		vel_motor = CAC_xk(1, 0);
		theta_c = CAC_xk(2, 0);
		theta_l = CAC_xk(3, 0);

		// Putting Dt from (Tsea_k - Tsea_k-1)/Dt
		// into the old C2 = (1 - stiffness_d / STIFFNESS) / (stiffness_d + damping_d / C_DT)
		static float C2 = (1 - stiffness_d / STIFFNESS) / (C_DT*stiffness_d + damping_d);
		static float C1 = damping_d / (damping_d + stiffness_d*C_DT);

		vel_adm = C1*vel_adm - C2*(CAC_xk(4, 0) - torque_sea);   // C2*(Tsea_k - Tsea_k-1)

		torque_sea = CAC_xk(4, 0); // Tsea_k-1 <- Tsea_k

		// Inner Control (PI)	//
		IntInnerC += (vel_hum + vel_adm - vel_motor)*control_t_Dt;
		// Integration reset:
		if (resetInt == 2718281)
		{
			IntInnerC = (vel_hum + vel_adm - vel_motor)*control_t_Dt;
		}
		torque_m = Kp_adm*(vel_hum + vel_adm - vel_motor) + Ki_adm*IntInnerC - torque_sea;

		//-> vel_motor = 1/s * torque_m/J_EQ
		IntTorqueM += (torque_m / J_EQ)*control_t_Dt;
		// Integration reset:
		if (resetInt == 3141592)
		{
			IntTorqueM = (torque_m / J_EQ)*control_t_Dt;
			resetInt = 0;
		}

		vel_motor = RADS2RPM*GEAR_RATIO*IntTorqueM;

		SetEposVelocityLimited(vel_motor);

		auto control_t_end = steady_clock::now();
		control_t_Dt = (float)duration_cast<microseconds>(control_t_end - control_t_begin).count();
		control_t_Dt = 1e-6*control_t_Dt;

		Run_Logger();
	}
}

void accBasedControl::CACurrent(float &velHum, std::condition_variable &cv, std::mutex &m)
{
	while (Run.load())
	{
		unique_lock<std::mutex> Lk(m);
		cv.notify_one();

		control_t_begin = steady_clock::now();

		// try with low values until get confident 1000 Hz
		this_thread::sleep_for(nanoseconds(1500));

		resetInt++;	// counter to periodically reset the Inner Loop Integrator 
		m_epos->sync();	// CAN Synchronization

		vel_hum = HPF_SMF*vel_hum + HPF_SMF*(velHum - vel_hum_last);	// HPF on gyro
		vel_hum_last = velHum;

		// SEA Torque:
		m_eixo_out->ReadPDO01();
		theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]
		m_eixo_in->ReadPDO01();
		theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]
		torque_sea += LPF_SMF*(STIFFNESS*(theta_c - theta_l) - torque_sea);
		d_torque_sea = 13 * (torque_sea - IntAdm_In);
		IntAdm_In += d_torque_sea*control_t_Dt;

		// Outer Admittance Control loop: the discrete realization relies on the derivative of tau_e (check my own red notebook)
		// Here, the reference torque is the torque required to sustain the Exo lower leg mass
		vel_adm = damping_d / (damping_d + stiffness_d*C_DT) * vel_adm +
			(1 - stiffness_d / STIFFNESS) / (stiffness_d + damping_d / C_DT) * (-d_torque_sea);
		// testar sem as duas linhas abaixo
		vel_adm += LPF_SMF*(vel_adm - vel_adm_last);
		vel_adm_last = vel_adm;

		m_eixo_in->ReadPDO02();
		vel_motor = RPM2RADS / GEAR_RATIO * m_eixo_in->PDOgetActualVelocity();

		// Integration for the Inner Control (PI), Trapezoidal Rule
		IntInnerC += 0.5*(vel_inner + vel_hum + vel_adm - vel_motor)*control_t_Dt;
		vel_inner = vel_hum + vel_adm - vel_motor;
		// Integration reset:
		if (resetInt == 2718281){
			IntInnerC = resetInt = vel_inner = 0;
		}
		// Inner Control Loop (PI):
		torque_m = Kp_adm*(vel_hum + vel_adm - vel_motor) + Ki_adm*IntInnerC - torque_sea;
		torque_ref += LPF_SMF*(L_CG*LOWERLEGMASS*GRAVITY*sinf(theta_l) - torque_ref);

		setpoint = 1 / (TORQUE_CONST * GEAR_RATIO)* torque_m; // now in Ampere!
		setpoint_filt += LPF_SMF * (setpoint - setpoint_filt);

		SetEposCurrentLimited(setpoint_filt);

		m_eixo_in->ReadPDO01();
		actualCurrent = m_eixo_in->PDOgetActualCurrent();

		auto control_t_end = steady_clock::now();
		control_t_Dt = (float) duration_cast<microseconds>(control_t_end - control_t_begin).count();
		control_t_Dt = 1e-6*control_t_Dt;

		Run_Logger();
	}
}

void accBasedControl::CACurrentKF(float &velHum, std::condition_variable &cv, std::mutex &m)
{
	while (Run.load())
	{
		unique_lock<std::mutex> Lk(m);
		cv.notify_one();

		control_t_begin = steady_clock::now();

		// try with low values until get confident 1000 Hz
		this_thread::sleep_for(nanoseconds(15));

		resetInt++;	// counter to periodically reset the Inner Loop Integrator 
		m_epos->sync();	// CAN Synchronization

		// Positions
		m_eixo_out->ReadPDO01(); theta_l = ((float)(-m_eixo_out->PDOgetActualPosition() - pos0_out) / ENCODER_OUT) * 2 * MY_PI;				// [rad]
		m_eixo_in->ReadPDO01();  theta_c = ((float)(m_eixo_in->PDOgetActualPosition() - pos0_in) / (ENCODER_IN * GEAR_RATIO)) * 2 * MY_PI;	// [rad]

		// Motor Velocity
		m_eixo_in->ReadPDO02();
		vel_motor = RPM2RADS / GEAR_RATIO * m_eixo_in->PDOgetActualVelocity();

		// Assigning the measured states to the Sensor reading Vector
		CAC_zk << velHum, vel_motor, theta_c, theta_l;

		m_eixo_in->ReadPDO01();
		actualCurrent = m_eixo_in->PDOgetActualCurrent();
    CAC_uk << (float) (0.001*actualCurrent * TORQUE_CONST - STIFFNESS*(theta_c - theta_l))/ J_EQ;
    // Predicting (test without the Control Mtx)	//
    CAC_xk = CAC_Fk * CAC_xk + CAC_Bk * CAC_uk;
		CAC_Pk = CAC_Fk * CAC_Pk * CAC_Fk.transpose() + CAC_Qk;

		// Kalman Gain	//

		// !!! WARNING: DECLARATION MUST MATCH THE SENSOR DIM !!!
		static FullPivLU<Matrix4f> TotalCovariance(CAC_Hk * CAC_Pk * CAC_Hk.transpose() + CAC_Rk);
		if (TotalCovariance.isInvertible())
			CAC_KG = CAC_Pk * CAC_Hk.transpose() * TotalCovariance.inverse();

		// Updating		//
		CAC_xk = CAC_xk + CAC_KG * (CAC_zk - CAC_Hk*CAC_xk);
		CAC_Pk = CAC_Pk - CAC_KG * CAC_Hk*CAC_Pk;

		// Controlling using the state estimations
		vel_hum = CAC_xk(0, 0);
		vel_motor = CAC_xk(1, 0);
		theta_c = CAC_xk(2, 0);
		theta_l = CAC_xk(3, 0);

    // Putting Dt from (Tsea_k - Tsea_k-1)/Dt
    // into the old C2 = (1 - stiffness_d / STIFFNESS) / (stiffness_d + damping_d / C_DT)
	// Back to the old C2:
    static float C2 = (1 - stiffness_d / STIFFNESS) / (stiffness_d + damping_d/C_DT);
	static float C1 = damping_d / (damping_d + stiffness_d*C_DT);

	d_torque_sea = (CAC_xk(4, 0) - torque_sea) / C_DT;
	torque_sea = CAC_xk(4, 0); // Tsea_k-1 <- Tsea_k

	// Acc-based Admittance Control*: ----------------
	static float Pu = J_EQ*stiffness_d / STIFFNESS + (Kp_adm / STIFFNESS - 1)*damping_d;
	static float Iu = (Ki_adm*damping_d + Kp_adm*stiffness_d - stiffness_d*STIFFNESS) / STIFFNESS;
	static float I2u = Ki_adm*stiffness_d / STIFFNESS;
	k_bar = 1 - stiffness_d / STIFFNESS;
	static float Du = J_EQ*damping_d / STIFFNESS - J_EQ*k_bar;

	IntTsea += C_DT*torque_sea;
	Int2Tsea += C_DT*IntTsea;

	torque_u = Pu*torque_sea + Iu*IntTsea + I2u*Int2Tsea + Du*d_torque_sea;
	// -----------------------------------------------

    vel_adm = C1*vel_adm + C2*(0 - d_torque_sea);   // 

		// Inner Control (PI)	//
		IntInnerC += (vel_hum + vel_adm - vel_motor)*control_t_Dt;
		// Integration resets:
		if (resetInt == 2718281)
		{
			IntInnerC = (vel_hum + vel_adm - vel_motor)*control_t_Dt;
			resetInt = 0;
		}
    if (resetInt == 2318421) 
      IntTsea = C_DT*torque_sea;
    if (resetInt == 1612422)
      Int2Tsea = C_DT*IntTsea;

		torque_m = Kp_adm*(vel_hum + vel_adm - vel_motor) + Ki_adm*IntInnerC - torque_sea; // + torque_u; not working...
		setpoint_filt = 1 / (TORQUE_CONST * GEAR_RATIO)* torque_m; // now in Ampere!

		SetEposCurrentLimited(setpoint_filt);

		auto control_t_end = steady_clock::now();
		control_t_Dt = (float)duration_cast<microseconds>(control_t_end - control_t_begin).count();
		control_t_Dt = 1e-6*control_t_Dt;

		Run_Logger();
	}
}

void accBasedControl::SetEposVelocityLimited(float speed_stp)
{
	voltage = abs(speed_stp / SPEED_CONST);

	if ((voltage > 0.060) && (voltage <= VOLTAGE_MAX))
	{
		m_eixo_in->PDOsetVelocitySetpoint((int)speed_stp); // speed in RPM
	}
	else if (voltage > VOLTAGE_MAX)	// upper saturation
	{
		if (speed_stp < 0)
			m_eixo_in->PDOsetVelocitySetpoint(-(int)(SPEED_CONST * VOLTAGE_MAX));
		else
			m_eixo_in->PDOsetVelocitySetpoint((int)(SPEED_CONST * VOLTAGE_MAX));
	}
	else                            // lower saturation
	{
		m_eixo_in->PDOsetVelocitySetpoint(0);
	}
	m_eixo_in->WritePDO02();
}

void accBasedControl::SetEposCurrentLimited(float current_stp)
{
	if (abs(current_stp) < CURRENT_MAX)
	{
		if ((theta_l >= -1.5708) && (theta_l <= 0.5236)) //(caminhando)
			m_eixo_in->PDOsetCurrentSetpoint((int)(current_stp * 1000));	// esse argumento é em mA !!!
	}
	else
	{
		if (current_stp < 0)
			m_eixo_in->PDOsetCurrentSetpoint(-(int)(CURRENT_MAX * 1000));
		else
			m_eixo_in->PDOsetCurrentSetpoint((int)(CURRENT_MAX * 1000));
	}
	m_eixo_in->WritePDO01();
}


void accBasedControl::GainScan()
{
	switch (m_control_mode)
	{
	case 'p':	// GainScan_Position()
		gains_values = fopen("gainsPosition.txt", "rt");

		if (gains_values != NULL)
		{
			fscanf(gains_values, "KFF_V %f\nKP_V %f\nKI_V %f\nKD_V %f\n", &Kff_V, &Kp_V, &Ki_V, &Kd_V);
			fclose(gains_values);
		}
		break;
	case 's':	// GainScan_Velocity()
		gains_values = fopen("gainsSpeed.txt", "rt");

		if (gains_values != NULL)
		{
			//fscanf(gains_values, "KFF_V %f\nKP_V %f\nKI_V %f\nKD_V %f\n", &Kff_V, &Kp_V, &Ki_V, &Kd_V);
      fscanf(gains_values, "KP %f\nKI %f\nSTF %f\nDAM %f\n", &Kp_adm, &Ki_adm, &stiffness_d, &damping_d);
			fclose(gains_values);
		}
		break;
	case 'a':	// GainScan_CAC()
		gains_values = fopen("gainsCAC.txt", "rt");

		if (gains_values != NULL)
		{
			fscanf(gains_values, "KP %f\nKI %f\nSTF %f\nDAM %f\n", &Kp_adm, &Ki_adm, &stiffness_d, &damping_d);
			fclose(gains_values);
		}
		break;
	case 'k':
	case 'u':	// GainScan_CACu()
		gains_values = fopen("gainsCACu.txt", "rt");

		if (gains_values != NULL)
		{
			fscanf(gains_values, "KP %f\nKI %f\nSTF %f\nDAM %f\n", &Kp_adm, &Ki_adm, &stiffness_d, &damping_d);
			fclose(gains_values);
		}
		break;
	default:
		break;
	}
}

void accBasedControl::Recorder()
{
	logger = fopen(getLogfilename(), "a");
	if (logger != NULL)
	{
		switch (m_control_mode)
		{
		case 'a':
			fprintf(logger, "%5.6f  %5.3f  %5.3f  %5.3f  %5.3f\n",\
			timestamp, vel_hum, vel_adm, RPM2RADS*vel_motor, torque_sea);
			break;
		case 'p':
			fprintf(logger, "%5.6f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f\n",\
			timestamp, acc_hum, vel_hum, vel_exo, torque_sea, theta_m);
			break;
		case 's':
			fprintf(logger, "%5.6f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f\n",\
			timestamp, acc_hum, vel_hum, vel_exo, torque_sea, grav_comp, accbased_comp, RPM2RADS*vel_motor);
			break;
		case 'k':
		case 'u':
			fprintf(logger, "%5.6f  %5.3f  %5.3f  %5.3f  %5.2f  %5d  %5.3f\n",\
			timestamp, vel_hum, vel_adm, vel_motor, 1000 * setpoint_filt, actualCurrent, torque_sea); // currents in mA
			break;
		default:
			break;
		}// everything logged in standard units (SI)
		fclose(logger);
	}
}

void accBasedControl::Run_Logger()
{
	if (logging.load())
	{
		timestamp = (float)duration_cast<microseconds>(steady_clock::now() - timestamp_begin).count();
		timestamp = 1e-6*timestamp;
		if (timestamp < m_seconds)
			Recorder();
		else
			logging = false;
	}
}

void accBasedControl::UpdateControlStatus()
{
	char numbers_str[20];
  m_eixo_in->ReadPDO02();
	actualVelocity = m_eixo_in->PDOgetActualVelocity();  //  [rpm]
	switch (m_control_mode)
	{
	case 'p':
		ctrl_word = " POSITION CONTROLLER\n";
		sprintf(numbers_str, "%+5.3f", 180 / MY_PI*theta_m);
		ctrl_word += " Setpoint Position: " + (std::string) numbers_str + " | ";
		sprintf(numbers_str, "%+5.3f", 180 / MY_PI*theta_l);
		ctrl_word += "Leg Position: " + (std::string) numbers_str + " deg\n";
		sprintf(numbers_str, "%+5.3f", vel_motor);
		ctrl_word += " vel_motor: " + (std::string) numbers_str + " rpm ";
		sprintf(numbers_str, "%+2.3f", abs(actualVelocity / SPEED_CONST));
		ctrl_word += "[" + (std::string) numbers_str + " V]\n";
		sprintf(numbers_str, "%5.3f", Kff_V);
		ctrl_word += " Kff_P: " + (std::string) numbers_str;
		sprintf(numbers_str, "%5.3f", Kp_V);
		ctrl_word += " Kp_P: " + (std::string) numbers_str;
		sprintf(numbers_str, "%5.3f", Ki_V);
		ctrl_word += " Ki_P: " + (std::string) numbers_str;
		sprintf(numbers_str, "%5.3f", Kd_V);
		ctrl_word += " Kd_P: " + (std::string) numbers_str + "\n";
		ctrl_word += " T_Sea: " + std::to_string(torque_sea) + " N.m\n";
		break;
	case 's':
		ctrl_word = " SPEED CONTROLLER\n";
		sprintf(numbers_str, "%+5.3f", vel_motor);
		ctrl_word += " vel_motor: " + (std::string) numbers_str + " rpm ";
		sprintf(numbers_str, "%+2.3f", abs(actualVelocity / SPEED_CONST));
		ctrl_word += "[" + (std::string) numbers_str + " V]\n";
		sprintf(numbers_str, "%5.3f", stiffness_d);
		ctrl_word += " STF: " + (std::string) numbers_str;
		sprintf(numbers_str, "%5.3f", damping_d);
		ctrl_word += " DAM: " + (std::string) numbers_str + "\n";
    ctrl_word += " T_Sea: " + std::to_string(torque_sea) + " N.m " +\
    std::to_string(180 / MY_PI*theta_l) + " " + std::to_string(180 / MY_PI*theta_c) + "\n";
		ctrl_word += " InvDyn: " + std::to_string(grav_comp) + " N.m ";
		ctrl_word += " AccBased: " + std::to_string(accbased_comp) + " N.m\n";
		break;
	case 'a':
	case 'u':
	case 'k':
		ctrl_word = " COLLOCATED ADMITTANCE CONTROLLER\n";
		if (m_control_mode == 'a')
		{
			sprintf(numbers_str, "%+5.3f", vel_motor);
			ctrl_word += " vel_motor: " + (std::string) numbers_str + " rpm";
			sprintf(numbers_str, "%+4d", actualVelocity);
			ctrl_word += " [" + (std::string) numbers_str + " rpm	";
			sprintf(numbers_str, "%+2.3f", abs(actualVelocity / SPEED_CONST));
			ctrl_word += (std::string) numbers_str + " V]\n";
		}
		else if (m_control_mode == 'u' || m_control_mode == 'k')
		{
			ctrl_word += " Torque: " + std::to_string(torque_m) + " N.m";
			ctrl_word += " [ " + std::to_string(1000 * setpoint_filt) + " ";
			ctrl_word += std::to_string(actualCurrent) + " mA]\n";
		}
		sprintf(numbers_str, "%5.3f", Kp_adm);
		ctrl_word += " Kp: " + (std::string) numbers_str;
		sprintf(numbers_str, "%5.3f", Ki_adm);
		ctrl_word += " Ki: " + (std::string) numbers_str;
		sprintf(numbers_str, "%5.3f", stiffness_d);
		ctrl_word += " STF: " + (std::string) numbers_str;
		sprintf(numbers_str, "%5.3f", damping_d);
		ctrl_word += " DAM: " + (std::string) numbers_str + "\n";
		ctrl_word += " T_Sea: " + std::to_string(torque_sea);
		ctrl_word += " | T_ref: " + std::to_string(torque_ref) + " [N.m]\n";
		ctrl_word += "\n -> Passivity Constraints <-\n ";

		k_bar = 1 - stiffness_d / STIFFNESS;
		kd_min = damping_d*(Ki_adm / Kp_adm - 1 / J_EQ*(damping_d / k_bar - Kp_adm));

		ctrl_word += std::to_string(kd_min) + " < kd < " + std::to_string(kd_max) + "\n\n";
		break;
	default:
		break;
	}
	sprintf(numbers_str, "%4.2f", 1 / control_t_Dt);
	ctrl_word += " EPOS Rate: " + (std::string) numbers_str + " Hz\n";
}

void accBasedControl::SavitskyGolay(float window[], float newest_value, float* first_derivative)
{
	// shifting values in 1 position, updating the window with one sample:
	for (int i = 10; i > 0; --i)
	{
		window[i] = window[i - 1];
	}

	// *Savitsky-Golay Smoothing Coefficients, 4th order fit with 11 pts and +5 offset from the centre point
	// Norm: 143 |	Coeff: 6	-10	-5	5	10	6	-5	-15	-10	30	131

	window[0] = (6 * window[10] - 10 * window[9] - 5 * window[8]
		+ 5 * window[7] + 10 * window[6] + 6 * window[5]
		- 5 * window[4] - 15 * window[3] - 10 * window[2]
		+ 30 * window[1] + 131 * newest_value) / 143;


	// *Finite Difference, SG Coefficients, First Derivative, linear fit with 11 pts and +5 offset
	*first_derivative = (5 * window[0] + 4 * window[1] + 3 * window[2] + 2 * window[3] +
		1 * window[4] + 0 * window[5] - 1 * window[6] - 2 * window[7] -
		3 * window[8] - 4 * window[9] - 5 * window[10]) / (110) * RATE;

}