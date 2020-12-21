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

//		Extended Kalman Filter				    //
Matrix<float, EKF_STATE_DIM, 1>                 accBasedControl::ekf_xk;    // State Vector				[x_h x_e \dot{x_e} x_a \dot{x_a}]
Matrix<float, EKF_SENSOR_DIM, 1>                accBasedControl::ekf_zk;	// Sensor reading Vector	[\dot{x_h} x_e \dot{x_e} x_a \dot{x_a}]
Matrix<float, EKF_STATE_DIM, EKF_STATE_DIM>     accBasedControl::ekf_Pk;	// State Covariance Matrix
Matrix<float, EKF_STATE_DIM, EKF_STATE_DIM>     accBasedControl::ekf_Gk;	// State transition Jacobian
Matrix<float, EKF_STATE_DIM, EKF_CTRL_DIM>      accBasedControl::ekf_Bk;	// Control Matrix
Matrix<float, EKF_CTRL_DIM, 1>                  accBasedControl::ekf_uk; 	// Control Vector
Matrix<float, EKF_STATE_DIM, EKF_STATE_DIM>     accBasedControl::ekf_Qk;	// Process noise Covariance
Matrix<float, EKF_SENSOR_DIM, EKF_SENSOR_DIM>   accBasedControl::ekf_Rk; 	// Sensor noise Covariance
Matrix<float, EKF_SENSOR_DIM, EKF_STATE_DIM>    accBasedControl::ekf_Hk;	// Sensor expectations Jacobian
Matrix<float, EKF_STATE_DIM, EKF_SENSOR_DIM>    accBasedControl::ekf_KG;	// Kalman Gain Matrix
Matrix<float, EKF_SENSOR_DIM, EKF_CTRL_DIM>	    accBasedControl::ekf_Dk;	// Feedforward Matrix

void accBasedControl::ekfUpdate()
{
    // revisar tudo...
    // Assigning the measured states to the Sensor reading Vector
	ekf_zk << vel_hum, vel_motor, theta_c, theta_l;
	m_eixo_in->ReadPDO01();
	actualCurrent = m_eixo_in->PDOgetActualCurrent();
	ekf_uk << (float)(0.001*actualCurrent * TORQUE_CONST - STIFFNESS*(theta_c - theta_l)) / J_EQ;

	// Predict      //
	//ekf_xk = ekf_Fk * ekf_xk + ekf_Bk * ekf_uk;
	//ekf_Pk = ekf_Fk * ekf_Pk * ekf_Fk.transpose() + ekf_Qk;

	// Kalman Gain	//
	static FullPivLU<SensorSzMtx> TotalCovariance(ekf_Hk * ekf_Pk * ekf_Hk.transpose() + ekf_Rk);
	if (TotalCovariance.isInvertible())
		ekf_KG = ekf_Pk * ekf_Hk.transpose() * TotalCovariance.inverse();
    
	// Updating		//
	ekf_xk = ekf_xk + ekf_KG * (ekf_zk - ekf_Hk*ekf_xk);
	ekf_Pk = (StateSzMtx::Identity() - ekf_KG * ekf_Hk)*ekf_Pk;

    theta_l = ekf_xk(0,1);
    ekf_Gk(2, 1) = -(int_stiffness + STIFFNESS - LOWERLEGMASS*GRAVITY*L_CG*cos(theta_l))/INERTIA_EXO;
}