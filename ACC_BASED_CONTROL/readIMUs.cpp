//////////////////////////////////////////\/////////\/
// INTERFACE DE CONTROLE EXO-TAU  /       /\     ////\
// EESC-USP                      / _____ ___  ___  //|
// RehabLab                     /  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  | |   \ \   |_|  /|
//\///////////////////////\// //// \_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\

#include "mastercallback.h" // Inclui Xsens device API header
#include "mtwcallback.h"
// Copyright (c) 2003-2016 Xsens Technologies B.V.
// or subsidiaries worldwide. All rights reserved.

#include "QpcLoopTimer.h" // ja inclui <windows.h>
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include "LowPassFilter2p.h"
#include <processthreadsapi.h>
#include <iostream>
#include <iomanip>
#include <conio.h>
#include <string>

/*! \brief Stream insertion operator overload for XsPortInfo */
std::ostream& operator<<(std::ostream& out, XsPortInfo const& p)
{
	out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
		<< std::setw(7) << p.baudrate() << " Bd"
		<< ", "
		<< "ID: " << p.deviceId().toString().toStdString();
	return out;
}

/*! \brief Stream insertion operator overload for XsDevice */
std::ostream& operator<<(std::ostream& out, XsDevice const& d)
{
	out << "ID: " << d.deviceId().toString().toStdString() << " (" << d.productCode().toStdString() << ")";
	return out;
}

int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate);

void readIMUs(ThrdStruct& data_struct)
{
	using namespace std;
#if PRIORITY
	SetThreadPriority(GetCurrentThread(), data_struct.param00_);
#endif

#if IMU_DBG_LOG
	char filename[] = "./data/im_debug_log.txt";
	FILE* logFileHandle = fopen(filename, "w");
	if (logFileHandle != NULL) fclose(logFileHandle);
#endif

	{
		unique_lock<mutex> _(*data_struct.mtx_);
		*data_struct.param0A_ = false; // not ready
		*data_struct.param1A_ = false;  // not aborting
		*data_struct.param3F_ = false;  // not finished
	}

	/* Xsens Awinda Station management
	__________________________________
	| MTw  | desiredUpdateRate (max) |
	|------|-------------------------|
	|  1   |           150 Hz        |
	|  2   |           120 Hz        |
	|  4   |           100 Hz        |
	|  6   |            75 Hz        |
	|  12  |            50 Hz        |
	|__18__|____________40 Hz________| */

	const int desiredUpdateRate = 75; //
	const int desiredRadioChannel = 25;
	const float sampleTime = 1 / float(desiredUpdateRate);

	WirelessMasterCallback wirelessMasterCallback; // Callback for wireless master
	vector<MtwCallback*> mtwCallbacks;            // Callbacks for mtw devices

	XsControl* control = XsControl::construct();
	if (control == 0)
	{
		cout << "Failed to construct XsControl instance." << endl;
	}

	try
	{
		XsPortInfoArray SerialPorts = XsScanner::enumerateSerialPorts();
		XsPortInfoArray::iterator SPt = SerialPorts.begin();
		while (SPt != SerialPorts.end())
		{
			cout << *SPt << endl;
			++SPt;
		}

		XsPortInfoArray detectedDevices = XsScanner::scanPorts();
		XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();
		while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster())
		{
			++wirelessMasterPort;
		}
		if (wirelessMasterPort == detectedDevices.end())
		{
			throw runtime_error("No wireless masters found");
		}
		cout << "Wireless master found @ " << *wirelessMasterPort << endl;

		if (!control->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate()))
		{
			ostringstream error;
			error << "Failed to open port " << *wirelessMasterPort;
			throw runtime_error(error.str());
		}
		XsDevicePtr wirelessMasterDevice = control->device(wirelessMasterPort->deviceId());
		if (wirelessMasterDevice == 0)
		{
			ostringstream error;
			error << "Failed to construct XsDevice instance: " << *wirelessMasterPort;
			throw runtime_error(error.str());
		}
		if (!wirelessMasterDevice->gotoConfig())
		{
			ostringstream error;
			error << "Failed to goto config mode: " << *wirelessMasterDevice;
			throw runtime_error(error.str());
		}

		//detectedDevices.clear();

		wirelessMasterDevice->addCallbackHandler(&wirelessMasterCallback);

		const XsIntArray supportedUpdateRates = wirelessMasterDevice->supportedUpdateRates();
		const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, desiredUpdateRate);

		if (!wirelessMasterDevice->setUpdateRate(newUpdateRate))
		{
			ostringstream error;
			error << "Failed to set update rate: " << *wirelessMasterDevice;
			throw runtime_error(error.str());
		}
		if (wirelessMasterDevice->isRadioEnabled())
		{
			if (!wirelessMasterDevice->disableRadio())
			{
				ostringstream error;
				error << "Failed to disable radio channel: " << *wirelessMasterDevice;
				throw runtime_error(error.str());
			}
		}
		if (!wirelessMasterDevice->enableRadio(desiredRadioChannel))
		{
			ostringstream error;
			error << "Failed to set radio channel: " << *wirelessMasterDevice;
			throw runtime_error(error.str());
		}

		cout << "Waiting for MTW to wirelessly connect..." << endl;

		bool quitOnMTw = false;
		bool waitForConnections = true;

		size_t connectedMTWCount = wirelessMasterCallback.getWirelessMTWs().size();
		do
		{
			XsTime::msleep(100);

			while (true)
			{
				size_t nextCount = wirelessMasterCallback.getWirelessMTWs().size();
				if (nextCount != connectedMTWCount)
				{
					cout << "Number of connected MTWs: " << nextCount << ". Press 'y' to start measurement or 'q' to quit \n";
					connectedMTWCount = nextCount;
				}
				else
				{
					break;
				}
			}
			if (_kbhit())
			{
				char keypressed = _getch();
				if ('y' == keypressed)
					waitForConnections = false;
				if ('q' == keypressed)
				{
					quitOnMTw = true;
					waitForConnections = false;
				}
			}
		} while (waitForConnections);

		if (quitOnMTw)
		{
			wirelessMasterDevice->gotoConfig();
			wirelessMasterDevice->disableRadio();
			throw runtime_error("quit by user request");
		}

		if (!wirelessMasterDevice->gotoMeasurement())
		{
			ostringstream error;
			error << "Failed to goto measurement mode: " << *wirelessMasterDevice;
			throw runtime_error(error.str());
		}

		XsDeviceIdArray allDeviceIds = control->deviceIds();
		XsDeviceIdArray mtwDeviceIds;
		for (XsDeviceIdArray::const_iterator i = allDeviceIds.begin(); i != allDeviceIds.end(); ++i)
		{
			if (i->isMtw())
			{
				mtwDeviceIds.push_back(*i);
			}
		}
		XsDevicePtrArray mtwDevices;
		for (XsDeviceIdArray::const_iterator i = mtwDeviceIds.begin(); i != mtwDeviceIds.end(); ++i)
		{
			XsDevicePtr mtwDevice = control->device(*i);
			if (mtwDevice != 0)
			{
				mtwDevices.push_back(mtwDevice);
			}
			else
			{
				throw runtime_error("Failed to create an MTW XsDevice instance");
			}
		}

		cout << "Attaching callback handlers to MTWs..." << endl;
		vector<string> imu_names(mtwDevices.size());
		mtwCallbacks.resize(mtwDevices.size());
		for (int i = 0; i < (int)mtwDevices.size(); ++i)
		{
			mtwCallbacks[i] = new MtwCallback(i, mtwDevices[i]);
			mtwDevices[i]->addCallbackHandler(mtwCallbacks[i]);
			imu_names[i] = mtwDevices[i]->deviceId().toString().toStdString();
			if (imu_names[i].compare("00B412DF") == 0)
				cout << "IMU Coxa Direita: " << imu_names[i] << "\n";
			if (imu_names[i].compare("00B410D2") == 0)
				cout << "IMU Canela Direita: " << imu_names[i] << "\n";
			if (imu_names[i].compare("00B41244") == 0)
				cout << "IMU Pe Direito: " << imu_names[i] << "\n";
			if (imu_names[i].compare("00B4108C") == 0)
				cout << "IMU Canela ExoTau: " << imu_names[i] << "\n";
			if (imu_names[i].compare("00342322") == 0)
				cout << "IMU 01: " << imu_names[i] << "\n";
			if (imu_names[i].compare("00342323") == 0)
				cout << "IMU 02: " << imu_names[i] << "\n";
			if (imu_names[i].compare("00342324") == 0)
				cout << "IMU 03: " << imu_names[i] << "\n";
		}

		vector<XsVector> accData(mtwCallbacks.size());
		vector<XsVector> gyroData(mtwCallbacks.size());

		//wirelessMasterCallback.mtw_event.clear(); // XsMutex here...?

		float imus_data[DTVC_SZ];
		for (int i = 0; i < DTVC_SZ; i++) imus_data[i] = 0;

		// Filtros Passa Baixa para os dados das IMUs
		LowPassFilter2pFloat imu_filters[DTVC_SZ];
		for (int i = 0; i < DTVC_SZ; i++)
		{
			float thread_frequency = desiredUpdateRate;
			imu_filters[i].set_cutoff_frequency(thread_frequency, LPF_CUTOFF);
			imu_filters[i].reset();
		}

		{   // readIMUs nao espera as outras threads:
			unique_lock<mutex> _(*data_struct.mtx_);
			*data_struct.param0A_ = true; // readIMUs avisa que esta pronto!
			cout << "-> Reading IMUs!\n";
		}

		looptimer xsensTimer(sampleTime, data_struct.exectime_);
		// inicializar looptimer:
		xsensTimer.start();
		do
		{
			xsensTimer.tik();
			// IMU connection check for safety
			// Avoid wirelessMasterCallback here, I dont know if their mutex is the same of
			// mtwCallbacks!!!
			//int imus_connected = wirelessMasterCallback.getWirelessMTWs().size();
			if ((int)mtwCallbacks.size() < 2)
			{
				unique_lock<mutex> _(*data_struct.mtx_);
				*data_struct.param1A_ = true; // aborting
				break; // get out of the reading loop!
			}

			for (size_t i = 0; i < (int)mtwCallbacks.size(); ++i)
			{
				if (mtwCallbacks[i] == NULL) continue;
				bool newDataAvailable = false;
				if (mtwCallbacks[i]->dataAvailable())
				{
					newDataAvailable = true;

					//XsDataPacket const* packet = mtwCallbacks[i]->getOldestPacket();
					XsDataPacket packet = mtwCallbacks[i]->fetchOldestPacket();
					if (packet.containsCalibratedGyroscopeData())
						gyroData[i] = packet.calibratedGyroscopeData();

					if (packet.containsCalibratedAcceleration())
						accData[i] = packet.calibratedAcceleration();

					//mtwCallbacks[i]->deleteOldestPacket();
				}

				if (newDataAvailable) {
					// Orientacao Perna DIR: [-3 2 1]
					// Orientacao Perna ESQ: [3 -2 1]
					// Orientacao P??? DIR:    [2 -1 3]
					// Avoid gyroData[i][k] or gyroData[i].at(k) or gyroData[i].value(k)
					// due to the 'assert' inside these operators on xsvector.h !!!
					vector<XsReal> gyroVector = gyroData[i].toVector();
					vector<XsReal> accVector = accData[i].toVector();
					if (imu_names[i].compare("00B412DF") == 0) {
						imus_data[0] = imu_filters[0].apply(-gyroVector[2]);
						imus_data[1] = imu_filters[1].apply(gyroVector[1]);
						imus_data[2] = imu_filters[2].apply(gyroVector[0]);
						imus_data[3] = imu_filters[3].apply(-accVector[2]);
						imus_data[4] = imu_filters[4].apply(accVector[1]);
						imus_data[5] = imu_filters[5].apply(accVector[0]);
						//cout << imu_names[i] << endl;
					}
					if (imu_names[i].compare("00B410D2") == 0) {
						imus_data[6] = imu_filters[6].apply(-gyroVector[2]);
						imus_data[7] = imu_filters[7].apply(gyroVector[1]);
						imus_data[8] = imu_filters[8].apply(gyroVector[0]);
						imus_data[9] = imu_filters[9].apply(-accVector[2]);
						imus_data[10] = imu_filters[10].apply(accVector[1]);
						imus_data[11] = imu_filters[11].apply(accVector[0]);
						//cout << imu_names[i] << endl;
					}
					if (imu_names[i].compare("00B41244") == 0) {
						imus_data[12] = imu_filters[12].apply(gyroVector[1]);
						imus_data[13] = imu_filters[13].apply(-gyroVector[0]);
						imus_data[14] = imu_filters[14].apply(gyroVector[2]);
						imus_data[15] = imu_filters[15].apply(accVector[1]);
						imus_data[16] = imu_filters[16].apply(-accVector[0]);
						imus_data[17] = imu_filters[17].apply(accVector[2]);
						//cout << imu_names[i] << endl;
					}
					if (imu_names[i].compare("00B4108C") == 0) {
						imus_data[18] = imu_filters[18].apply(gyroVector[2]);
						imus_data[19] = imu_filters[19].apply(-gyroVector[1]);
						imus_data[20] = imu_filters[20].apply(gyroVector[0]);
						imus_data[21] = imu_filters[21].apply(accVector[2]);
						imus_data[22] = imu_filters[22].apply(-accVector[1]);
						imus_data[23] = imu_filters[23].apply(accVector[0]);
						//cout << imu_names[i] << endl;
					}


					if (imu_names[i].compare("00342322") == 0) {
						imus_data[0] = imu_filters[0].apply(-gyroVector[2]);
						imus_data[1] = imu_filters[1].apply(gyroVector[1]);
						imus_data[2] = imu_filters[2].apply(gyroVector[0]);
						imus_data[3] = imu_filters[3].apply(-accVector[2]);
						imus_data[4] = imu_filters[4].apply(accVector[1]);
						imus_data[5] = imu_filters[5].apply(accVector[0]);
						//cout << imu_names[i] << endl;
					}
					if (imu_names[i].compare("00342323") == 0) {
						imus_data[6] = imu_filters[6].apply(-gyroVector[2]);
						imus_data[7] = imu_filters[7].apply(gyroVector[1]);
						imus_data[8] = imu_filters[8].apply(gyroVector[0]);
						imus_data[9] = imu_filters[9].apply(-accVector[2]);
						imus_data[10] = imu_filters[10].apply(accVector[1]);
						imus_data[11] = imu_filters[11].apply(accVector[0]);
						//cout << imu_names[i] << endl;
					}
					if (imu_names[i].compare("00342324") == 0) {
						// Orientacao P??? DIR: [2 -1 3]
						imus_data[12] = imu_filters[12].apply(gyroVector[1]);
						imus_data[13] = imu_filters[13].apply(-gyroVector[0]);
						imus_data[14] = imu_filters[14].apply(gyroVector[2]);
						imus_data[15] = imu_filters[15].apply(accVector[1]);
						imus_data[16] = imu_filters[16].apply(-accVector[0]);
						imus_data[17] = imu_filters[17].apply(accVector[2]);
						//cout << imu_names[i] << endl;
					}
#if IMU_DBG_LOG
					logFileHandle = fopen(filename, "a");
					if (logFileHandle != NULL) {
						float timestamp = float(xsensTimer.micro_now()) / MILLION;
						fprintf(logFileHandle, "%.5f", timestamp);
						for (size_t i = 0; i < 18; i++)
							fprintf(logFileHandle, ", %.4f", imus_data[i]);
						fprintf(logFileHandle, "\n");
						fclose(logFileHandle);
					}
#endif // IMU_DBG_LOG

					unique_lock<mutex> _(*data_struct.mtx_);
					memcpy(*data_struct.datavec_, imus_data, (DTVC_SZ * sizeof(float)));
					if (data_struct.param39_ == IMUBYPASS) {
						*(*data_struct.datavecB_ + 1) = imus_data[12]; // hum_rgtknee_vel
					}
				}
			}

			xsensTimer.tak();
		} while (!xsensTimer.end());

		if (!wirelessMasterDevice->gotoConfig())
		{
			ostringstream error;
			error << "Failed to goto config mode: " << *wirelessMasterDevice;
			throw runtime_error(error.str());
		}
		if (!wirelessMasterDevice->disableRadio())
		{
			ostringstream error;
			error << "Failed to disable radio: " << *wirelessMasterDevice;
			throw runtime_error(error.str());
		}
	}
	catch (exception const& ex)
	{
		cout << ex.what() << endl;
		cout << "****ABORT****" << endl;
		unique_lock<mutex> _(*data_struct.mtx_);
		*data_struct.param0A_ = false; // not ready anymore
		*data_struct.param1A_ = true; // aborting
		*data_struct.param3F_ = true; // finished
		return;
	}
	catch (...)
	{
		cout << "An unknown fatal error has occured. Aborting." << endl;
		cout << "****ABORT****" << endl;
		unique_lock<mutex> _(*data_struct.mtx_);
		*data_struct.param0A_ = false; // not ready anymore
		*data_struct.param1A_ = true; // aborting
		*data_struct.param3F_ = true; // finished
		return;
	}

	cout << "Closing XsControl..." << endl;
	control->close();

	for (vector<MtwCallback*>::iterator i = mtwCallbacks.begin(); i != mtwCallbacks.end(); ++i) {
		delete (*i);
	}

	{
		unique_lock<mutex> _(*data_struct.mtx_);
		*data_struct.param0A_ = false;
		*data_struct.param3F_ = true;
	}
}

int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate)
{
	if (supportedUpdateRates.empty())
		return 0;

	if (supportedUpdateRates.size() == 1)
		return supportedUpdateRates[0];

	int uRateDist = -1;
	int closestUpdateRate = -1;
	for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
	{
		const int currDist = std::abs(*itUpRate - desiredUpdateRate);
		if ((uRateDist == -1) || (currDist < uRateDist)) {
			uRateDist = currDist;
			closestUpdateRate = *itUpRate;
		}
	}
	return closestUpdateRate;
}