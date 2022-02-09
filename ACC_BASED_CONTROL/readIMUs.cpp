//////////////////////////////////////////\/////////\//
// Leonardo Felipe Lima Santos dos Santos /\     ////\/
// leonardo.felipe.santos@usp.br	_____ ___  ___  //|
// github/bitbucket qleonardolp /	| |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  	| |   \ \   |_|  /|
//\///////////////////////\// ////	\_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\/

#ifdef _WIN32
#include "QpcLoopTimer.h" // ja inclui <windows.h>
#else
#include <windows.h>
#endif
// Copyright (c) 2003-2016 Xsens Technologies B.V. or subsidiaries worldwide. All rights reserved.
#include "mastercallback.h" // Inclui Xsens device API header
#include "mtwcallback.h"
#include <conio.h>
#include "XsensEpos.h"
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include "LowPassFilter2p.h"
#include <processthreadsapi.h>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <utility>
// #include <chrono>

/*! \brief Stream insertion operator overload for XsPortInfo */
std::ostream &operator<<(std::ostream &out, XsPortInfo const &p)
{
  out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
      << std::setw(7) << p.baudrate() << " Bd"
      << ", "
      << "ID: " << p.deviceId().toString().toStdString();
  return out;
}

/*! \brief Stream insertion operator overload for XsDevice */
std::ostream &operator<<(std::ostream &out, XsDevice const &d)
{
  out << "ID: " << d.deviceId().toString().toStdString() << " (" << d.productCode().toStdString() << ")";
  return out;
}

int findClosestUpdateRate(const XsIntArray &supportedUpdateRates, const int desiredUpdateRate);

void readIMUs(ThrdStruct &data_struct)
{
    using namespace std;
#if PRIORITY
    SetThreadPriority(GetCurrentThread(), data_struct.param00_);
#endif

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
    try{
    const int desiredUpdateRate = 100; // data_struct.sampletime_;
    const int desiredRadioChannel = 25;

    WirelessMasterCallback wirelessMasterCallback; // Callback for wireless master
    vector<MtwCallback *> mtwCallbacks;            // Callbacks for mtw devices

    XsControl *control = XsControl::construct();
    if (control == 0)
    {
        cout << "Failed to construct XsControl instance." << endl;
    }

    try
    {
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
        mtwCallbacks.resize(mtwDevices.size());
        for (int i = 0; i < (int)mtwDevices.size(); ++i)
        {
            mtwCallbacks[i] = new MtwCallback(i, mtwDevices[i]);
            mtwDevices[i]->addCallbackHandler(mtwCallbacks[i]);
            string imu_id = mtwDevices[i]->deviceId().toString().toStdString();
            if (i == 0)
                cout << "IMU Usuario Coxa: " << imu_id << "\n";
            if (i == 1)
                cout << "IMU Usuario Canela: " << imu_id << "\n";
            if (i == 2)
                cout << "IMU Exo: " << imu_id << "\n";
        }
        //this_thread::sleep_for(chrono::seconds(3));
        {   // readIMUs nao espera as outras threads:
            unique_lock<mutex> _(*data_struct.mtx_);
            *data_struct.param0A_ = true; // readIMUs avisa que esta pronto!
        }

        vector<XsVector> accData(mtwCallbacks.size());
        vector<XsVector> gyroData(mtwCallbacks.size());

        //wirelessMasterCallback.mtw_event.clear(); // XsMutex here...

        float imus_data[18]; 
        // Filtros Passa Baixa para os dados das IMUs
        LowPassFilter2pFloat imu_filters[18];
        for (int i = 0; i < sizeof(imu_filters)/sizeof(LowPassFilter2pFloat); i++)
        {
          imu_filters[i].set_cutoff_frequency(desiredUpdateRate, 16);
          imu_filters[i].reset();
        }

        llint exec_time_micros = (data_struct.exectime_)*MILLION;
        // looptimer xsensTimer(data_struct.sampletime_);
        looptimer xsensTimer(desiredUpdateRate);
        llint t_begin = xsensTimer.micro_now();
        do
        {
            xsensTimer.tik();
            ///*
            for (size_t i = 0; i < mtwCallbacks.size(); ++i)
            {
                if (mtwCallbacks[i]->dataAvailable())
                {
                    XsDataPacket const *packet = mtwCallbacks[i]->getOldestPacket();
                    accData[i] = packet->calibratedAcceleration();
                    gyroData[i] = packet->calibratedGyroscopeData();

                    mtwCallbacks[i]->deleteOldestPacket();

                    if (mtwCallbacks.size() >= 3)
                    {
                      // Orientacao Exo: [3 -2 1]
                      // Orientacao Pessoa: [-3 2 1]

                      if (i == 0 || i == 1){
                        imus_data[6*i+0] = imu_filters[6*i+0].apply( gyroData[i].value(2) );
                        imus_data[6*i+1] = imu_filters[6*i+1].apply(-gyroData[i].value(1) );
                        imus_data[6*i+2] = imu_filters[6*i+2].apply( gyroData[i].value(0) );
                        imus_data[6*i+3] = imu_filters[6*i+3].apply(  accData[i].value(2) );
                        imus_data[6*i+4] = imu_filters[6*i+4].apply( -accData[i].value(1) );
                        imus_data[6*i+5] = imu_filters[6*i+5].apply(  accData[i].value(0) );
                      }
                      if (i == 2){
                        imus_data[6*i+0] = imu_filters[6*i+0].apply( gyroData[i].value(2) );
                        imus_data[6*i+1] = imu_filters[6*i+1].apply(-gyroData[i].value(1) );
                        imus_data[6*i+2] = imu_filters[6*i+2].apply( gyroData[i].value(0) );
                        imus_data[6*i+3] = imu_filters[6*i+3].apply(  accData[i].value(2) );
                        imus_data[6*i+4] = imu_filters[6*i+4].apply( -accData[i].value(1) );
                        imus_data[6*i+5] = imu_filters[6*i+5].apply(  accData[i].value(0) );
                      }
                      unique_lock<mutex> _(*data_struct.mtx_);
                      memcpy(*data_struct.datavec_, imus_data, 18*sizeof(float));
                      //*data_struct.datavec_ = imus_data;
                    }
                }
            }
            //*/
            xsensTimer.tak();
        } while (xsensTimer.micro_now() - t_begin <= exec_time_micros);

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
    catch (exception const &ex)
    {
        imu_isready = false;
        cout << ex.what() << endl;
        cout << "****ABORT****" << endl;
    }
    catch (...)
    {
        imu_isready = false;
        cout << "An unknown fatal error has occured. Aborting." << endl;
        cout << "****ABORT****" << endl;
    }

    imu_isready = false;
    cout << "Closing XsControl..." << endl;
    control->close();

    for (vector<MtwCallback *>::iterator i = mtwCallbacks.begin(); i != mtwCallbacks.end(); ++i){
        delete (*i);
    }

    }catch(...){ cout << "Unknown Xsens Error" << endl; }
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
		if ((uRateDist == -1) || (currDist < uRateDist)){
			uRateDist = currDist;
			closestUpdateRate = *itUpRate;
		}
	}
	return closestUpdateRate;
}