/////////////////////////////\_____///\///\////\/\|//\
// Leonardo Felipe Lima Santos dos Santos /\____//|\//\
// leonardo.felipe.santos@usp.br /	_____ ___  ___  \//|
// github/bitbucket qleonardolp /	  | |  | . \/   \  \/|
// *Copyright 2021-2026* \//// //\ 	| |   \ \   |_|   \|
//\////////\/\/\/\/\/\/\//\// ////\	\_'_/\_`_/__|     /
///\_]//////\/\/\/\/\/\////\_////\////|////////\//[__/

#include "AXIS.h"
#include "EPOS_NETWORK.h"
#include "QpcLoopTimer.h" // ja inclui <windows.h>
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include "LowPassFilter2p.h"
#include <processthreadsapi.h>
#include <Eigen/Core>
#include <iostream>

extern AXIS eixo_out;
extern AXIS eixo_in;
extern EPOS_NETWORK epos;
extern void HabilitaEixo(int ID);
extern void DesabilitaEixo(int ID);

void Controle(ThrdStruct &data_struct){
    using namespace std;
#if PRIORITY
    SetThreadPriority(GetCurrentThread(), data_struct.param00_);
#endif
    
    // setup stuff...

    bool isready_imu(false);
    bool isready_asg(false);
    do{ 
        {   // Controle confere se IMU e ASGD estao prontos:
            unique_lock<mutex> _(*data_struct.mtx_);
            isready_imu = *data_struct.param0A_;
            isready_asg = *data_struct.param0B_;
        } 
    } while (!isready_imu || !isready_asg);

    // Sincroniza as epos
    epos.sync();
    HabilitaEixo(2);

    // Habilita o controle de velocidade
    eixo_in.VCS_SetOperationMode(VELOCITY_MODE);
    eixo_out.ReadPDO01();
    eixo_in.ReadPDO01();
    
    {   // Controle avisa que esta pronto!
        unique_lock<mutex> _(*data_struct.mtx_);
        *data_struct.param0C_ = true;
    }

    looptimer Timer(data_struct.sampletime_);
    llint exec_time_micros = data_struct.exectime_*MILLION;
    llint t_begin = Timer.micro_now();
    do
    {
        Timer.tik();
        /* code */

        Timer.tak();
    } while (Timer.micro_now() - t_begin <= exec_time_micros);

    // ending stuff...
    DesabilitaEixo(0);
}