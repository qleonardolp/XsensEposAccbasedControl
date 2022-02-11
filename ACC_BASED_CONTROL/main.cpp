//////////////////////////////////////////\/////////\//
// Leonardo Felipe Lima Santos dos Santos /\     ////\/
// leonardo.felipe.santos@usp.br  	_____ ___  ___  //|
// github/bitbucket qleonardolp /	  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  	| |   \ \   |_|  /|
//\///////////////////////\// ////	\_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\/

#include "XsensEpos.h"
#include "QpcLoopTimer.h" // ja inclui <windows.h>
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include <processthreadsapi.h>
#include <stdexcept>
#include <iostream>
#include <conio.h>

void Interface();
void readIMUs(ThrdStruct &data_struct);
void readFTSensor(ThrdStruct &data_struct);
void qASGD(ThrdStruct &data_struct);
void Controle(ThrdStruct &data_struct);
void Logging(ThrdStruct &data_struct);

// DEBUGGING DEFINES:
#define PRIORITY     0
#define ISREADY_WAIT 0
#define EXEC_TIME   50

// Threads Sample Time:
#define IMU_SMPLTM  0.0100 // "@100 Hz", actually is defined by Xsens 'desiredUpdateRate'
#define ASGD_SMPLTM 0.0050 //  @200  Hz
#define CTRL_SMPLTM 0.0002 //  @5000 Hz
#define LOG_SMPLTM  0.0050 //  @200  Hz 
#define FT_SMPLTM   0.0010 //  @1000 Hz, pode chegar a 7kHz ... 
// Threads Priority:
#define IMU_PRIORITY   -1 //
#define ASGD_PRIORITY  -1 //
#define CTRL_PRIORITY  -2 //
#define LOG_PRIORITY    0 // 

// Global vars shared with Interface() funct
ThrdStruct imu_struct, asgd_struct, ati_struct;
ThrdStruct control_struct, logging_struct;
short imu_isready(false);
short asgd_isready(false);
short control_isready(false);
short logging_isready(false);
short ftsensor_isready(false);

bool execution_end(false);
int  execution_time = EXEC_TIME;

int main()
{
  using namespace std;
#if PRIORITY
  DWORD dwPriority, dwError;
  if (!SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS)){
      dwError = GetLastError();
      printf("Error %lu! \n", dwError);
  } else{
      dwPriority = GetPriorityClass( GetCurrentProcess() );
      printf("Process priority class: %lu \n", dwPriority);
  }
#endif

  cout << "INICIALIZANDO COMUNICACAO CANOpen COM AS EPOS" << endl;
  IniciaRedeCan();

  mutex imu_mtx;
  mutex ati_mtx;
  float shared_data[18];
  float ati_data[6];
  //initialize shared_data:
  for (int i = 0; i < sizeof(shared_data)/sizeof(float); i++){
    shared_data[i] = i*i;
    if (i < 6)
      ati_data[i] = 0;
  }

  imu_struct.sampletime_ = IMU_SMPLTM;
  imu_struct.param00_  = IMU_PRIORITY;
  *(imu_struct.datavec_) = shared_data;

  asgd_struct.sampletime_ = ASGD_SMPLTM;
  asgd_struct.param00_  = ASGD_PRIORITY;
  *(asgd_struct.datavec_) = shared_data;

  control_struct.sampletime_ = CTRL_SMPLTM;
  control_struct.param00_  = CTRL_PRIORITY;
  *(control_struct.datavec_) = shared_data;

  logging_struct.sampletime_ = LOG_SMPLTM;
  logging_struct.param00_  = LOG_PRIORITY;
  *(logging_struct.datavec_) = shared_data;

  ati_struct.sampletime_ = FT_SMPLTM;
  *ati_struct.datavec_ = ati_data;
  ati_struct.mtx_ = &ati_mtx;

  imu_struct.mtx_ = asgd_struct.mtx_ = control_struct.mtx_ = logging_struct.mtx_ = &imu_mtx;
  // Readiness Flags:
  logging_struct.param0A_ = control_struct.param0A_ = asgd_struct.param0A_ = imu_struct.param0A_ = &imu_isready;
  logging_struct.param0B_ = control_struct.param0B_ = asgd_struct.param0B_ = &asgd_isready;
  logging_struct.param0B_ = control_struct.param0B_ = &control_isready;
  logging_struct.param0E_ = control_struct.param0E_ = &ftsensor_isready;
  logging_struct.param0D_ = &logging_isready; // "apenas logging sabe que se aprontou..."

  thread thr_imus;
  thread thr_qasgd;
  thread thr_controle;
  thread thr_logging;
  thread thr_ftsensor;

  /*
    - TODO: Programar thread com GainScan geral...
  */

  do
  {
    Interface();
    // Fire Threads:
    if (!imu_isready)  thr_imus = thread(readIMUs, imu_struct);
    if (!asgd_isready) thr_qasgd = thread(qASGD, asgd_struct);
    if (!logging_isready)  thr_logging = thread(Logging, logging_struct);
    if (!control_isready)  thr_controle = thread(Controle, control_struct);
    if (!ftsensor_isready) thr_ftsensor = thread(readFTSensor, ati_struct);

    // main waits while the threads execute thier tasks...
    if (thr_controle.joinable()) thr_controle.join();
    if (thr_logging.joinable()) thr_logging.join();
    if (thr_ftsensor.joinable()) thr_ftsensor.join();
    if (thr_qasgd.joinable()) thr_qasgd.join();
    if (thr_imus.joinable()) thr_imus.join();

    system("cls");
  } while (!execution_end);

  epos.StopPDOS(1);
  this_thread::sleep_for(chrono::milliseconds(1234));
  cout << "Successful exit. Press [ENTER] to quit." << endl;
  cin.get();
  return 0;
}

// Funcao de interface com o usuario:
void Interface()
{
  short option = 0;
  using namespace std;

  cout << "//////////////////////////////////////////\\/////////\\/"   << endl;
  cout << "// INTERFACE DE CONTROLE EXO-TAU  /       /\\     ////\\"   << endl;
  cout << "// EESC-USP                      / _____ ___  ___  //|"     << endl;
  cout << "// RehabLab                     /  | |  | . \\/   \\  /|"   << endl;
  cout << "// *Copyright 2021-2026* \\//// //  | |   \\ \\   |_|  /|"  << endl;
  cout << "//\\///////////////////////\\// //// \\_'_/\\_`_/__|   ///" << endl;
  cout << "///\\///////////////////////\\ //////////////////\\/////\\" << endl;

  cout << " Escolha uma opcao: \n";
  cout << " [01]: Controle com IMUs + F/T \n";
  cout << " [02]: Controle com IMUs \n";
  cout << " [03]: Controle sem IMUs \n";
  cout << " [04]: Leitura IMUs \n";
  cout << " [05]: Leitura F/T  \n";
  cout << " Ou zero (0) para finalizar. \n";
  cin >> option;

  switch (option)
  {  
  case 1:
    // tudo roda!
    break;
  case 2:
    ftsensor_isready = true;
    break;
  case 3:
    imu_isready = true;
    asgd_isready = true;
    ftsensor_isready = true;
    break;
  case 4:
    control_isready  = true;
    ftsensor_isready = true;
    break;
  case 5:
    control_isready  = true;
    imu_isready = true;
    break;
  case 0:
  default:
    imu_isready  = true;
    asgd_isready = true;
    logging_isready  = true;
    control_isready  = true;
    ftsensor_isready = true;
    execution_end = true;
    break;
  }

  if (!execution_end) {
    cout << "\n Defina o tempo de execucao em segundos: \n";
    cin >> execution_time;
    imu_struct.exectime_  = execution_time;
    asgd_struct.exectime_ = execution_time;
    control_struct.exectime_ = execution_time;
    logging_struct.exectime_ = execution_time;
    ati_struct.exectime_ = execution_time;

    // Escolher tipo de controle:
    system("cls");
    if (!control_isready) {
      cout << " Escolha uma controlador: \n";
      cout << " [01]: Controle com IMUs + F/T \n";
      cout << " [02]: Controle com IMUs \n";
      cout << " [03]: Controle sem IMUs \n";
      cout << " [04]: Leitura IMUs \n";
      cout << " [05]: Leitura F/T  \n";
      cin >> option;
      control_struct.param01_ = option;
    }
    cout << " Iniciando Threads..." << endl;
  } 
}