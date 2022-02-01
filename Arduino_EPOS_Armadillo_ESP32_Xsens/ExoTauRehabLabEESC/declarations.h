#ifndef DEFINIONS_H
#define DEFINIONS_H

#include <time.h>

#ifdef _WIN32
#define CLC "cls"
#endif

#ifdef linux
#define CLC "clear"
#endif

int COMMAND_KEY = 0;
int COMMAND_SETPOINT = 0;
int COMMAND_CONTROLLER = 0;

int exec_time = 0;

//CONSTANTES
#define pi 3.1416
const int N = 150;            // Redução do sistema
const int encoder_in = 4096;  // Resolução do encoder do motor
const int encoder_out = 2000; // Resolução do encoder de saída
const int ks = 104;           // Constante elástica

//VARIAVEIS
double Im;      // Corrente enviada ao motor
double theta_m; // Posição do motor
double theta_l; // Posição da saída/carga
double theta_c; // Posição da coroa
double omega_m; // Velocidade do motor
double omega_l; // Velocidade da saída/carga

//DATALOGGERS
#define n_datalogs_ard 6
#define n_datalogs_esp 6
#define n_datalogs_exo 7
#define n_datalogs_imu 50
double *datalog_ard;
double *datalog_esp;
double *datalog_exo;
double *datalog_imu;

double *setpoints_theta;


char *port_name = "\\\\.\\COM34"; // Arduino Uno - Exo
char *port_name_esp = "\\\\.\\COM40"; // Arduino Uno - Exo

///bool flag_arduino_multi_emg = true;
///bool flag_arduino_multi_ard = true;
///bool flag_arduino_multi_esp = true;
///bool flag_arduino_multi_exo = true;
///bool flag_arduino_multi_imu = true;

int total_time_imu = 0;
int total_time_esp = 0;

//bool aborting_emg = false;
//bool aborting_ard = false;
//bool aborting_esp = false;
//bool aborting_exo = false;
//bool aborting_imu = false;

int OPC_K = 0;

std::atomic<bool> flag_arduino_multi_emg(true);
std::atomic<bool> flag_arduino_multi_ard(true);
std::atomic<bool> flag_arduino_multi_esp(true);
std::atomic<bool> flag_arduino_multi_exo(true);
std::atomic<bool> flag_arduino_multi_imu(true);
std::atomic<bool> aborting_emg(false);
std::atomic<bool> aborting_ard(false);
std::atomic<bool> aborting_esp(false);
std::atomic<bool> aborting_exo(false);
std::atomic<bool> aborting_imu(false);


#ifdef _WIN32
//LARGE_INTEGER tick_after, tick_before, TICKS_PER_SECOND;
LARGE_INTEGER tick_after_exo, tick_before_exo;
LARGE_INTEGER tick_after_ard, tick_before_ard;
#endif

class loop_timers
{
public:
    double SAMPLE_TIME;
    int samples_per_second;
    //clock_t initial_time;
    //clock_t final_time;
    long long int initial_time;
    long long int final_time;
    double t;
    long long int tempo;
    long long int tempo2;
    long long int time_now;

#ifdef _WIN32
    LARGE_INTEGER tick_after, tick_before, TICKS_PER_SECOND;
    long long int ticksSampleTime;
#endif

    loop_timers(double s_time)
    {
        this->SAMPLE_TIME = s_time;
#ifdef _WIN32
      	QueryPerformanceFrequency(&TICKS_PER_SECOND);
	      this->ticksSampleTime = TICKS_PER_SECOND.QuadPart * SAMPLE_TIME;
#endif
        this->samples_per_second = (int)(1 / SAMPLE_TIME);
        this->tempo = clock();
        this->tempo2 = clock();
    };

    void start_timer()
    {
#ifdef _WIN32
        
        QueryPerformanceCounter(&tick_before);
        this->initial_time = tick_before.QuadPart;
        this->final_time = tick_before.QuadPart + 1 * this->ticksSampleTime;
        
        /*
        this->initial_time = clock();
        this->final_time = clock() + this->SAMPLE_TIME * CLOCKS_PER_SEC;
        */
#endif

#ifdef linux
        this->initial_time = clock();
        this->final_time = clock() + this->SAMPLE_TIME * CLOCKS_PER_SEC;
#endif
    };

    void wait_final_time()
    {
#ifdef _WIN32
        
        QueryPerformanceCounter(&tick_after);
        while (this->final_time > this->tick_after.QuadPart)
            QueryPerformanceCounter(&tick_after);

        this->t = ((double)(this->tick_after.QuadPart - this->initial_time)) / TICKS_PER_SECOND.QuadPart;
        
        /*
        this->time_now = clock();
        while (this->final_time > this->time_now)
            this->time_now = clock();

        this->t = this->time_now - this->initial_time;
        */
#endif
#ifdef linux
        this->time_now = clock();
        while (this->final_time > this->time_now)
            this->time_now = clock();

        this->t = this->time_now - this->initial_time;
#endif
        this->tempo2 = clock() - this->tempo;
    };

    clock_t get_current_time()
    {
        this->time_now = clock();
        return this->time_now;
    };
};

double SAMPLE_TIME_ARD = 0.010;
int samples_per_second_ard = (int)(1 / SAMPLE_TIME_ARD);
int total_time_ard;
long long int initial_time_ard;
long long int final_time_ard;
long int t_ard;
long int tempo_ard;
long int tempo2_ard;
clock_t time_now_ard; // Clock de segundos

double SAMPLE_TIME_EXO = 0.005;
int samples_per_second_exo = (int)(1 / SAMPLE_TIME_EXO);
int total_time_exo;
long long int initial_time_exo;
long long int final_time_exo;
long int t_exo;
long int tempo2_exo;
clock_t time_now_exo; // Clock de segundos

double SAMPLE_TIME_IMU = 0.010;
int samples_per_second_imu = (int)(1 / SAMPLE_TIME_IMU);

double SAMPLE_TIME_EMG = 0.001;
int samples_per_second_emg = (int)(1 / SAMPLE_TIME_EMG);

double SAMPLE_TIME_ESP = 0.010;
int samples_per_second_esp = (int)(1 / SAMPLE_TIME_ESP);

clock_t endwait; // Clock de segundos

#endif //DEFINIONS_H