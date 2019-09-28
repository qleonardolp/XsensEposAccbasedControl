#ifndef GENERAL_HEADER_H
#define GENERAL_HEADER_H

// CONSTANTES

#define		  GEAR_RATIO		  150.0		    // Redução do Sistema
#define		  ENCODER_IN		  4096		    // Resolução do encoder do motor
#define		  ENCODER_OUT		  2048		    // Resolução do encoder de saída
#define		  STIFFNESS			  104.0			// Constante da mola SEA [N.m/rad]
#define		  SAMPLE_TIME		  0.005		    // Tempo de Amostragem [s]

#define		  CURRENT_MAX		  3.1000		// Max corrente nominal no motor Maxon RE40 [A]
#define		  TORQUE_CONST		60.300		// Constante de torque do motor RE40	[N.m/mA]


#define     GRAVITY         9.8066      // [m/s^2]
#define     INERTIA_EXO     0.0655      // [Kg.m^2], +- 0.0006, estimado em 2019-08-21
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


// ENDEREÇAMENTO DA BASE DE DADOS CAN

char* CAN_INTERFACE =		"CAN1";
char* CAN_DATABASE =	"database";
char* CAN_CLUSTER =		  "NETCAN";
char* NET_ID_SERVO_01 =		   "1";
char* NET_ID_SERVO_02 =		   "2";

//INICIALIZANDO O QUERY PERFORMANCE PARA CONTROLE DOS CICLOS DE 5MS

LARGE_INTEGER tick_after, tick_before, TICKS_PER_SECOND;
long long int ticksSampleTime, final_time;
int total_time;

//INICIALIZAÇÃO DA REDE CAN

EPOS_NETWORK  epos (CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER);

//INICIALIZAÇÃO DAS EPOS

AXIS eixo_out(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_02);
AXIS eixo_in(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_01);

clock_t endwait;	// Clock de segundos

FILE * logger;		// Ponteiro para arquivo


#endif /* GENERAL_HEADER_H */