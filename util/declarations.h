//ENDEREÇAMENTO DA BASE DE DADOS CAN

char *CAN_INTERFACE = "CAN1";
char *CAN_DATABASE = "database";
char *CAN_CLUSTER = "NETCAN";
char *NET_ID_SERVO_01 = "1";
char *NET_ID_SERVO_02 = "2";

int COMMAND_KEY = 0;
int COMMAND_KEY2 = 0;

int loopTime = 5;

int i_dt=0;
int i=0;

double datalog01[60000];
double datalog02[60000];
double datalog03[60000];
double datalog04[60000];
double datalog05[60000];
double datalog06[60000];
double datalog07[60000];
double datalog08[60000];
double datalog09[60000];
double datalog10[60000];
double datalog11[60000];
double datalog12[60000];
double datalog13[60000];
double datalog14[60000];
double datalog15[60000];
double datalog16[60000];
double datalog17[60000];
double datalog18[60000];
double datalog19[60000];
double datalog20[60000];

double setpoints[60000];

//CONSTANTES

int N=150;						  // Redução do sistema
int encoder_in=4096;			  // Resolução do encoder do motor
int encoder_out=2048;		      // Resolução do encoder de saída
int ks=104;						  // Constante elástica, ou 84, 94

double pi=3.141592;

double Ts=0.005;				  // tempo de amostragem


int exec_time = 0;


//VARIAVEIS

double kv=0.0;						// rigidez virtual
double bv=0.0;						// amortecimento virtual

int ZERO_01=0;                      // Zero do eixo_out
int ZERO_02=0;                      // Zero do eixo_in

float DADOS=0.0;                     

double theta_l=0.0;                 // posição da carga

double theta_m=0.0;					// posição do motor
double omega_m=0.0;					// velocidade do motor

double omega=0.0;

double theta_c=0.0;                 // posição da coroa

double theta_l_ant=0.0;				// posição do eixo_out no instante [k-1]
double theta_ld=0.0;				// velocidade de saída desejada

double Im=0.0;						// corrente enviada 

double omega_ld=0.0;				// velocidade de saída desejada
double omega_l=0.0;					// velocidade de saída
double omega_lf=0.0;				// velocidade de saída filtrada
double omega_lant=0.0;				// velocidade de saída no instante [k-1]
double omega_lfant=0.0;				// velocidade de saída filtrada no instante [k-1]
double omega_lant2=0.0;				// velocidade de saída no instante [k-2]
double omega_lfant2=0.0;			// velocidade de saída filtrada no instante [k-2]

double a2=-0.9428;
double a3=0.3333;
double b1=0.0976;
double b2=0.1953;
double b3=0.0976;

double torque_l=0.0;                // torque na carga
double torque_lf=0.0;				// torque na carga filtrado
double torque_lant=0.0;				// torque na carga no instante [k-1]
double torque_lfant=0.0;			// torque na carga filtrado no instante [k-1]
double torque_lant2=0.0;			// torque na carga no instante [k-2]
double torque_lfant2=0.0;			// torque na carga filtrado no instante [k-2]

double c2=-1.889;
double c3=0.8949;
double d1=0.0015;
double d2=0.0029;
double d3=0.0015;

double torque_d=0.0;                // torque desejado
double torque_m=0.0;				// torque no motor

double controle=0.0;		        // entrada de controle
double controle_ant=0.0;		    // entrada de controle no instante [k-1]

double controle2=0.0;		        // entrada de controle
double controle2_ant=0.0;		    // entrada de controle no instante [k-1]
double controle2_ant2=0.0;

double erro=0.0;				    // erro
double erro_ant=0.0;				// erro no instante [k-1]
double erro_ant2=0.0;				// erro no instante [k-2]

double kp = 370;				    // Ganho proporcional
double ki = 3.5;					// Ganho integrativo
double kd = 0.0;					// Ganho derivativo

double m = 1.0;
double g = 9.81;
double l = 0.59;



//INICIALIZANDO O QUERY PERFORMANCE PARA CONTROLE DOS CICLOS DE 5MS

LARGE_INTEGER tick_after, tick_before, TICKS_PER_SECOND;
long long int ticksSampleTime, final_time;
double SAMPLE_TIME = 0.005;
int total_time;

//INICIALIZAÇÃO DA REDE CAN

EPOS_NETWORK  epos (CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER);

//INICIALIZAÇÃO DAS EPOS

AXIS eixo_out(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_02);
AXIS eixo_in(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_01);

clock_t endwait;	// Clock de segundos

FILE * pFile;		// Ponteiro para arquivo
