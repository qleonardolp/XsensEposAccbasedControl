//*******************************************************************************
//
//	CONTROLE DO ATUADOR ELÁSTICO EM SÉRIE ROTACIONAL
//	DATA 13/12/2013
//
//********************************************************************************

#include <WinSock2.h>
#include "AXIS.h"
#include "EPOS_NETWORK.h"
#include "declarations.h"



void POSICIONA_J(int T){
	
	
		//Inicializa o loop
		total_time = 0;
   // int indice = 0;
		loopTime = 0;

		//ZERA OS DATALOGGERS
		printf("\nInicializando datalog.....");
		//Inicializando o vetor de datalog
		for (i_dt=0; i_dt<60000; i_dt++){
           datalog01[i_dt] = 0.0; 
		       datalog02[i_dt] = 0.0; 
		       datalog03[i_dt] = 0.0; 
		       datalog04[i_dt] = 0.0; 
    	  	 datalog05[i_dt] = 0.0;
           datalog06[i_dt] = 0.0;
           datalog07[i_dt] = 0.0;
           datalog08[i_dt] = 0.0;
           datalog09[i_dt] = 0.0;
           datalog10[i_dt] = 0.0;
		       datalog11[i_dt] = 0.0; 
		       datalog12[i_dt] = 0.0; 
		       datalog13[i_dt] = 0.0; 
		       datalog14[i_dt] = 0.0; 
		       datalog15[i_dt] = 0.0;
           datalog16[i_dt] = 0.0;
           datalog17[i_dt] = 0.0;
           datalog18[i_dt] = 0.0;
           datalog19[i_dt] = 0.0;
           datalog20[i_dt] = 0.0;
        }
        i_dt = 0;


		//LOOP DE EXECUÇÃO de 5 ms
		do{

			//CALCULA O TEMPO MÁXIMO DE EXECUÇÃO DO LOOP
			QueryPerformanceCounter(&tick_before);
			final_time = tick_before.QuadPart + 1*ticksSampleTime;

			//Incrementa o loop time
			loopTime += 1;

				//Sincroniza a CAN
				epos.sync();
				
				eixo_in.ReadPDO01();
				Im = eixo_in.PDOgetActualCurrent();
		
				datalog01[total_time] = kv;
				datalog02[total_time] = bv;
				datalog03[total_time] = theta_ld;
				datalog04[total_time] = theta_l;
				datalog05[total_time] = theta_c;
				datalog06[total_time] = theta_m;
				datalog07[total_time] = omega_ld;
				datalog08[total_time] = omega_lf;
				datalog09[total_time] = omega_m;								
				datalog10[total_time] = torque_d;
				datalog11[total_time] = torque_l;
				datalog12[total_time] = torque_lf;				
				datalog13[total_time] = Im;
				datalog14[total_time] = controle;
				datalog15[total_time] = controle2;
				datalog16[total_time] = total_time;

				eixo_in.ReadPDO01();
				Im = eixo_in.PDOgetActualCurrent();
				theta_c = ((eixo_in.PDOgetActualPosition()-ZERO_02)*2*pi)/(encoder_in*N);
				theta_m = ((eixo_in.PDOgetActualPosition()-ZERO_02)*2*pi)/(encoder_in);
				        
				eixo_in.ReadPDO02();				
				omega_m = eixo_in.PDOgetActualVelocity();

				eixo_out.ReadPDO01();
				theta_l = ((-eixo_out.PDOgetActualPosition()-ZERO_01)*2*pi)/encoder_out;

				eixo_out.ReadPDO02();				
				omega = -eixo_out.PDOgetActualVelocity();


				//---------------Controle de Impedância--------------------//
			    
				//theta_ld = 45*(pi/180)*cos(2*pi*0.2*total_time*5e-3) - 45*(pi/180); //[rad]
      
				//int init_time = 30; // tempo inicial [seg] impedancia zero
		    
        /*
				if ( total_time > init_time*200 && kv < 60 && total_time < (exec_time - init_time)*200 ){
					kv = kv + 0.01;
					//bv = 0;
				}
				if ( total_time > (exec_time - init_time)*200 ){
					kv = kv - 0.01;
					//bv = 0;
				}
				
				if (kv >= 100){
					kv = 100;
				}

				if (kv < 0){
					kv = 0;
				}
        */

        --------------------------------------------------------

				
				
				omega_l = (theta_l-theta_l_ant)/Ts;
				omega_lf = -c2*omega_lfant-c3*omega_lfant2+d1*omega_l+d2*omega_lant+d3*omega_lant2;

				//---------------------------------------------------------//	
        kv = 0;
        bv = 0;
        
        //torque_d = -kv*(theta_l-theta_ld) - bv*(omega_lf-omega_ld);
          
       //torque_d = m*g*l*cos(theta_l);

       torque_d = 0.4*setpoints[total_time];



/*				if ( theta_l < theta_ld + (5*pi/180) && theta_l > theta_ld - (5*pi/180) ){
					torque_d = 0;
				}
				else if ( theta_l >= theta_ld + (5*pi/180) )
				{
					torque_d = -kv*(theta_l-(theta_ld + 5*pi/180)) - bv*(omega_lf-omega_ld);
				}
				else if ( theta_l <= theta_ld - (5*pi/180) )
				{
					torque_d = -kv*(theta_l-(theta_ld - 5*pi/180)) - bv*(omega_lf-omega_ld);
				}
*/


				torque_l = ks*(theta_c - theta_l);
				torque_lf = -a2*torque_lfant-a3*torque_lfant2+b1*torque_l+b2*torque_lant+b3*torque_lant2;

				erro = (torque_d-torque_lf);

          controle = controle_ant + kp*(erro - erro_ant) + ki*Ts*erro + (kd/Ts)*(erro - 2*erro_ant + erro_ant2);

        //controle2 = (0.9912*controle2_ant + 0.005273*controle2_ant2 + 385*erro_ant - 383.7*erro_ant2); // ks = 84 Nm/rad
        //controle2 = (0.9884*controle2_ant + 0.007935*controle2_ant2 + 357.1*erro_ant - 355.8*erro_ant2); // ks = 94 Nm/rad
				//controle2 = (0.9871*controle2_ant + 0.009259*controle2_ant2 + 333.2*erro_ant - 331.9*erro_ant2); // ks = 104 Nm/rad

        controle = 0*controle;

				//envia o controle ao motor
				eixo_in.PDOsetVelocitySetpoint(int(controle));
				//eixo_in.PDOsetVelocitySetpoint(int(controle2));
				eixo_in.WritePDO02();

				erro_ant2 = erro_ant;
				erro_ant = erro;

				theta_l_ant=theta_l;

				controle_ant = controle;

				controle2_ant2 = controle2_ant;
				controle2_ant = controle2;

				omega_lant2=omega_lant;
				omega_lant=omega_l;

				omega_lfant2=omega_lfant;
				omega_lfant=omega_lf;

				torque_lant2=torque_lant;
				torque_lant=torque_l;

				torque_lfant2=torque_lfant;
				torque_lfant=torque_lf;

				
			//incrementa contador de tempo
			total_time = total_time+1;
    /*  indice += 1;
        if(indice > 444){
        indice = 0;
        } 
        */
			//AGUARDA FIM DE TEMPO DE EXECUÇÃO DE TAREFA
			QueryPerformanceCounter(&tick_after);
			while (final_time > tick_after.QuadPart) QueryPerformanceCounter(&tick_after);

		}while (total_time < T); // Continua por até a ocorrência de erros total_time = 2
				
		//Zera o comando do motor
		eixo_in.PDOsetVelocitySetpoint(0);
		eixo_in.WritePDO02();
                
		//SALVA OS DATALOGGERS
		//Grava o log de posição para arquivo
		pFile = fopen ("kv.dat","w");
		for (i_dt=0;i_dt<60000;i_dt++)
			{
				fprintf(pFile, "\n%.5f", datalog01[i_dt]);
			}
		fclose (pFile);

		pFile = fopen ("bv.dat","w");
		for (i_dt=0;i_dt<60000;i_dt++)
			{
				fprintf(pFile, "\n%.5f", datalog02[i_dt]);
			}
		fclose (pFile);
		
		pFile = fopen ("theta_ld.dat","w");
		for (i_dt=0;i_dt<60000;i_dt++)
			{
				fprintf(pFile, "\n%.5f", datalog03[i_dt]);
			}
		fclose (pFile);
		pFile = fopen ("theta_l.dat","w");
		for (i_dt=0;i_dt<60000;i_dt++)
			{
				fprintf(pFile, "\n%.5f", datalog04[i_dt]);
			}
		fclose (pFile);
		pFile = fopen ("theta_c.dat","w");
		for (i_dt=0;i_dt<60000;i_dt++)
			{
				fprintf(pFile, "\n%.5f", datalog05[i_dt]);
			}
		fclose (pFile);

		pFile = fopen ("theta_m.dat","w");
		for (i_dt=0;i_dt<60000;i_dt++)
			{
				fprintf(pFile, "\n%.5f", datalog06[i_dt]);
			}
		fclose (pFile);

		pFile = fopen ("omega_ld.dat","w");
		for (i_dt=0;i_dt<60000;i_dt++)
			{
				fprintf(pFile, "\n%.5f", datalog07[i_dt]);
			}
		fclose (pFile);

		pFile = fopen ("omega_lf.dat","w");
		for (i_dt=0;i_dt<60000;i_dt++)
			{
				fprintf(pFile, "\n%.5f", datalog08[i_dt]);
			}
		fclose (pFile);

		pFile = fopen ("omega_m.dat","w");
		for (i_dt=0;i_dt<60000;i_dt++)
			{
				fprintf(pFile, "\n%.5f", datalog09[i_dt]);
			}
		fclose (pFile);

		pFile = fopen ("torque_d.dat","w");
		for (i_dt=0;i_dt<60000;i_dt++)
			{
				fprintf(pFile, "\n%.5f", datalog10[i_dt]);
			}
		fclose (pFile);

		pFile = fopen ("torque_l.dat","w");
		for (i_dt=0;i_dt<60000;i_dt++)
			{
				fprintf(pFile, "\n%.5f", datalog11[i_dt]);
			}
		fclose (pFile);

		pFile = fopen ("torque_lf.dat","w");
		for (i_dt=0;i_dt<60000;i_dt++)
			{
				fprintf(pFile, "\n%.5f", datalog12[i_dt]);
			}
		fclose (pFile);

		pFile = fopen ("Im.dat","w");
		for (i_dt=0;i_dt<60000;i_dt++)
			{
				fprintf(pFile, "\n%.5f", datalog13[i_dt]);
			}
		fclose (pFile);

		pFile = fopen ("controle.dat","w");
		for (i_dt=0;i_dt<60000;i_dt++)
			{
				fprintf(pFile, "\n%.5f", datalog14[i_dt]);
			}
		fclose (pFile);

		pFile = fopen ("controle2.dat","w");
		for (i_dt=0;i_dt<60000;i_dt++)
			{
				fprintf(pFile, "\n%.5f", datalog15[i_dt]);
			}
		fclose (pFile);

		pFile = fopen ("total_time.dat","w");
		for (i_dt=0;i_dt<60000;i_dt++)
			{
				fprintf(pFile, "\n%.5f", datalog16[i_dt]);
			}
		fclose (pFile);

		loopTime = 0;

}

void Habilita_Eixo(int ID){
	
    if ((ID==2) | (ID==0)){
        
        eixo_in.PDOsetControlWord_SwitchOn(false);
		eixo_in.PDOsetControlWord_EnableVoltage(true);
		eixo_in.PDOsetControlWord_QuickStop(true);
		eixo_in.PDOsetControlWord_EnableOperation(false);
		eixo_in.WritePDO01();
        
		printf("\nENERGIZANDO O MOTOR 2 E HABILITANDO O CONTROLE");
        
		endwait = clock () + 0.5 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}
        
		eixo_in.PDOsetControlWord_SwitchOn(true);
		eixo_in.PDOsetControlWord_EnableVoltage(true);
		eixo_in.PDOsetControlWord_QuickStop(true);
		eixo_in.PDOsetControlWord_EnableOperation(false);
		eixo_in.WritePDO01();
		
		endwait = clock () + 0.5 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}
        
		eixo_in.PDOsetControlWord_SwitchOn(true);
		eixo_in.PDOsetControlWord_EnableVoltage(true);
		eixo_in.PDOsetControlWord_QuickStop(true);
		eixo_in.PDOsetControlWord_EnableOperation(true);
		eixo_in.WritePDO01();
    
    }
    
}

void Desabilita_Eixo(int ID){
 
    if ((ID==2) | (ID==0)){
		printf("\nDESABILITANDO O MOTOR E CONTROLE");
        
		eixo_in.PDOsetControlWord_SwitchOn(true);
		eixo_in.PDOsetControlWord_EnableVoltage(true);
		eixo_in.PDOsetControlWord_QuickStop(true);
		eixo_in.PDOsetControlWord_EnableOperation(false);
		eixo_in.WritePDO01();
        
		endwait = clock () + 0.5 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}
        
		eixo_in.PDOsetControlWord_SwitchOn(false);
		eixo_in.PDOsetControlWord_EnableVoltage(true);
		eixo_in.PDOsetControlWord_QuickStop(true);
		eixo_in.PDOsetControlWord_EnableOperation(false);
		eixo_in.WritePDO01();
		
    }

}

int main(int argc, char** argv) {

	QueryPerformanceFrequency(&TICKS_PER_SECOND);
	ticksSampleTime= TICKS_PER_SECOND.QuadPart * SAMPLE_TIME;

	//START DE TRANSMISSÃO DA REDE
	epos.StartPDOS(1);
	epos.StartPDOS(2);
	epos.StartPDOS(3);
	epos.StartPDOS(4);
	epos.StartPDOS(5);
    epos.StartPDOS(1);
    epos.StartPDOS(2);
    epos.StartPDOS(3);
	epos.StartPDOS(4);
	epos.StartPDOS(5);

	//INICIALIZACAO DE TELA
	printf("********************************************************************************");
	printf("*                SISTEMA DE CONTROLE DE MOTOR - EPOS CANOpen                   *");
	printf("*                 ESCOLA DE ENGENHARIA DE SAO CARLOS - USP                     *");
	printf("*                   LABORATORIO DE MANIPULACAO ROBOTICA                        *");
	printf("********************************************************************************");

	printf("\n\n\nINICIALIZANDO COMUNICACAO CANOpen COM AS EPOS");

	//Inicializando a comunicação com os eixos
	for (i=0;i<10;i++){

	//Aguarda tempo
	endwait = clock () + 1 * CLOCKS_PER_SEC ;
	while (clock() < endwait) {}
	
	//Sincroniza as epos
	epos.sync();
	
	eixo_out.ReadPDO01();
	eixo_in.ReadPDO01();

	printf(".");

	}
	
	//LOOP DE CONTROLE DE COMANDOS
	do{
	system("cls");
	printf("********************************************************************************");
	printf("*                SISTEMA DE CONTROLE DE MOTOR - EPOS CANOpen                   *");
	printf("*                 ESCOLA DE ENGENHARIA DE SÃO CARLOS - USP                     *");
	printf("*                   LABORATÓRIO DE MANIPULACAO ROBOTICA                        *");
	printf("********************************************************************************");	
	printf("\n");
	printf("\n");
	printf("\n");
	printf("\nSELECIONE UMA OPCAO:");
	printf("\n");
	printf("\n [00] - ENCERRAR PROGRAMA");
	printf("\n [01] - LEITURA DA STATUS WORD");
	printf("\n [02] - RESET DE FALHAS");
	printf("\n [03] - LEITURA DA POSICAO ATUAL");
	printf("\n [04] - DEFINE POSICAO DE ORIGEM DO EIXO");
	printf("\n [05] - INICIALIZA CONTROLE");
	printf("\n");

	printf("\nOPCAO: ");

	//VERIFICA O COMANDO DO OPERADOR
	scanf("%d", &COMMAND_KEY);

	//************************************************************************************************************************
	// LEITURA DA STATUS WORD
	//************************************************************************************************************************

	if (COMMAND_KEY == 1) {
		//system("cls");
		printf("\n");
		printf("\n");
		printf("********************************************************************************");
		printf("*                SISTEMA DE CONTROLE DE MOTOR - EPOS CANOpen                   *");
		printf("********************************************************************************");
		printf("\n");
		printf("\n");

		//ZERA CONTADOR DE CICLOS
		total_time = 0;

        printf("\n********************************************************************************");
		printf("\nLEITURA DO STATUS WORD EIXO 01");
		eixo_out.ReadPDO01();
		eixo_in.ReadPDO01();

		printf("\n   1   2");
		printf("\n   %d   %d  >> Ready to Swicht On", eixo_out.PDOgetStatusWord_Ready2SwitchOn(), eixo_in.PDOgetStatusWord_Ready2SwitchOn()); 
		printf("\n   %d   %d  >> Swichted On", eixo_out.PDOgetStatusWord_SwtchedOn(), eixo_in.PDOgetStatusWord_SwtchedOn()); 
		printf("\n   %d   %d  >> Operation Enabled", eixo_out.PDOgetStatusWord_OperationEnabled(), eixo_in.PDOgetStatusWord_OperationEnabled()); 
		printf("\n   %d   %d  >> Fault", eixo_out.PDOgetStatusWord_Fault(), eixo_in.PDOgetStatusWord_Fault()); 
		printf("\n   %d   %d  >> Voltage Enabled", eixo_out.PDOgetStatusWord_VoltageEnabled(), eixo_in.PDOgetStatusWord_VoltageEnabled()); 
		printf("\n   %d   %d  >> Quick Stop", eixo_out.PDOgetStatusWord_QuickStop(), eixo_in.PDOgetStatusWord_QuickStop()); 
		printf("\n   %d   %d  >> Swicth On Disabled", eixo_out.PDOgetStatusWord_SwitchOnDisable(), eixo_in.PDOgetStatusWord_SwitchOnDisable()); 
		printf("\n   %d   %d  >> NMT Remote", eixo_out.PDOgetStatusWord_RemoteNMT(), eixo_in.PDOgetStatusWord_RemoteNMT()); 
		printf("\n   %d   %d  >> Target Reached", eixo_out.PDOgetStatusWord_TargetReached(), eixo_in.PDOgetStatusWord_TargetReached()); 
		printf("\n   %d   %d  >> Setpoint Ack", eixo_out.PDOgetStatusWord_SetpointAck(), eixo_in.PDOgetStatusWord_SetpointAck()); 

      	//VERIFICA O COMANDO DO OPERADOR
		printf("\n\n\n\nEntre com o valor 0 para voltar ao menu");
		scanf("%d", &COMMAND_KEY2);
	
	}

	//************************************************************************************************************************
	// RESET DAS FALHAS
	//************************************************************************************************************************

	if (COMMAND_KEY == 2) {
		system("cls");
		printf("\n");
		printf("\n");
		printf("********************************************************************************");
		printf("*                SISTEMA DE CONTROLE DE MOTOR - EPOS CANOpen                   *");
		printf("********************************************************************************");
		printf("\n");
		printf("\n");

		//ZERA CONTADOR DE CICLOS
		total_time = 0;

        //EPOS 01
		eixo_out.PDOsetControlWord_FaultReset(true);
		eixo_out.WritePDO01();

		printf("\nResetando as falhas.");
        
		endwait = clock () + 2 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}

		printf("..");

		//EPOS 01
		eixo_out.PDOsetControlWord_FaultReset(false);
		eixo_in.WritePDO01();
               
		printf("..");

		endwait = clock () + 2 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}
	
		printf("..");

        //EPOS 02
        eixo_in.PDOsetControlWord_FaultReset(true);
		eixo_in.WritePDO01();

		printf("..");

		endwait = clock () + 2 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}

		printf("..");

        //EPOS 02
        eixo_in.PDOsetControlWord_FaultReset(false);
		eixo_in.WritePDO01();

		printf("..");

		endwait = clock () + 2 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}
	
		printf("OK");
	
	}


	//************************************************************************************************************************
	// LEITURA DA POSIÇÃO ATUAL
	//************************************************************************************************************************
	
	if (COMMAND_KEY == 3) {
		
		//ZERA CONTADOR DE CICLOS
		total_time = 0;
		
		epos.sync();
		endwait = clock () + 1 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}
		epos.sync();
		
		do{

		total_time += 1;

		endwait = clock () + 1 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}
		epos.sync();
	
		system("cls");
		printf("\n");
		printf("\n");
		printf("********************************************************************************");
		printf("*                SISTEMA DE CONTROLE DE MOTOR - EPOS CANOpen                   *");
		printf("********************************************************************************");
		printf("\n");
		printf("\n");
        printf("\n********************************************************************************");
		eixo_out.ReadPDO01();
        printf("\nPOSICAO CALIBRADA out: %d ppr", (-eixo_out.PDOgetActualPosition())); 
	    eixo_in.ReadPDO01();
        printf("\nPOSICAO CALIBRADA  in: %d ppr", (eixo_in.PDOgetActualPosition()));
		printf("\n\n********************************************************************************");
        printf("\nPOSIÇÃO DO ROBO");

		theta_l = ((-eixo_out.PDOgetActualPosition()-ZERO_01)*2*pi)/encoder_out;
		theta_c = ((eixo_in.PDOgetActualPosition()-ZERO_02)*2*pi)/(encoder_in*N);

		printf("\n");
		printf("\n");
		printf("\n********************************************************************************");
		printf("\n       POSICAO DO EIXO DE SAIDA: %.4f graus", theta_l*(180/pi));
		printf("\n               POSICAO DA COROA: %.4f graus", theta_c*(180/pi));
		printf("\n*********************************************************************************");

		torque_l = ks*(theta_c - theta_l);

		printf("\n");
		printf("\n");
		printf("\n********************************************************************************");
		printf("\n                    Torque: %.4f Nm", torque_l);
		printf("\n*********************************************************************************");


		}while (total_time < 10);

   		total_time = 0;
	}


	//************************************************************************************************************************
	// DEFINE A POSIÇÃO DE ORIGEM
	//************************************************************************************************************************
	
	if (COMMAND_KEY == 4) {
		system("cls");
		printf("\n");
		printf("\n");
		printf("********************************************************************************");
		printf("*                SISTEMA DE CONTROLE DE MOTOR - EPOS CANOpen                   *");
		printf("********************************************************************************");
		printf("\n");
		printf("\n");

		//ZERA CONTADOR DE CICLOS
		total_time = 0;

		epos.sync();

		printf("\nDefinindo Origem...");

		endwait = clock () + 1 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}

		printf("...");
		
		eixo_out.ReadPDO01();
		ZERO_01 = -eixo_out.PDOgetActualPosition();
	
        eixo_in.ReadPDO01();
		ZERO_02 = eixo_in.PDOgetActualPosition();

	}


	//************************************************************************************************************************
	// INTERPRETADOR DE COMANDOS
	//************************************************************************************************************************

	if (COMMAND_KEY == 5) {
		system("cls");
		printf("\n");
		printf("\n");
		printf("********************************************************************************");
		printf("*                SISTEMA DE CONTROLE DE MOTOR - EPOS CANOpen                   *");
		printf("********************************************************************************");
		printf("\n");
		printf("\n");

		//Sincroniza as epos
		epos.sync();

		endwait = clock () + 1 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}

	
		//Habilita o controle de velocidade nos servomotores
		eixo_in.VCS_SetOperationMode(VELOCITY_MODE);
				
		eixo_out.ReadPDO01();
		eixo_in.ReadPDO01();

		//Carrega setpoints
		pFile = fopen ("setpoints.dat","r");
		
		for (i=0;i<48008;i++){

			fscanf(pFile, "%f", &DADOS);
			setpoints[i] = DADOS;
			
		}

    fclose (pFile);

  // #######################################################################################



  // #######################################################################################


		endwait = clock () + 2 * CLOCKS_PER_SEC ;
		while (clock() < endwait) {}

		//Leitura da posição atual
		eixo_out.ReadPDO01();
		theta_l = ((-eixo_out.PDOgetActualPosition()-ZERO_01)*2*pi)/encoder_out;
        eixo_in.ReadPDO01();		
		theta_c = ((eixo_in.PDOgetActualPosition()-ZERO_02)*2*pi)/(encoder_in*N);

		//Habilitação do eixos
		Habilita_Eixo(2);

		//Leitura da posição atual
		eixo_out.ReadPDO01();
		theta_l = ((-eixo_out.PDOgetActualPosition()-ZERO_01)*2*pi)/encoder_out;
        eixo_in.ReadPDO01();		
		theta_c = ((eixo_in.PDOgetActualPosition()-ZERO_02)*2*pi)/(encoder_in*N);

		system("cls");
		printf("\n");
		printf("\n");
		printf("********************************************************************************");
		printf("*                SISTEMA DE CONTROLE DE MOTOR - EPOS CANOpen                   *");
		printf("********************************************************************************");
		printf("\n");
		printf("\n");
		printf("\n********************************************************************************");
		printf("POSICÃO INICIAL DO ROBO:");
		printf("\n          POSICAO DO EIXO DE SAIDA [graus]: %.4f", theta_l*(180/pi));
		printf("\n                  POSICAO DA COROA [graus]: %.4f", theta_c*(180/pi));
		printf("\n********************************************************************************");

		printf("\n\n\nCONTROLE INICIALIZADO:");

		
		printf("\n\n\nDEFINA O TEMPO DE EXECUCAO (s): ");
		scanf("%d", &exec_time);

		printf("\n\n\nCONTROLE INICIALIZADO:");
		
		// POSICIONA
		POSICIONA_J(exec_time*200);
        
        //Zera o comando do motor
		eixo_in.PDOsetVelocitySetpoint(0);
		eixo_in.WritePDO02();

		//Desabilita o eixo
		Desabilita_Eixo(0);

	}

}while (COMMAND_KEY != 0);

	//FINALIZA A COMUNICAÇÃO COM AS EPOS
	epos.StopPDOS(1);

	endwait = clock () + 2 * CLOCKS_PER_SEC ;
	while (clock() < endwait) {}

    //FIM DO PROGRAMA
    printf("\nFIM DE PROGRAMA");
	return 0;

}


