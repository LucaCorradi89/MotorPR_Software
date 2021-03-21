/**********************************************************************
* $Id$		main.c			 30-07-2013
*//**
* @file		main.c
* @brief	This file is the main.c of MotorPR Board.
* @version	0.1
* @date		30. July. 2013
* @author	Luca Corradi
***********************************************************************/

//todo: Sistemare flag in maniera corretta.


#include "LPC17xx.h"
#include <cr_section_macros.h>
#include <NXP/crp.h>
//Includo le librerie che mi servono.

#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"
#include "SystemConfig.h"
#include "UserIO.h"
#include "ConfigManager.h"
#include "Networking.h"
#include "Sensor.h"
#include "DC-Motor.h"
#include <pid.h>


// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;

/* Variabili di uso generale */
int32_t Duty = 0, maxSpeed = 0;
uint16_t tmp = 0,tm=0;
/* variabile per cicli for */
uint32_t n=0;
/* Soglia sotto la quale non agisce il pid */
uint32_t Duty_th = 5;
uint32_t comunicationFlags __attribute__((section(".bss.$RAM2")));
uint32_t tmpVal = 0;
uint8_t EmergenzaMIN=0,EmergenzaMAX=0;
/* Gestione antirimbalzo software */
uint32_t tmrCnt=0;
uint8_t tmrCntEn=0;


/*Strutture*/
SWITCH_SETTING_Type DipConfig;
ENC_STATUS_Type EncSt;
pid_f_t pid_Struct;
pid_f_t pid_p_Struct;
RS422_RX_Type RS422_Struct;
RS422_RX_Type RS485_Struct;
CommandMngBuffer_Type DecodeStrCmd, DecodeStrCmd1,SendStrCmd;
GlobalSettingValue_Type GlobalValueSetting;

/*Buffer circolare RX seriale e puntatori*/
uint8_t SerialRxBuffer[64], SerialRxBuffer2[64];
uint8_t *PntRS422, *PntRS485;
uint8_t last=0, last2=0;

/* Interrupt service routine */
void TIMER0_IRQHandler(void);
void QEI_IRQHandler(void);
void EINT3_IRQHandler(void);
void UART1_IRQHandler(void);
void UART3_IRQHandler(void);

/* Generic Function */
void PID_TimerInit(void);
void LoadSystemParam();

/* Task gestione PID posizione/velocità */
void TIMER0_IRQHandler(void)
{
	if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT)== SET)
	{
		/* In base alla modalità di funzionamento impostata eseguo il calcolo sulla posizione/velocità o sulla sola velocità */
		if(GlobalValueSetting.MODPID == MOD_PID_SPEEDPOS){
			/* Posizione e Velocità */
			ReadSpeed(&EncSt,GlobalValueSetting.ENC_RES);
			ReadPosition(&EncSt);
			EncSt.SetSpeed = (int32_t)(pid_update_f( EncSt.SetPosition, EncSt.ActPosition, &pid_p_Struct ));
			Duty = (int32_t)(pid_update_f( EncSt.SetSpeed, EncSt.ActSpeed, &pid_Struct ));
		}else{
			/* Velocità */
			ReadSpeed(&EncSt,GlobalValueSetting.ENC_RES);
			Duty = (int32_t)(pid_update_f( EncSt.SetSpeed, EncSt.ActSpeed, &pid_Struct ));
		}

		if (Duty == 0) {
			MotorModeSet(MOTOR_BRAKE);
		} else if (Duty < 0) {
			MotorModeSet(MOTOR_CCW);
			Duty = -Duty;
		} else {
			MotorModeSet(MOTOR_CW);
		}

		if ((Duty < Duty_th) && (Duty != 0)) {
			Duty = Duty_th;
		}

		MotorSetDuty(LPC_MCPWM, 1 , Duty);

	}
	TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
}

/* Task gestione buffer gira ogni 20ms
 * ToDo: Fare in modo dinamico il dt
 *  */
void TIMER1_IRQHandler(void)
{
	if (TIM_GetIntStatus(LPC_TIM1, TIM_MR1_INT)== SET){

		tmrCnt++;

		/* Se sono nella modalità master spedisco i valori e carico il nuovo valore in RAM */
		if(DipConfig.MasterSlave == MOD_MASTER){

			SendStrCmd.Function = FUN_BUFWRT;
			SendStrCmd.Register = 0x0000;


			/* Mando il valore sulla RS485... */
			if((GlobalValueSetting.BUFEN & 0b010) && (GlobalValueSetting.BUFFREE[1] < BUFSIZE)){
				SendStrCmd.Address = GlobalValueSetting.BUFADD1;
				SendStrCmd.Length = 0x04;
				tmpVal = (uint32_t)(GlobalValueSetting.BUFVAL1[GlobalValueSetting.BUFOUTPNT[1]]);
				SendStrCmd.Message[0] = (uint8_t)(tmpVal >> 24);
				SendStrCmd.Message[1] = (uint8_t)(tmpVal >> 16);
				SendStrCmd.Message[2] = (uint8_t)(tmpVal >> 8);
				SendStrCmd.Message[3] = (uint8_t)(tmpVal & 0xFF);
				SendStrCmd.Length = 0x04;

				RS485_SendCommand(&SendStrCmd);

				++GlobalValueSetting.BUFOUTPNT[1];
				if(GlobalValueSetting.BUFOUTPNT[1] == BUFSIZE) GlobalValueSetting.BUFOUTPNT[1]=0;
				++GlobalValueSetting.BUFFREE[1];
			}
			for(tm=0;tm<600;tm++){
				LedOn(LED_BUS);
			}
			for(tm=0;tm<600;tm++){
				LedOff(LED_BUS);
			}
			/* Mando il valore sulla RS485... */
			if((GlobalValueSetting.BUFEN & 0b100) && GlobalValueSetting.BUFFREE[2] < BUFSIZE){
				SendStrCmd.Address = GlobalValueSetting.BUFADD2;
				SendStrCmd.Length = 0x04;
				tmpVal = (uint32_t)(GlobalValueSetting.BUFVAL2[GlobalValueSetting.BUFOUTPNT[2]]);
				SendStrCmd.Message[0] = (uint8_t)(tmpVal >> 24);
				SendStrCmd.Message[1] = (uint8_t)(tmpVal >> 16);
				SendStrCmd.Message[2] = (uint8_t)(tmpVal >> 8);
				SendStrCmd.Message[3] = (uint8_t)(tmpVal & 0xFF);
				SendStrCmd.Length = 0x04;

				RS485_SendCommand(&SendStrCmd);

				++GlobalValueSetting.BUFOUTPNT[2];
				if(GlobalValueSetting.BUFOUTPNT[2] == BUFSIZE) GlobalValueSetting.BUFOUTPNT[2]=0;
				++GlobalValueSetting.BUFFREE[2];
			}
			/* Mando il valore sulla RS485... */
			if((GlobalValueSetting.BUFEN & 0b1000) && GlobalValueSetting.BUFFREE[3] < BUFSIZE){
				SendStrCmd.Address = GlobalValueSetting.BUFADD3;
				SendStrCmd.Length = 0x04;
				tmpVal = (uint32_t)(GlobalValueSetting.BUFVAL3[GlobalValueSetting.BUFOUTPNT[3]]);
				SendStrCmd.Message[0] = (uint8_t)(tmpVal >> 24);
				SendStrCmd.Message[1] = (uint8_t)(tmpVal >> 16);
				SendStrCmd.Message[2] = (uint8_t)(tmpVal >> 8);
				SendStrCmd.Message[3] = (uint8_t)(tmpVal & 0xFF);
				SendStrCmd.Length = 0x04;

				RS485_SendCommand(&SendStrCmd);

				++GlobalValueSetting.BUFOUTPNT[3];
				if(GlobalValueSetting.BUFOUTPNT[3] == BUFSIZE) GlobalValueSetting.BUFOUTPNT[3]=0;
				++GlobalValueSetting.BUFFREE[3];
			}
			/* ToDo:Invio byte per sincronia */
			/* Controllo che sia tutto ok nella risposta delle schede slave */

		}
		/* Carico in ram il valore per la mia scheda solo se non ci sono condizioni di errore e gestisco eventuale home*/

		if(GlobalValueSetting.BUFFREE[0] < BUFSIZE && EmergenzaMAX==0 && (EmergenzaMIN==0 || (EmergenzaMIN==1 && (int32_t)(GlobalValueSetting.BUFVAL0[GlobalValueSetting.BUFOUTPNT[0]])>=0))){
			EncSt.SetPosition = (int32_t)(GlobalValueSetting.BUFVAL0[GlobalValueSetting.BUFOUTPNT[0]]);
			++GlobalValueSetting.BUFOUTPNT[0];
			if(GlobalValueSetting.BUFOUTPNT[0] == BUFSIZE) GlobalValueSetting.BUFOUTPNT[0]=0;
			++GlobalValueSetting.BUFFREE[0];
			/* Gestione antirimbalzo se l'asse è in posizione di home*/
			if(GlobalValueSetting.ATHOME==0x01){
				EmergenzaMIN=0;
				GlobalValueSetting.ATHOME = 0x00;
				tmrCnt=0;
				tmrCntEn=1;
			}

		}

	}
	TIM_ClearIntPending(LPC_TIM1, TIM_MR1_INT);
}

/* Gestione Encoder */
void QEI_IRQHandler(void){
	QEI_Interrupt();
}

/* Gestione Finecorsa */
void EINT3_IRQHandler(void){
	/* Gestione anti rimbalzo todo: gestione flag in modo elegante*/
	if(tmrCntEn==0 || tmrCnt>=9){
		if(GPIO_GetIntStatus(PORTNUM_ENDSTOP1,PINNUM_ENDSTOP1, GlobalValueSetting.STAT_ENDSTOP1)){
			/* Resetto posizione attuale a 0 e blocco il motore */
			TIM_Cmd(LPC_TIM1,DISABLE);
			EmergenzaMIN = 1;
			GlobalValueSetting.ATHOME=0x01;
			QEI_Reset(LPC_QEI, QEI_RESET_POS);
			EncSt.SetPosition=0;
			MotorModeSet(MOTOR_BRAKE);
			LedOff(LED_BUS);
			TIM_Cmd(LPC_TIM1,ENABLE);
			GPIO_ClearInt(PORTNUM_ENDSTOP1,1<<PINNUM_ENDSTOP1);
		}
		if(GPIO_GetIntStatus(PORTNUM_ENDSTOP2,PINNUM_ENDSTOP2, GlobalValueSetting.STAT_ENDSTOP2)){
			/* blocco il motore emergenza! */
			TIM_Cmd(LPC_TIM1,DISABLE);
			EmergenzaMAX = 1;
			MotorModeSet(MOTOR_BRAKE);
			EncSt.SetPosition=EncSt.ActPosition;
			GPIO_ClearInt(PORTNUM_ENDSTOP2,1<<PINNUM_ENDSTOP2);
		}
		tmrCnt=0;
		tmrCntEn=1;
	}else{
		tmrCntEn=0;
		GPIO_ClearInt(PORTNUM_ENDSTOP1,1<<PINNUM_ENDSTOP1);
		GPIO_ClearInt(PORTNUM_ENDSTOP2,1<<PINNUM_ENDSTOP2);
	}
}


/* Gestione rete */
void UART1_IRQHandler(void){

	last = RS422_Interrupt(&RS422_Struct);

	//Controllo se ho dei nuovi dati da elaborare dalla seriale
	for(n=0;n<RS422_Struct.Flag;n++){
		//Inserisco nell'array circolare
		*PntRS422 = RS422_Struct.RxData[n];
		PntRS422++;
		//Se arrivo in fondo al buffer resetto e vado a 0
		if(PntRS422>=(SerialRxBuffer+64)){
			//Errore, buffer pieno, non è possibile scriverci
			LedOn(LED_ERROR);
		}
	}
	/* Sequenza comandi finita, procedo
	 * todo: last metterlo come flagsComunications
	 * */
	if(last==1){

		PntRS422=SerialRxBuffer;
		/* Se mi ritorna 1 vuol dire che il CRC è valido e posso utilizzare i dati */
		if(Net_DecodeCommand(PntRS422, &DecodeStrCmd)){
			LedOff(LED_ERROR);
			if(DecodeStrCmd.Address == GlobalValueSetting.BUFADD0 ){
				/* Eseguo il comando */
				CommandExec(&DecodeStrCmd, &GlobalValueSetting,(LPC_UART_TypeDef *)LPC_UART1);
			}else{
				/* Giro la richiesta sul bus RS485 */
				RS485_SendCommand(&DecodeStrCmd);

			}

		}else{
			/* Errore CRC */
			LedOn(LED_ERROR);

		}
		last=0;
	}
}


void UART3_IRQHandler(void){

	last2 = RS485_Interrupt(&RS485_Struct);

	//Controllo se ho dei nuovi dati da elaborare dalla seriale
	for(n=0;n<RS485_Struct.Flag;n++){
		//Inserisco nell'array circolare
		*PntRS485 = RS485_Struct.RxData[n];
		PntRS485++;
		//Se arrivo in fondo al buffer resetto e vado a 0
		if(PntRS485>=(SerialRxBuffer2+64)){
			//Errore, buffer pieno, non è possibile scriverci
			LedOn(LED_ERROR);
		}
	}
	/* Sequenza comandi finita, procedo
	 * todo: last metterlo come flagsComunications
	 * */
	if(last2==1){

		PntRS485=SerialRxBuffer2;
		/* Se mi ritorna 1 vuol dire che il CRC è valido e posso utilizzare i dati */
		if(Net_DecodeCommand(PntRS485, &DecodeStrCmd1)){
			LedOff(LED_ERROR);
			if(DecodeStrCmd1.Address == GlobalValueSetting.BUFADD0 ){
				/* Eseguo il comando */
				CommandExec(&DecodeStrCmd1, &GlobalValueSetting,(LPC_UART_TypeDef *)LPC_UART3);
			}
		}else{
			/* Errore CRC */
			LedOn(LED_ERROR);

		}
		last=0;
	}

}

/* Timer gestione buffer seriali */
void BUFFER_TimerInit(void){

	TIM_TIMERCFG_Type TIM_ConfigStruct;
	TIM_MATCHCFG_Type TIM_MatchConfigStruct ;

	// Initialize timer 1, prescale count time of 100uS
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 100;

	// use channel 1, MR1
	TIM_MatchConfigStruct.MatchChannel = 1;
	// Enable interrupt when MR1 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR1: TIMER will reset if MR1 matches it
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Stop on MR1 if MR1 matches it
	TIM_MatchConfigStruct.StopOnMatch  = FALSE;
	//Toggle MR1.0 pin if MR0 matches it
	TIM_MatchConfigStruct.ExtMatchOutputType =TIM_EXTMATCH_NOTHING;
	// Set Match value, count value of 500 (200 * 100uS = 2000us = 20ms)
	TIM_MatchConfigStruct.MatchValue   = 200;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM1, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM1,&TIM_MatchConfigStruct);

	/* preemption = 2, sub-priority = 1 */
	NVIC_SetPriority(TIMER1_IRQn, ((0x01<<3)|0x03));
	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(TIMER1_IRQn);
	// To start timer 0
	TIM_Cmd(LPC_TIM1,ENABLE);
}

/* Inizializzazione Timer che gestisce il pid */
void PID_TimerInit(void){

	TIM_TIMERCFG_Type TIM_ConfigStruct;
	TIM_MATCHCFG_Type TIM_MatchConfigStruct ;

	// Initialize timer 0, prescale count time of 100uS
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 100;

	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 0;
	// Enable interrupt when MR0 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR0: TIMER will reset if MR0 matches it
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Stop on MR0 if MR0 matches it
	TIM_MatchConfigStruct.StopOnMatch  = FALSE;
	//Toggle MR0.0 pin if MR0 matches it
	TIM_MatchConfigStruct.ExtMatchOutputType =TIM_EXTMATCH_NOTHING;
	// Set Match value, count value of 500 (20 * 100uS = 2000us = 2ms)
	TIM_MatchConfigStruct.MatchValue   = 20;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM0, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM0,&TIM_MatchConfigStruct);

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER0_IRQn, ((0x01<<3)|0x01));
	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(TIMER0_IRQn);
	// To start timer 0
	TIM_Cmd(LPC_TIM0,ENABLE);
}


void LoadSystemParam(){

	/* Coeff. PID */
	/* Setto i parametri del PID di velocità */
	pid_Struct.kd = (float)(EEPROM_ReadRegister32(REG_PID_V_KD)/1000.0); //0.01
	pid_Struct.ki = (float)(EEPROM_ReadRegister32(REG_PID_V_KI)/1000.0); //0.01;
	pid_Struct.kp = (float)((EEPROM_ReadRegister32(REG_PID_V_KP))/1000.0); //0.13;

	/* Setto i parametri del PID di posizione */
	pid_p_Struct.kd = (float)(EEPROM_ReadRegister32(REG_PID_P_KD)/1000.0); //0.5;
	pid_p_Struct.ki = (float)(EEPROM_ReadRegister32(REG_PID_P_KI)/1000.0); //0;
	pid_p_Struct.kp = (float)(EEPROM_ReadRegister32(REG_PID_P_KP)/1000.0); //5;

	GlobalValueSetting.PID_V_KD = pid_Struct.kd;
	GlobalValueSetting.PID_V_KI = pid_Struct.ki;
	GlobalValueSetting.PID_V_KP = pid_Struct.kp;

	GlobalValueSetting.PID_P_KD = pid_p_Struct.kd;
	GlobalValueSetting.PID_P_KI = pid_p_Struct.ki;
	GlobalValueSetting.PID_P_KP = pid_p_Struct.kp;

	/* Velocità max (utile nella modalità di posizione*/
	GlobalValueSetting.MAXSPEED = EEPROM_ReadRegister16(REG_MAXSPEED);

	/* Carico la risoluzione dell'encoder */
	GlobalValueSetting.ENC_RES = EEPROM_ReadRegister16(REG_ENC_RES);

	/* Carico il dt di aggiornamento del buffer */
	GlobalValueSetting.BUFF_UPDATE = EEPROM_ReadRegister16(REG_BUFF_UPDATE);

	/* Modalità di controllo PID */
	GlobalValueSetting.MODPID =  EEPROM_ReadRegister8(REG_MODPID);

	/* Leggo l'indirizzo della mia scheda */
	GlobalValueSetting.BUFADD0 = DipConfig.BusAdd;

	/* Leggo la modalità master/slave */
	GlobalValueSetting.MODMASL = DipConfig.MasterSlave;

	//EEPROM_SetRegister8(REG_STAT_ENDSTOP1,0);
	//EEPROM_SetRegister8(REG_STAT_ENDSTOP2,0);

	/* Finecorsa tipo di contatto */
	GlobalValueSetting.STAT_ENDSTOP1 =  EEPROM_ReadRegister8(REG_STAT_ENDSTOP1);
	GlobalValueSetting.STAT_ENDSTOP2 =  EEPROM_ReadRegister8(REG_STAT_ENDSTOP2);


	/* Resetto i puntatori e lo spazio libero nei buffer */
	for(tmp=0; tmp<8; tmp++){
		GlobalValueSetting.BUFINPNT[tmp] = 0;
		GlobalValueSetting.BUFOUTPNT[tmp] = 0;
		GlobalValueSetting.BUFFREE[tmp] = BUFSIZE;
	}

}



int main(void) {

	/*Avvio il sistema*/
	SystemInit();

	/* Inizializzo le interfacce utente */
	InitUserIO();

	/* Inizializzo i sensori finecorsa e encoder */
	InitSensor();

	/* Inizializzo il motore, ponte H &Co. */
	InitMotor();

	/* Inizializzo la parte di gestione della configurazione sulla eeprom */
	EEPROM_Init();

	/* Inizializzo due puntatori per il buffer circolare RX-RS422 */
	PntRS422 = SerialRxBuffer;
	PntRS485 = SerialRxBuffer2;


	/* Leggo le impostazioni utente da dipSwitch */
	LoadSetting(&DipConfig);

	/* Carico le impostazioni */
	LoadSystemParam();


	/* Abilito interrupt per finecorsa */
	GPIO_IntCmd(PORTNUM_ENDSTOP1, 1<<PINNUM_ENDSTOP1, GlobalValueSetting.STAT_ENDSTOP1);
	GPIO_IntCmd(PORTNUM_ENDSTOP2, 1<<PINNUM_ENDSTOP2, GlobalValueSetting.STAT_ENDSTOP2);

	/* preemption = 4, sub-priority = 1 */
	NVIC_SetPriority(EINT3_IRQn, ((0x01<<3)|0x04));
	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(EINT3_IRQn);


	/* In modalità master abilito entrambi i bus, RS485 e RS422, in modalità slave, solo il bus 485 */
	if(GlobalValueSetting.MODMASL == MOD_MASTER){
		/*Inizializzo l'interfaccia master per la comunicazione*/
		RS422_Init();
		/*Inizializzo l'interfaccia secondaria per la comunicazione con gli slave*/
		RS485_Init();

	}else{
		/*Inizializzo l'interfaccia slave per la comunicazione*/
		RS485_Init();

	}

	/*In base alla modalità impostata inizializzo il pid in modalità velocità o posizione/velocità*/
	if(GlobalValueSetting.MODPID == MOD_PID_SPEEDPOS){
		/* Modalità pid di posizione */

		/* Imposto minimo e massimo DutyCycle */
		pid_init_f(&pid_Struct,-100, 100);

		/* Imposto minimo e massimo Velocità */
		pid_init_f(&pid_p_Struct,(int16_t)(-GlobalValueSetting.MAXSPEED), GlobalValueSetting.MAXSPEED);

		//Velocità a posizione iniziali = 0
		EncSt.SetPosition = 0;
		EncSt.SetSpeed = 0;

	}else{
		/* Modalità pid di velocità */

		/* Velocità iniziali = 0 */
		EncSt.SetSpeed = 0;

		/* Imposto minimo e massimo sulla velocità (dutycycle) */
		pid_init_f(&pid_Struct,-100, 100);
	}

	GlobalValueSetting.ATHOME = 0x00;
	EmergenzaMIN=0;
	EmergenzaMAX=0;

	if(((GPIO_ReadValue(PORTNUM_ENDSTOP1) & (1<<PINNUM_ENDSTOP1))>>PINNUM_ENDSTOP1)!=GlobalValueSetting.STAT_ENDSTOP1){
		GlobalValueSetting.ATHOME = 0x01;
		EmergenzaMIN = 1;
	}else if(((GPIO_ReadValue(PORTNUM_ENDSTOP2) & (1<<PINNUM_ENDSTOP2))>>PINNUM_ENDSTOP2)!=GlobalValueSetting.STAT_ENDSTOP2){
		EmergenzaMAX = 1;
	}

	/* Inizializzo il timer principale */
	PID_TimerInit();

	/* Inizializzo il timer di gestione del buffer */
	BUFFER_TimerInit();

	/* Attivo la ricezione*/
	GPIO_ClearValue(PORTNUM_RS485_RE, 1<<PINNUM_RS485_RE);
	GPIO_ClearValue(PORTNUM_RS485_DE, 1<<PINNUM_RS485_DE);

	/* Fine sequenza init */
	while(1) {
		/*ToDo: Valutare eventualmente flag o altro metodo*/
		if(GlobalValueSetting.ATHOME==0x0F){
			if(((GPIO_ReadValue(PORTNUM_ENDSTOP1) & (1<<PINNUM_ENDSTOP1))>>PINNUM_ENDSTOP1)==GlobalValueSetting.STAT_ENDSTOP1){
				EncSt.SetPosition=(-30000);
				GlobalValueSetting.ATHOME=0x0E;
				LedOn(LED_BUS);
			}else{
				GlobalValueSetting.ATHOME=0x01;
				EmergenzaMIN=0x01;
			}
		}
		/* Non faccio nulla, attendo che qualche interrupt mi svegli!!!! */
		for(tmp=0;tmp<60000;tmp++){
			LedOn(LED_STATUS);
		}
		for(tmp=0;tmp<60000;tmp++){
			LedOff(LED_STATUS);
		}
	}
	return 0 ;
}
