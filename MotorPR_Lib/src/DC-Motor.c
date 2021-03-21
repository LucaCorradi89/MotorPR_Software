/**********************************************************************
* $Id$		DC-Motor.c				2013-08-20
*//**
* @file		DC-Motor.c
* @brief	Contiene le funzioni per la gestione del motore DC
* @version	0.1
* @date		20. Aug. 2013
* @author	Corradi Luca
*
*
***********************************************************************/

/* Gruppo --------------------------------------------------------------------- */
/** @addtogroup DC-Motor
 * @{
 */

/* Includes ------------------------------------------------------------------- */
#include <DC-Motor.h>

/* Private Types -------------------------------------------------------------- */
/** @defgroup DC-Motor_Private_Types DC-Motor Private Types
 * @{
 */
/**
 * @brief		Struttura che contiene le informazioni per il movimento.
 */

/**
 * @brief		Struttura che contiene le posisioni sugli assi.
 */

//Struttura

/**
 * @}
 */

/* Private Variables ---------------------------------------------------------- */
/** @defgroup DC-Motor_Private_Variables DC-Motor Private Variables
 * @{
 */
/**
 * @brief		Frequenza di clock in ingresso alla periferica MCPWM.
 */

//Variabili private


/**
 * @}
 */
/* Private Functions ---------------------------------------------------------- */
/** @defgroup DC-Motor_Private_Functions DC-Motor Private Functions
 * @{
 */

void InitHwMotorCntrl(void);


/*********************************************************************//**
 * @brief 		Si impostano i pin e i vari pull-up dove necessari
 * 				per il controllo del ponte H e quindi del motore
 * @details		I pin sono definiti nel file SystemConfig.h
 **********************************************************************/
void InitHwMotorCntrl(void){

	/* Creo la struttura PinCfg*/
	PINSEL_CFG_Type PinCfg;

	/* MOTOR-PWM */
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;

	PinCfg.Funcnum = FUNCNUM_HBRIDGE_PWM;
	PinCfg.Portnum = PORTNUM_HBRIDGE_PWM;
	PinCfg.Pinnum = PINNUM_HBRIDGE_PWM;
	PINSEL_ConfigPin(&PinCfg);

	/*INA,INB as GPIO*/
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;

	PinCfg.Funcnum = FUNCNUM_HBRIDGE_INA;
	PinCfg.Portnum = PORTNUM_HBRIDGE_INA;
	PinCfg.Pinnum = PINNUM_HBRIDGE_INA;
	PINSEL_ConfigPin(&PinCfg);

	PinCfg.Funcnum = FUNCNUM_HBRIDGE_INB;
	PinCfg.Portnum = PORTNUM_HBRIDGE_INB;
	PinCfg.Pinnum = PINNUM_HBRIDGE_INB;
	PINSEL_ConfigPin(&PinCfg);

	/*Imposto come uscita i pin INA e INB*/
	GPIO_SetDir(PORTNUM_HBRIDGE_INA,1<<PINNUM_HBRIDGE_INA, 0x01);
	GPIO_SetDir(PORTNUM_HBRIDGE_INB,1<<PINNUM_HBRIDGE_INB, 0x01);

	/* DIAGA e DIAGB */
	PinCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;

	PinCfg.Funcnum = FUNCNUM_HBRIDGE_DIAGA;
	PinCfg.Portnum = PORTNUM_HBRIDGE_DIAGA;
	PinCfg.Pinnum = PINNUM_HBRIDGE_DIAGA;
	PINSEL_ConfigPin(&PinCfg);

	PinCfg.Funcnum = FUNCNUM_HBRIDGE_DIAGB;
	PinCfg.Portnum = PORTNUM_HBRIDGE_DIAGB;
	PinCfg.Pinnum = PINNUM_HBRIDGE_DIAGB;
	PINSEL_ConfigPin(&PinCfg);

}

/**
 * @}
 */

/* Public Functions ----------------------------------------------------------- */
/** @addtogroup DC-Motor_Public_Functions
 * @{
 */

/*********************************************************************//**
 * @brief 		Inizializza le periferiche per il controllo dei
 * 				sensori.
 * @details		Inizializza le periferiche e i pin per il controllo
 * 				di endstop ed encoder.
 **********************************************************************/
void InitMotor(void){
	/* Inizializzo i pin e le periferiche hardware */
	InitHwMotorCntrl();

	/* Mi creo una struttura contenete le impostazioni del canale mcpwm che voglio usare*/
	MCPWM_CHANNEL_CFG_Type MC_CH_Struct;

	/* Disabilito interrupt*/
	NVIC_DisableIRQ(MCPWM_IRQn);

	/* Chiamo l'init dalla libreria */
	MCPWM_Init(LPC_MCPWM);

	/* Imposto la struttura di configurazione */
	MC_CH_Struct.channelDeadtimeEnable =  DISABLE;
	MC_CH_Struct.channelPolarity = MCPWM_CHANNEL_PASSIVE_HI;
	MC_CH_Struct.channelType = MCPWM_CHANNEL_EDGE_MODE;
	MC_CH_Struct.channelUpdateEnable = ENABLE;
	MC_CH_Struct.channelTimercounterValue = 0;
	MC_CH_Struct.channelPeriodValue = PWM_PERIOD;
	MC_CH_Struct.channelPulsewidthValue = 0;

	/* Imposto il canale */
	MCPWM_ConfigChannel(LPC_MCPWM, 1, &MC_CH_Struct);

	/* Freno il motore */
	MotorModeSet(MOTOR_BRAKE);

	/* Avvio il pwmm canale1 */
	MCPWM_Start(LPC_MCPWM,DISABLE,ENABLE,DISABLE);

}

/*********************************************************************//**
 * @brief 		Imposta il dutycycle del MCPWM che controlla il motore
 * @details		Imposta il dutycycle del MCPWM che controlla il motore,
 * 				i valori vanno da 0 a 100.
 **********************************************************************/
void MotorSetDuty(LPC_MCPWM_TypeDef *MCPWMx, unsigned char channelNum, uint32_t DutyCycle){

	unsigned int Period;
	/* ToDo: Sistemare la matematica in modo elegante   */
	Period = ((uint32_t)PWM_PERIOD*DutyCycle)/(100);

	if (channelNum == 0) {
		MCPWMx->MCPW0 = Period;
	} else if (channelNum == 1) {
		MCPWMx->MCPW1 = Period;
	} else if (channelNum == 2) {
		MCPWMx->MCPW2 = Period;
	}

}





/*********************************************************************//**
 * @brief 		Questa funzione decide il verso di rotazione del motore o
 * 				se frenarlo
 * @details		Con questa funzione si fa ruotare nei due versi il motore o
 * 				si frena.
 * @param[in]	Mode
 * 					->MOTOR_CW
 * 					->MOTOR_CCW
 * 					->MOTOR_BRAKE
 **********************************************************************/
void MotorModeSet(unsigned char Mode){

	/* Il motore deve girare nel verso orario*/
	if(Mode == MOTOR_CW){
		GPIO_SetValue(PORTNUM_HBRIDGE_INA, 1<<PINNUM_HBRIDGE_INA);
		GPIO_ClearValue(PORTNUM_HBRIDGE_INB, 1<<PINNUM_HBRIDGE_INB);
	}
	/* Il motore deve girare nel verso anti-orario*/
	if(Mode == MOTOR_CCW){
		GPIO_ClearValue(PORTNUM_HBRIDGE_INA, 1<<PINNUM_HBRIDGE_INA);
		GPIO_SetValue(PORTNUM_HBRIDGE_INB, 1<<PINNUM_HBRIDGE_INB);
	}
	/* Il motore deve frenare*/
	if(Mode == MOTOR_BRAKE){
		GPIO_ClearValue(PORTNUM_HBRIDGE_INA, 1<<PINNUM_HBRIDGE_INA);
		GPIO_ClearValue(PORTNUM_HBRIDGE_INB, 1<<PINNUM_HBRIDGE_INB);
	}
}



/**
 * @}
 */

/**
 * @}
 */

