/**********************************************************************
* $Id$		Sensor.c				2013-08-18
*//**
* @file		Sensor.c
* @brief	Contiene le funzioni per la gestione dei sensori,
* 			in particolare encoder e fine corsa
* @version	0.1
* @date		18. Aug. 2013
* @author	Corradi Luca
*
*
***********************************************************************/

/* Gruppo --------------------------------------------------------------------- */
/** @addtogroup Sensor
 * @{
 */

/* Includes ------------------------------------------------------------------- */
#include <Sensor.h>

/* Private Types -------------------------------------------------------------- */
/** @defgroup Sensor_Private_Types Stepper Private Types
 * @{
 */
/**
 * @brief		Struttura che contiene le informazioni sulla posizione e velocità.
 */



/**
 * @}
 */

/* Private Variables ---------------------------------------------------------- */
/** @defgroup Sensor_Private_Variables Stepper Private Variables
 * @{
 */

/**
 * @brief		Velocità del motore, la variabile viene aggiornata dalla routine di interrupt del QEI quando c'è un overflow del contatore velocità.
 */
int Speed = 0;


/**
 * @}
 */
/* Private Functions ---------------------------------------------------------- */
/** @defgroup Sensor_Private_Functions Stepper Private Functions
 * @{
 */
void InitHwSensorCntrl(void);


/*********************************************************************//**
 * @brief 		Si impostano i pin e i vari pull-up dove necessari
 * 				per il controllo dell'encoder e dei finecorsa.
 * @details		I pin sono definiti nel file SystemConfig.h
 **********************************************************************/
void InitHwSensorCntrl(void){

	/* Creo la struttura PinCfg*/
	PINSEL_CFG_Type PinCfg;

	/*Modalità normale*/
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;

	/*QEI PHA*/
	PinCfg.Funcnum = FUNCNUM_ENCODER_PHA;
	PinCfg.Portnum = PORTNUM_ENCODER_PHA;
	PinCfg.Pinnum = PINNUM_ENCODER_PHA;
	PINSEL_ConfigPin(&PinCfg);

	/*QEI PHB*/
	PinCfg.Funcnum = FUNCNUM_ENCODER_PHB;
	PinCfg.Portnum = PORTNUM_ENCODER_PHB;
	PinCfg.Pinnum = PINNUM_ENCODER_PHB;
	PINSEL_ConfigPin(&PinCfg);

	/*QEI INDEX*/
	PinCfg.Funcnum = FUNCNUM_ENCODER_INDEX;
	PinCfg.Portnum = PORTNUM_ENCODER_INDEX;
	PinCfg.Pinnum = PINNUM_ENCODER_INDEX;
	PINSEL_ConfigPin(&PinCfg);

	/*EndStop1*/
	PinCfg.Funcnum = 0x00;
	PinCfg.Portnum = PORTNUM_ENDSTOP1;
	PinCfg.Pinnum = PINNUM_ENDSTOP1;
	PINSEL_ConfigPin(&PinCfg);

	/*EndStop2*/
	PinCfg.Funcnum = 0x00;
	PinCfg.Portnum = PORTNUM_ENDSTOP2;
	PinCfg.Pinnum = PINNUM_ENDSTOP2;
	PINSEL_ConfigPin(&PinCfg);
}

/**
 * @}
 */

/* Public Functions ----------------------------------------------------------- */
/** @addtogroup Sensor_Public_Functions
 * @{
 */
/*********************************************************************//**
 * @brief 		Inizializza le periferiche per il controllo dei
 * 				sensori.
 * @details		Inizializza le periferiche e i pin per il controllo
 * 				di endstop ed encoder.
 **********************************************************************/
void InitSensor(void){

	InitHwSensorCntrl();

	QEI_CFG_Type QEI_Struct;
	QEI_RELOADCFG_Type QEI_ReloadConfig;

	/*Configuro la struttura QEI_Struct*/
	QEI_Struct.CaptureMode = QEI_CAPMODE_2X;
	QEI_Struct.DirectionInvert = QEI_DIRINV_NONE;
	QEI_Struct.InvertIndex = QEI_INVINX_NONE;
	QEI_Struct.SignalMode = QEI_SIGNALMODE_QUAD;

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(QEI_IRQn, ((0x01<<3)|0x01));
	/* Enable interrupt for QEI  */
	NVIC_EnableIRQ(QEI_IRQn);

	QEI_Init(LPC_QEI, &QEI_Struct);
	QEI_SetMaxPosition(LPC_QEI,0xFFFFFFFF);

	/* Enable interrupt for velocity Timer overflow for capture velocity into Acc */
	QEI_IntCmd(LPC_QEI, QEI_INTFLAG_TIM_Int, ENABLE);

	/* Enable interrupt for direction change */
	//QEI_IntCmd(LPC_QEI, QEI_INTFLAG_DIR_Int, ENABLE);



	/*Configuro la struttura ReloadConfig*/
	QEI_ReloadConfig.ReloadOption = QEI_TIMERRELOAD_USVAL;
	QEI_ReloadConfig.ReloadValue = 2500UL;
	QEI_SetTimerReload(LPC_QEI, &QEI_ReloadConfig);

}

/*********************************************************************//**
 * @brief		QEI interrupt handler. Questa funzione gestisce l'interrupt
 * 				generato dal QEI
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void QEI_Interrupt(void)
{
	// Check whether if velocity timer overflow
	if (QEI_GetIntStatus(LPC_QEI, QEI_INTFLAG_TIM_Int) == SET) {

		/* Scrivo la velocità nella variabile Speed */
		Speed = QEI_GetVelocityCap(LPC_QEI);

		// Reset Interrupt flag pending
		QEI_IntClear(LPC_QEI, QEI_INTFLAG_TIM_Int);
	}
}

/*********************************************************************//**
 * @brief 		Inizializza le periferiche per il controllo dei
 * 				sensori.
 * @details		Inizializza le periferiche e i pin per il controllo
 * 				di endstop ed encoder.
 **********************************************************************/
void ReadSpeed(ENC_STATUS_Type *EncStatus, uint16_t encRes){

	/* Leggo la velocità attuale */
	//Controllo il verso di rotazione
	if((LPC_QEI->QEISTAT & 0b1)==0b1){
		EncStatus->ActSpeed = -QEI_CalculateRPM(LPC_QEI, Speed, encRes);
	}else{
		EncStatus->ActSpeed = QEI_CalculateRPM(LPC_QEI, Speed, encRes);
	}
}

/*********************************************************************//**
 * @brief 		Inizializza le periferiche per il controllo dei
 * 				sensori.
 * @details		Inizializza le periferiche e i pin per il controllo
 * 				di endstop ed encoder.
 **********************************************************************/
void ReadPosition(ENC_STATUS_Type *EncStatus){

	/*Leggo la posizione attuale*/
	EncStatus->ActPosition = QEI_GetPosition(LPC_QEI);


}





/**
 * @}
 */

/**
 * @}
 */
