/**********************************************************************
* $Id$		UserIO.c				2013-08-18
*//**
* @file		UserIO.c
* @brief	Contiene le funzioni per la gestione Led e del DipSwitch di configurazione
* @version	0.1
* @date		18. Aug. 2013
* @author	Corradi Luca
*
*
***********************************************************************/

/* Gruppo --------------------------------------------------------------------- */
/** @addtogroup UserIO
 * @{
 */

/* Includes ------------------------------------------------------------------- */
#include <lpc17xx_libcfg_default.h>
#include <lpc17xx_gpio.h>
#include <lpc17xx_pinsel.h>
#include <UserIO.h>

/* Private Types -------------------------------------------------------------- */
/** @defgroup UserIO_Private_Types UserIO Private Types
 * @{
 */

//Struttura

/**
 * @}
 */

/* Private Variables ---------------------------------------------------------- */
/** @defgroup UserIO_Private_Variables Stepper Private Variables
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
/** @defgroup UserIO_Private_Functions UserIO Private Functions
 * @{
 */

/*********************************************************************//**
 * @brief 		Si impostano i pin e i vari pull-up dove necessari
 * 				per led e setting switch
 * @details		I pin sono definititi nel file SystemConfig.h
 **********************************************************************/
void InitHwUserIO(void){

	/* Creo la struttura PinCfg*/
	PINSEL_CFG_Type PinCfg;

	/*Imposto GPIO-TRISTATE*/
	PinCfg.Funcnum = 0x00;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;

	/*Led ERROR*/
	PinCfg.Portnum = PORTNUM_LED_ERROR;
	PinCfg.Pinnum = PINNUM_LED_ERROR;
	PINSEL_ConfigPin(&PinCfg);
	/*Imposto come uscita*/
	GPIO_SetDir(PORTNUM_LED_ERROR,1<<PINNUM_LED_ERROR, 0x01);

	/*Led BUS*/
	PinCfg.Portnum = PORTNUM_LED_BUS;
	PinCfg.Pinnum = PINNUM_LED_BUS;
	PINSEL_ConfigPin(&PinCfg);
	/*Imposto come uscita*/
	GPIO_SetDir(PORTNUM_LED_BUS,1<<PINNUM_LED_BUS, 0x01);

	/*Led STATUS*/
	PinCfg.Portnum = PORTNUM_LED_STATUS;
	PinCfg.Pinnum = PINNUM_LED_STATUS;
	PINSEL_ConfigPin(&PinCfg);
	/*Imposto come uscita*/
	GPIO_SetDir(PORTNUM_LED_STATUS,1<<PINNUM_LED_STATUS, 0x01);

	/*GPIO-SETTING SWITCH*/
	/*Imposto i pin degli switch di impostazione*/
	PinCfg.Funcnum = 0x00;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;

	/*Switch AUX2*/
	PinCfg.Portnum = PORTNUM_SWITCH_BUS;
	PinCfg.Pinnum = PINNUM_SWITCH_BUS;
	PINSEL_ConfigPin(&PinCfg);
	/*Switch MS*/
	PinCfg.Portnum = PORTNUM_SWITCH_MS;
	PinCfg.Pinnum = PINNUM_SWITCH_MS;
	PINSEL_ConfigPin(&PinCfg);
	/*Switch AUX1*/
	PinCfg.Portnum = PORTNUM_SWITCH_AUX1;
	PinCfg.Pinnum = PINNUM_SWITCH_AUX1;
	PINSEL_ConfigPin(&PinCfg);
	/*Switch ADD2*/
	PinCfg.Portnum = PORTNUM_SWITCH_ADD2;
	PinCfg.Pinnum = PINNUM_SWITCH_ADD2;
	PINSEL_ConfigPin(&PinCfg);
	/*Switch ADD1*/
	PinCfg.Portnum = PORTNUM_SWITCH_ADD1;
	PinCfg.Pinnum = PINNUM_SWITCH_ADD1;
	PINSEL_ConfigPin(&PinCfg);
	/*Switch ADD0*/
	PinCfg.Portnum = PORTNUM_SWITCH_ADD0;
	PinCfg.Pinnum = PINNUM_SWITCH_ADD0;
	PINSEL_ConfigPin(&PinCfg);

	/* GPIO generici */
	/*Imposto GPIO-TRISTATE*/
	PinCfg.Funcnum = 0x00;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;

	/* Output 1 */
	PinCfg.Portnum = PORTNUM_GPIO_OUT1;
	PinCfg.Pinnum = PINNUM_GPIO_OUT1;
	PINSEL_ConfigPin(&PinCfg);
	/* Output 2 */
	PinCfg.Portnum = PORTNUM_GPIO_OUT2;
	PinCfg.Pinnum = PINNUM_GPIO_OUT2;
	PINSEL_ConfigPin(&PinCfg);
	/* Output 3 */
	PinCfg.Portnum = PORTNUM_GPIO_OUT3;
	PinCfg.Pinnum = PINNUM_GPIO_OUT3;
	PINSEL_ConfigPin(&PinCfg);
	/* Output 4 */
	PinCfg.Portnum = PORTNUM_GPIO_OUT4;
	PinCfg.Pinnum = PINNUM_GPIO_OUT4;
	PINSEL_ConfigPin(&PinCfg);

	/* Input 1 */
	PinCfg.Portnum = PORTNUM_GPIO_IN1;
	PinCfg.Pinnum = PINNUM_GPIO_IN1;
	PINSEL_ConfigPin(&PinCfg);
	/* Input 2 */
	PinCfg.Portnum = PORTNUM_GPIO_IN2;
	PinCfg.Pinnum = PINNUM_GPIO_IN2;
	PINSEL_ConfigPin(&PinCfg);
	/* Input 3 */
	PinCfg.Portnum = PORTNUM_GPIO_IN3;
	PinCfg.Pinnum = PINNUM_GPIO_IN3;
	PINSEL_ConfigPin(&PinCfg);
	/* Input 4 */
	PinCfg.Portnum = PORTNUM_GPIO_IN4;
	PinCfg.Pinnum = PINNUM_GPIO_IN4;
	PINSEL_ConfigPin(&PinCfg);

	/* Imposto come uscita gli out */
	GPIO_SetDir(PORTNUM_GPIO_OUT1,1<<PINNUM_GPIO_OUT1, 0x01);
	GPIO_SetDir(PORTNUM_GPIO_OUT2,1<<PINNUM_GPIO_OUT2, 0x01);
	GPIO_SetDir(PORTNUM_GPIO_OUT3,1<<PINNUM_GPIO_OUT3, 0x01);
	GPIO_SetDir(PORTNUM_GPIO_OUT4,1<<PINNUM_GPIO_OUT4, 0x01);

}


/**
 * @}
 */

/* Public Functions ----------------------------------------------------------- */
/** @addtogroup UserIO_Public_Functions
 * @{
 */

/*********************************************************************//**
 * @brief 		Imposta i pin e l'hardware necessario per i led e gli switch.
 * @details		Imposta i pin e l'hardware necessario per i led e gli switch.
 * 				Compresi Opendrain e pull-up per il dip switch.
 **********************************************************************/
void InitUserIO(void){

	InitHwUserIO();

}

/*********************************************************************//**
 * @brief 		Accende il led passato come parametro
 * @details		Questa funzione accende il led che passiamo come parametro.
 * @param[in]	led  - Indica il led che vogliamo accendere:
 * 				LED_STATUS 	-> Accende il led STATUS
 * 				LED_ERROR	-> Accende il led ERROR
 * 				LED_BUS		-> Accende il led BUS
 **********************************************************************/
void LedOn(unsigned char led){
	switch(led){
		case LED_STATUS:
			GPIO_SetValue(PORTNUM_LED_STATUS, 1<<PINNUM_LED_STATUS);
		break;

		case LED_ERROR:
			GPIO_SetValue(PORTNUM_LED_ERROR, 1<<PINNUM_LED_ERROR);
		break;

		case LED_BUS:
			GPIO_SetValue(PORTNUM_LED_BUS, 1<<PINNUM_LED_BUS);
		break;
	}
}

/*********************************************************************//**
 * @brief 		Spegne il led passato come parametro
 * @details		Questa funzione spegne il led che passiamo come parametro.
 * @param[in]	led  - Indica il led che vogliamo spegnere:
 * 				LED_STATUS 	-> Spegne il led STATUS
 * 				LED_ERROR	-> Spegne il led ERROR
 * 				LED_BUS		-> Spegne il led BUS
 **********************************************************************/
void LedOff(unsigned char led){
	switch(led){
		case LED_STATUS:
			GPIO_ClearValue(PORTNUM_LED_STATUS, 1<<PINNUM_LED_STATUS);
		break;

		case LED_ERROR:
			GPIO_ClearValue(PORTNUM_LED_ERROR, 1<<PINNUM_LED_ERROR);
		break;

		case LED_BUS:
			GPIO_ClearValue(PORTNUM_LED_BUS, 1<<PINNUM_LED_BUS);
		break;
	}
}

/*********************************************************************//**
 * @brief 		Aggiorna lo stato dei led.
 * @details		Questa funzione aggiorna lo stato dei led come
 * 				da struttura passata.
 * @param[in]	*LedStatus  - Puntatore alla struttura di tipo
 * 				LED_STATUS_Type che contiene lo stato dei 3 led:
 **********************************************************************/
void LedUpdate(LED_STATUS_Type *LedStatus){
	/*Aggiorno il led di stato*/
	if(LedStatus->Status){
		GPIO_SetValue(PORTNUM_LED_STATUS, 1<<PINNUM_LED_STATUS);
	}else{
		GPIO_ClearValue(PORTNUM_LED_STATUS, 1<<PINNUM_LED_STATUS);
	}
	/*Aggiorno il led bus*/
	if(LedStatus->Bus){
		GPIO_SetValue(PORTNUM_LED_BUS, 1<<PINNUM_LED_BUS);
	}else{
		GPIO_ClearValue(PORTNUM_LED_BUS, 1<<PINNUM_LED_BUS);
	}
	/*Aggiorno il led di stato*/
	if(LedStatus->Error){
		GPIO_SetValue(PORTNUM_LED_ERROR, 1<<PINNUM_LED_ERROR);
	}else{
		GPIO_ClearValue(PORTNUM_LED_ERROR, 1<<PINNUM_LED_ERROR);
	}
}


/*********************************************************************//**
 * @brief 		Legge le impostazioni da dipswitch.
 * @details		Questa funzione legge le impostazioni selezionate.
 * @param[in]	*SwitchSetting  - Puntatore alla struttura di tipo
 * 				SWITCH_SETTING_Type che contiene le impostazioni:
 **********************************************************************/
void LoadSetting(SWITCH_SETTING_Type *SwitchSetting){

	/*Leggo il tipo di bus*/
	SwitchSetting->BusType = (GPIO_ReadValue(PORTNUM_SWITCH_BUS) >> PINNUM_SWITCH_BUS) & 0b01;
	/*Leggo Master o slave*/
	SwitchSetting->MasterSlave = (GPIO_ReadValue(PORTNUM_SWITCH_MS) >> PINNUM_SWITCH_MS) & 0b01;
	/*Leggo il bit aux*/
	SwitchSetting->Aux = (GPIO_ReadValue(PORTNUM_SWITCH_AUX1) >> PINNUM_SWITCH_AUX1) & 0b01;
	/*Leggo l'indirizzo*/
	SwitchSetting->BusAdd = (GPIO_ReadValue(PORTNUM_SWITCH_ADD0) >> PINNUM_SWITCH_ADD0) & 0b001;
	SwitchSetting->BusAdd |= ((GPIO_ReadValue(PORTNUM_SWITCH_ADD1) >> PINNUM_SWITCH_ADD1) << 1) & 0b010;
	SwitchSetting->BusAdd |= ((GPIO_ReadValue(PORTNUM_SWITCH_ADD2) >> PINNUM_SWITCH_ADD2) << 2) & 0b100;
}

/*********************************************************************//**
 * @brief 		Legge lo stato dei 4 input generici.
 * @details		Questa funzione legge lo stato degli input.
 **********************************************************************/
uint8_t ReadGPIO_Input(){
	uint8_t val=0;

	val |= (uint32_t)((GPIO_ReadValue(PORTNUM_GPIO_IN1) >> PINNUM_GPIO_IN1) & 0b1);
	val |= (uint32_t)((GPIO_ReadValue(PORTNUM_GPIO_IN2) >> PINNUM_GPIO_IN2) & 0b1) << 1;
	val |= (uint32_t)((GPIO_ReadValue(PORTNUM_GPIO_IN3) >> PINNUM_GPIO_IN3) & 0b1) << 2;
	val |= (uint32_t)((GPIO_ReadValue(PORTNUM_GPIO_IN4) >> PINNUM_GPIO_IN4) & 0b1) << 3;

	return val;
}

/*********************************************************************//**
 * @brief 		Modifica lo stato degli out generici.
 * @details		Questa funzione modifica lo stato degli output generici.
 * @param[in]	newState stato degli output
 **********************************************************************/
void SetGPIO_Output(uint8_t newState){

	if(newState & 0b0001){
		GPIO_SetValue(PORTNUM_GPIO_OUT1, 1<<PINNUM_GPIO_OUT1);
	}else{
		GPIO_ClearValue(PORTNUM_GPIO_OUT1, 1<<PINNUM_GPIO_OUT1);
	}

	if(newState & 0b0010){
		GPIO_SetValue(PORTNUM_GPIO_OUT2, 1<<PINNUM_GPIO_OUT2);
	}else{
		GPIO_ClearValue(PORTNUM_GPIO_OUT2, 1<<PINNUM_GPIO_OUT2);
	}
	if(newState & 0b0100){
		GPIO_SetValue(PORTNUM_GPIO_OUT3, 1<<PINNUM_GPIO_OUT3);
	}else{
		GPIO_ClearValue(PORTNUM_GPIO_OUT3, 1<<PINNUM_GPIO_OUT3);
	}
	if(newState & 0b1000){
		GPIO_SetValue(PORTNUM_GPIO_OUT4, 1<<PINNUM_GPIO_OUT4);
	}else{
		GPIO_ClearValue(PORTNUM_GPIO_OUT4, 1<<PINNUM_GPIO_OUT4);
	}
}


/**
 * @}
 */

/**
 * @}
 */

