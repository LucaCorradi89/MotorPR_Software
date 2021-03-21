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
#include <ConfigManager.h>

/* Private Types -------------------------------------------------------------- */
/** @defgroup Stepper_Private_Types Stepper Private Types
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
/** @defgroup Stepper_Private_Variables Stepper Private Variables
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
/** @defgroup Stepper_Private_Functions Stepper Private Functions
 * @{
 */
// Prototipi //

/*********************************************************************//**
 * @brief 		Imposta il pin che indica al controller del motore stepper
 * 				di compiere un passo.
 * @details		Il pin hardware è definito nella configurazione del
 * 				sistema. Si tiene conto della configurazione relativa allo
 * 				stato attivo del pin.
 * @param[in]	moveAxis		Asse di cui settare la direzione.
 * @param[in]	stepPinState	Direzione del motore. Può assumere i valori:
 * 									- ENABLE, per indicare lo stato attivo.
 * 									- DISABLE, per indicare lo stato non
 * 										attivo.
 * 									- #stepperState_busy
 * @todo		Implementare la funzione per settare il pin di step.
 * @todo		Modificare per usare degli stati definiti internamente.
 **********************************************************************/


/*********************************************************************//**
 * @brief 		Routine di init della eeprom.
 * @details		Routine di init della eeprom on-board interfacciata tramite i2c.
 *
 **********************************************************************/
void EEPROM_Init(void){

	/* Creo la struttura PinCfg per impostare l'I2C */
	PINSEL_CFG_Type PinCfg;

	PinCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;

	PinCfg.Funcnum = FUNCNUM_EEPROM_SDA;
	PinCfg.Portnum = PORTNUM_EEPROM_SDA;
	PinCfg.Pinnum = PINNUM_EEPROM_SDA;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = FUNCNUM_EEPROM_SCL;
	PinCfg.Portnum = PORTNUM_EEPROM_SCL;
	PinCfg.Pinnum = PINNUM_EEPROM_SCL;
	PINSEL_ConfigPin(&PinCfg);

	I2C_Init(LPC_I2C2, 100000);

	I2C_Cmd(LPC_I2C2, I2C_MASTER_MODE, ENABLE);
}

/*********************************************************************//**
 * @brief 		Routine di scrittura sulla eeprom.
 * @details		Routine di scrittura di un registro sulla eeprom.
 * @param[in]	address		Indirizzo in cui scrivere.
 * @param[in]	data		Dato da scrivere.
 *
 **********************************************************************/
void EEPROM_SetRegister32(uint16_t address , uint32_t data){

	I2C_M_SETUP_Type I2C_CFG;
	uint32_t cnt;
	uint8_t SendBuffer[6];

	SendBuffer[0] = (uint16_t)(address >> 8);
	SendBuffer[1] = (uint16_t)(address & 0xFF);
	SendBuffer[2] = (uint32_t)(data >> 24);
	SendBuffer[3] = (uint32_t)((data >> 16) & 0xFF);
	SendBuffer[4] = (uint32_t)((data >> 8) & 0xFF);
	SendBuffer[5] = (data & 0xFF);

	I2C_CFG.sl_addr7bit = 0b1010000;
	I2C_CFG.tx_length = 6;
	I2C_CFG.tx_data = SendBuffer;
	I2C_CFG.rx_data = NULL;
	I2C_CFG.rx_length = 0;

	I2C_MasterTransferData(LPC_I2C2, &I2C_CFG, I2C_TRANSFER_POLLING);

	for(cnt=0; cnt<0xFFFFF; cnt++);

}

/*********************************************************************//**
 * @brief 		Routine di scrittura sulla eeprom.
 * @details		Routine di scrittura di un registro sulla eeprom.
 * @param[in]	address		Indirizzo in cui scrivere.
 * @return		data		Valore contenuto nel registro letto
 **********************************************************************/
uint32_t EEPROM_ReadRegister32(uint16_t address){

	uint8_t SendBuffer[2], ReciveBuffer[4];
	I2C_M_SETUP_Type I2C_CFG;

	SendBuffer[0] = (uint16_t)(address>>8);
	SendBuffer[1] = (address & 0xFF);

	I2C_CFG.sl_addr7bit = 0b1010000;
	I2C_CFG.tx_length = 2;
	I2C_CFG.tx_data = SendBuffer;
	I2C_CFG.rx_data = ReciveBuffer;
	I2C_CFG.rx_length = 4;

	I2C_MasterTransferData(LPC_I2C2, &I2C_CFG, I2C_TRANSFER_POLLING);

	return (uint32_t)((ReciveBuffer[0] << 24) & 0xFF000000) | ((ReciveBuffer[1] << 16) & 0xFF0000) | ((ReciveBuffer[2] << 8) & 0xFF00) | (ReciveBuffer[3] & 0xFF);
}

/*********************************************************************//**
 * @brief 		Routine di scrittura sulla eeprom.
 * @details		Routine di scrittura di un registro sulla eeprom.
 * @param[in]	address		Indirizzo in cui scrivere.
 * @param[in]	data		Dato da scrivere.
 *
 **********************************************************************/
void EEPROM_SetRegister16(uint16_t address , uint16_t data){

	I2C_M_SETUP_Type I2C_CFG;
	uint32_t cnt;
	uint8_t SendBuffer[4];

	SendBuffer[0] = (uint16_t)(address >> 8);
	SendBuffer[1] = (uint16_t)(address & 0xFF);
	SendBuffer[2] = (uint16_t)(data >> 8);
	SendBuffer[3] = (uint16_t)(data & 0xFF);

	I2C_CFG.sl_addr7bit = 0b1010000;
	I2C_CFG.tx_length = 4;
	I2C_CFG.tx_data = SendBuffer;
	I2C_CFG.rx_data = NULL;
	I2C_CFG.rx_length = 0;

	I2C_MasterTransferData(LPC_I2C2, &I2C_CFG, I2C_TRANSFER_POLLING);

	for(cnt=0; cnt<0xFFFFF; cnt++);

}

/*********************************************************************//**
 * @brief 		Routine di scrittura sulla eeprom.
 * @details		Routine di scrittura di un registro sulla eeprom.
 * @param[in]	address		Indirizzo in cui scrivere.
 * @return		data		Valore contenuto nel registro letto
 **********************************************************************/
uint16_t EEPROM_ReadRegister16(uint16_t address){

	uint8_t SendBuffer[2], ReciveBuffer[2];
	I2C_M_SETUP_Type I2C_CFG;

	SendBuffer[0] = (uint16_t)(address>>8);
	SendBuffer[1] = (address & 0xFF);

	I2C_CFG.sl_addr7bit = 0b1010000;
	I2C_CFG.tx_length = 2;
	I2C_CFG.tx_data = SendBuffer;
	I2C_CFG.rx_data = ReciveBuffer;
	I2C_CFG.rx_length = 2;

	I2C_MasterTransferData(LPC_I2C2, &I2C_CFG, I2C_TRANSFER_POLLING);

	return (uint16_t)((ReciveBuffer[0] << 8) & 0xFF00) | ReciveBuffer[1];
}

/*********************************************************************//**
 * @brief 		Routine di scrittura sulla eeprom.
 * @details		Routine di scrittura di un registro sulla eeprom.
 * @param[in]	address		Indirizzo in cui scrivere.
 * @param[in]	data		Dato da scrivere.
 *
 **********************************************************************/
void EEPROM_SetRegister8(uint16_t address , uint8_t data){

	I2C_M_SETUP_Type I2C_CFG;
	uint32_t cnt;
	uint8_t SendBuffer[3];

	SendBuffer[0] = (uint16_t)(address >> 8);
	SendBuffer[1] = (uint16_t)(address & 0xFF);
	SendBuffer[2] = data;

	I2C_CFG.sl_addr7bit = 0b1010000;
	I2C_CFG.tx_length = 3;
	I2C_CFG.tx_data = SendBuffer;
	I2C_CFG.rx_data = NULL;
	I2C_CFG.rx_length = 0;

	I2C_MasterTransferData(LPC_I2C2, &I2C_CFG, I2C_TRANSFER_POLLING);

	for(cnt=0; cnt<0xFFFFF; cnt++);

}

/*********************************************************************//**
 * @brief 		Routine di scrittura sulla eeprom.
 * @details		Routine di scrittura di un registro sulla eeprom.
 * @param[in]	address		Indirizzo in cui scrivere.
 * @return		data		Valore contenuto nel registro letto
 **********************************************************************/
uint8_t EEPROM_ReadRegister8(uint16_t address){

	uint8_t SendBuffer[2], ReciveBuffer[1];
	I2C_M_SETUP_Type I2C_CFG;

	SendBuffer[0] = (uint16_t)(address>>8);
	SendBuffer[1] = (address & 0xFF);

	I2C_CFG.sl_addr7bit = 0b1010000;
	I2C_CFG.tx_length = 2;
	I2C_CFG.tx_data = SendBuffer;
	I2C_CFG.rx_data = ReciveBuffer;
	I2C_CFG.rx_length = 1;

	I2C_MasterTransferData(LPC_I2C2, &I2C_CFG, I2C_TRANSFER_POLLING);

	return ReciveBuffer[0];
}

/**
 * @}
 */

/* Public Functions ----------------------------------------------------------- */
/** @addtogroup Stepper_Public_Functions
 * @{
 */
/*********************************************************************//**
 * @brief 		Inizializza le periferiche per il pilotaggio dei
 * 				motori stepper.
 * @details		Per ora i due stepper sono collegati ai canali 0 ed 1
 * 				della periferica MCPWM. Qui si inizializzano questi due
 * 				canali e le variabili che saranno usate nel calcolo.
 **********************************************************************/

/**
 * @}
 */

/**
 * @}
 */
