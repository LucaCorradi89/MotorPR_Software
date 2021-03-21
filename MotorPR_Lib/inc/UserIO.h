/**********************************************************************
* $Id$		UserIO.h				2013-08-18
*//**
* @file		UserIO.h
* @brief	Contiene le funzioni per la gestione Led e del DipSwitch di configurazione
* @version	0.1
* @date		18. Aug. 2013
* @author	Corradi Luca
*
*
***********************************************************************/

/* Driver group --------------------------------------------------------------- */
/** @defgroup UserIO
 * @{
 */

#ifndef USERIO_H_
#define USERIO_H_

/* Includes ------------------------------------------------------------------- */
#include <SystemConfig.h>

/* Pubblic Macros ------------------------------------------------------------- */
/** @defgroup UserIO_Public_Macros UserIO Public Macros
 * @{
 */

/**
 * @brief		Led STATUS.
 */
#define LED_STATUS			1

/**
 * @brief		Led ERROR.
 */
#define LED_ERROR			2

/**
 * @brief		Led BUS.
 */
#define LED_BUS				3

/**
 * @brief		Bus di tipo CAN.
 */
#define BUS_CAN				0

/**
 * @brief		Bus di tipo RS422.
 */
#define BUS_RS422			1

/**
 * @brief		Scheda Slave.
 */
#define BOARD_SLAVE			0

/**
 * @brief		Scheda Master.
 */
#define BOARD_MASTER		1


/**
 * @}
 */

/* Public Types --------------------------------------------------------------- */
/** @defgroup UserIO_Public_Types UserIO Public Types
 * @{
 */

/**
 * @brief		Struttura che contiene i settaggi dati dal dip-switch.
 */
typedef struct
{
	unsigned char BusType; 		/*Tipologia di bus selezionato, RS485 o RS422*/
	unsigned char MasterSlave;	/*Master o Slave in base se la stazione è comandata da altre stazioni o meno*/
	unsigned char Aux;			/*Bit ausiliario si impostazione lasciato per eventuali impostazioni.*/
	unsigned char BusAdd;		/*Indirizzo di rete*/

}SWITCH_SETTING_Type;

/**
 * @brief		Struttura che contiene lo stato dei led.
 */
typedef struct
{
	unsigned char Status;
	unsigned char Error;
	unsigned char Bus;

}LED_STATUS_Type;

/**
 * @}
 * UserIO_Public_Types
 */

/* Public Functions ----------------------------------------------------------- */
/** @defgroup UserIO_Public_Functions UserIO Public Functions
 * @{
 */
void InitUserIO(void);

void LedOff(unsigned char led);
void LedOn(unsigned char led);
void LedUpdate(LED_STATUS_Type *LedStatus);

void LoadSetting(SWITCH_SETTING_Type *SwitchSetting);

uint8_t ReadGPIO_Input();
void SetGPIO_Output(uint8_t newState);
/**
 * @}
 */


#endif /* USERIO_H_ */


/**
 * @}
 * USERIO
 */
