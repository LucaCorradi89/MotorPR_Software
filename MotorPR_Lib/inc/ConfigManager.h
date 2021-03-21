/**********************************************************************
* $Id$		ConfigManager.c				2013-08-18
*//**
* @file		ConfigManager.c
* @brief	Contiene le funzioni per la gestione della configurazione
* @version	0.1
* @date		18. Aug. 2013
* @author	Corradi Luca
*
*
***********************************************************************/

/* Driver group --------------------------------------------------------------- */
/** @defgroup Config Manager
 * @{
 */

#ifndef CONFIGMANAGER_H_
#define CONFIGMANAGER_H_

/* Includes ------------------------------------------------------------------- */
#include <lpc17xx_libcfg_default.h>
#include <lpc17xx_gpio.h>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_i2c.h>
#include "SystemConfig.h"

/* Private Macros ------------------------------------------------------------- */

/* Pubblic Macros ------------------------------------------------------------- */
/** @defgroup ConfigManager_Public_Macros ConfigManager Public Macros
 * @{
 */
/**
 * @brief		Shift per aumentare la precisione dei calcoli interi.
 */

//Define


/**
 * @}
 */

/* Public Types --------------------------------------------------------------- */
/** @defgroup ConfigManager_Public_Types ConfigManager Public Types
 * @{
 */
/**
 * @brief Asse selezionato per il movimento
 */

//Strutture

/**
 * @}
 * Stepper_Public_Types
 */

/* Public Functions ----------------------------------------------------------- */
/** @defgroup ConfigManager_Public_Functions ConfigManager Public Functions
 * @{
 */
void EEPROM_Int(void);
void EEPROM_Init(void);
void EEPROM_SetRegister16(uint16_t address , uint16_t data);
uint16_t EEPROM_ReadRegister16(uint16_t address);
void EEPROM_SetRegister8(uint16_t address , uint8_t data);
uint8_t EEPROM_ReadRegister8(uint16_t address);
void EEPROM_SetRegister32(uint16_t address , uint32_t data);
uint32_t EEPROM_ReadRegister32(uint16_t address);
/**
 * @}
 */


#endif /* CONFIGMANAGER_H_ */


/**
 * @}
 * CONFIGMANAGER
 */
