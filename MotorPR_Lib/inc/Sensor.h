/**********************************************************************
* $Id$		Sensor.h				2013-08-18
*//**
* @file		Sensor.h
* @brief	Contiene le funzioni per la gestione dei sensori,
* 			in particolare encoder e fine corsa
* @version	0.1
* @date		18. Aug. 2013
* @author	Corradi Luca
*
*
***********************************************************************/

/* Driver group --------------------------------------------------------------- */
/** @defgroup Sensor
 * @{
 */

#ifndef SENSOR_H_
#define SENSOR_H_

/* Includes ------------------------------------------------------------------- */
#include <lpc17xx_libcfg_default.h>
#include <SystemConfig.h>
#include <lpc17xx_gpio.h>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_nvic.h>
#include <lpc17xx_qei.h>


/* Private Macros ------------------------------------------------------------- */

/* Pubblic Macros ------------------------------------------------------------- */
/** @defgroup Sensor_Public_Macros Stepper Public Macros
 * @{
 */

/**
 * @}
 */

/* Public Types --------------------------------------------------------------- */
/** @defgroup Sensor_Public_Types Stepper Public Types
 * @{
 */

/**
 * @brief		Struttura che contiene le informazioni sulla posizione e velocità.
 */

typedef struct
{
	int SetSpeed; 			/* Velocità Target */
	int ActSpeed;			/* Velocità Attuale */
	int SetPosition;		/* Posizione Target */
	int ActPosition;		/* Posizione Attuale */

}ENC_STATUS_Type;


/**
 * @}
 * Stepper_Public_Types
 */

/* Public Functions ----------------------------------------------------------- */
/** @defgroup Sensor_Public_Functions Stepper Public Functions
 * @{
 */

void InitSensor(void);
void QEI_Interrupt(void);
void ReadSpeed(ENC_STATUS_Type *EncStatus, uint16_t encRes);
void ReadPosition(ENC_STATUS_Type *EncStatus);


/**
 * @}
 */


#endif /* SENSOR_H_ */


/**
 * @}
 * SENSOR
 */
