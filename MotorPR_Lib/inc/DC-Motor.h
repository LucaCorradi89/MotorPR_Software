/**********************************************************************
+* $Id$		DC-Motor.c				2013-08-20
*//**
* @file		DC-Motor.c
* @brief	Contiene le funzioni per la gestione del motore DC
* @version	0.1
* @date		20. Aug. 2013
* @author	Corradi Luca
*
*
***********************************************************************/

/* DC-Motor group --------------------------------------------------------------- */
/** @defgroup DC-Motor
 * @{
 */

#ifndef DCMOTOR_H_
#define DCMOTOR_H_

/* Includes ------------------------------------------------------------------- */
#include <lpc17xx_libcfg_default.h>
#include <lpc17xx_gpio.h>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_mcpwm.h>
#include <SystemConfig.h>


/* Private Macros ------------------------------------------------------------- */

/* Pubblic Macros ------------------------------------------------------------- */
/** @defgroup DC-Motor_Public_Macros Stepper Public Macros
 * @{
 */

/**
 * @brief		Periodo del pwm.
 */
#define PWM_PERIOD 		5000

/**
 * @brief		Verso di rotazione orario.
 */
#define MOTOR_CW 		1

/**
 * @brief		Verso di rotazione anti-orario.
 */
#define MOTOR_CCW 		2

/**
 * @brief		Motore Frenato.
 */
#define MOTOR_BRAKE		3


/**
 * @}
 */

/* Public Types --------------------------------------------------------------- */
/** @defgroup DC-Motor_Public_Types Stepper Public Types
 * @{
 */

/**
 * @brief Imposta la modalità operativa del motore
 */
void MotorModeSet(unsigned char Mode);


void InitMotor(void);


void MotorSetDuty(LPC_MCPWM_TypeDef *MCPWMx, unsigned char channelNum, uint32_t DutyCycle);




/**
 * @}
 * DC-Motor_Public_Types
 */

/* Public Functions ----------------------------------------------------------- */
/** @defgroup DC-Motor_Public_Functions Stepper Public Functions
 * @{
 */

//Prototipi

/**
 * @}
 */


#endif /* DCMOTOR_H_ */


/**
 * @}
 * DC-MOTOR
 */
