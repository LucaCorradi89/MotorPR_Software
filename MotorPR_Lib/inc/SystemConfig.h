/**********************************************************************
* $Id$		SystemConfig.h				2013-08-18
*//**
* @file		SystemConfig.h
* @brief	Contiene le definizioni generali di tutto il sistema
* @version	0.1
* @date		18. Aug. 2013
* @author	Corradi Luca
*
*
***********************************************************************/

/* Driver group --------------------------------------------------------------- */
/** @defgroup SystemConfig
 * @{
 */


#ifndef SYSTEMCONFIG_H_
#define SYSTEMCONFIG_H_
/* Includes ------------------------------------------------------------------- */



/* Private Macros ------------------------------------------------------------- */

/* Pubblic Macros ------------------------------------------------------------- */
/** @defgroup SystemConfig_Public_Macros System Config Public Macros
 * @{
 */

/* Impostazioni generali */

/**
 * @brief		BaudRate connessione seriale RS422.
 */
#define RS422_BAUD				115200

/**
 * @brief		BaudRate connessione seriale RS485.
 */
#define RS485_BAUD				115200

/*Status LED*/

/**
 * @brief		Numero porta e pin del led STATUS
 */
#define PORTNUM_LED_STATUS 	4
#define PINNUM_LED_STATUS 	29

/**
 * @brief		Numero porta e pin del led ERROR
 */
#define PORTNUM_LED_ERROR 	1
#define PINNUM_LED_ERROR 	14

/**
 * @brief		Numero porta e pin del led BUS
 */
#define PORTNUM_LED_BUS 	1
#define PINNUM_LED_BUS	 	15



/*Setting Switch*/

/**
 * @brief		Numero porta e pin dello switch - RS422<->CAN
 */
#define PORTNUM_SWITCH_BUS 		1
#define PINNUM_SWITCH_BUS 		9
/**
 * @brief		Numero porta e pin dello switch - Master<->Slave
 */
#define PORTNUM_SWITCH_MS 		1
#define PINNUM_SWITCH_MS 		10
/**
 * @brief		Numero porta e pin dello switch - AUX1
 */
#define PORTNUM_SWITCH_AUX1 	1
#define PINNUM_SWITCH_AUX1	 	8
/**
 * @brief		Numero porta e pin dello switch - Address bit2
 */
#define PORTNUM_SWITCH_ADD2 	1
#define PINNUM_SWITCH_ADD2	 	4
/**
 * @brief		Numero porta e pin dello switch - Address bit1
 */
#define PORTNUM_SWITCH_ADD1		1
#define PINNUM_SWITCH_ADD1 		1
/**
 * @brief		Numero porta e pin dello switch - Address bit0
 */
#define PORTNUM_SWITCH_ADD0		1
#define PINNUM_SWITCH_ADD0 		0

/*GPIO*/

/**
 * @brief		Numero porta e pin dei GPIO
 */
#define PORTNUM_GPIO_IN1 		2
#define PINNUM_GPIO_IN1 		3
#define PORTNUM_GPIO_IN2 		2
#define PINNUM_GPIO_IN2			4
#define PORTNUM_GPIO_IN3 		2
#define PINNUM_GPIO_IN3 		9
#define PORTNUM_GPIO_IN4 		0
#define PINNUM_GPIO_IN4 		16
#define PORTNUM_GPIO_OUT1 		0
#define PINNUM_GPIO_OUT1 		22
#define PORTNUM_GPIO_OUT2 		0
#define PINNUM_GPIO_OUT2 		18
#define PORTNUM_GPIO_OUT3		0
#define PINNUM_GPIO_OUT3		17
#define PORTNUM_GPIO_OUT4 		0
#define PINNUM_GPIO_OUT4 		15

/*Encoder e fine corsa*/

/**
 * @brief		Numero porta e pin del segnale PHA dell'encoder.
 */
#define PORTNUM_ENCODER_PHA		1
#define PINNUM_ENCODER_PHA 		20
#define FUNCNUM_ENCODER_PHA		1

/**
 * @brief		Numero porta e pin del segnale PHB dell'encoder.
 */
#define PORTNUM_ENCODER_PHB		1
#define PINNUM_ENCODER_PHB 		23
#define FUNCNUM_ENCODER_PHB		1

/**
 * @brief		Numero porta e pin del segnale INDEX dell'encoder.
 */
#define PORTNUM_ENCODER_INDEX	1
#define PINNUM_ENCODER_INDEX	24
#define FUNCNUM_ENCODER_INDEX	1

/**
 * @brief		Numero porta e pin del Finecorsa 1
 * 				(DEVONO AVERE ABILITABILE L'IntOnChange!!!)
 */
#define PORTNUM_ENDSTOP1		0
#define PINNUM_ENDSTOP1			2

/**
 * @brief		Numero porta e pin del Finecorsa 2.
 * 				(DEVONO AVERE ABILITABILE L'IntOnChange!!!)
 */
#define PORTNUM_ENDSTOP2		0
#define PINNUM_ENDSTOP2			3

/* PONTE-H  */

/**
 * @brief		Numero porta e pin del segnale INA.
 */
#define PORTNUM_HBRIDGE_INA		1
#define PINNUM_HBRIDGE_INA		19
#define FUNCNUM_HBRIDGE_INA		0

/**
 * @brief		Numero porta e pin del segnale INB.
 */
#define PORTNUM_HBRIDGE_INB		1
#define PINNUM_HBRIDGE_INB		26
#define FUNCNUM_HBRIDGE_INB		0

/**
 * @brief		Numero porta e pin del segnale PWM.
 */
#define PORTNUM_HBRIDGE_PWM		1
#define PINNUM_HBRIDGE_PWM		25
#define FUNCNUM_HBRIDGE_PWM		1

/**
 * @brief		Numero porta e pin del segnale DIAGA.
 */
#define PORTNUM_HBRIDGE_DIAGA	0
#define PINNUM_HBRIDGE_DIAGA	30
#define FUNCNUM_HBRIDGE_DIAGA	0

/**
 * @brief		Numero porta e pin del segnale DIAGB.
 */
#define PORTNUM_HBRIDGE_DIAGB	0
#define PINNUM_HBRIDGE_DIAGB	29
#define FUNCNUM_HBRIDGE_DIAGB	0

/**
 * @brief		Numero porta e pin del segnale RS422-TX.
 */
#define PORTNUM_RS422_TX		2
#define PINNUM_RS422_TX			0
#define FUNCNUM_RS422_TX		2

/**
 * @brief		Numero porta e pin del segnale RS422-RX.
 */
#define PORTNUM_RS422_RX		2
#define PINNUM_RS422_RX			1
#define FUNCNUM_RS422_RX		2

/**
 * @brief		Numero porta e pin del segnale RS485-TX.
 */
#define PORTNUM_RS485_TX		0
#define PINNUM_RS485_TX			0
#define FUNCNUM_RS485_TX		2

/**
 * @brief		Numero porta e pin del segnale RS485-RX.
 */
#define PORTNUM_RS485_RX		0
#define PINNUM_RS485_RX			1
#define FUNCNUM_RS485_RX		2

/**
 * @brief		Numero porta e pin del segnale RS485-DE.
 */
#define PORTNUM_RS485_DE		2
#define PINNUM_RS485_DE			6
#define FUNCNUM_RS485_DE		0

/**
 * @brief		Numero porta e pin del segnale RS485-RE.
 */
#define PORTNUM_RS485_RE		2
#define PINNUM_RS485_RE			7
#define FUNCNUM_RS485_RE		0

/**
 * @brief		Numero porta e pin del segnale SDA EEPROM.
 */
#define PORTNUM_EEPROM_SDA		0
#define PINNUM_EEPROM_SDA		10
#define FUNCNUM_EEPROM_SDA		2

/**
 * @brief		Numero porta e pin del segnale SCL EEPROM.
 */
#define PORTNUM_EEPROM_SCL		0
#define PINNUM_EEPROM_SCL		11
#define FUNCNUM_EEPROM_SCL		2


/**
 * @}
 */

/* Public Types --------------------------------------------------------------- */

/* Public Functions ----------------------------------------------------------- */

#endif /* SYSTEMCONFIG_H_ */


/**
 * @}
 * SYSTEMCONFIG
 */


