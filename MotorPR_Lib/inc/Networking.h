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

/* Networking group --------------------------------------------------------------- */
/** @defgroup Networking Manager
 * @{
 */
#ifndef NETWORKING_H_
#define NETWORKING_H_

/* Includes ------------------------------------------------------------------- */
#include <lpc17xx_libcfg_default.h>
#include <SystemConfig.h>
#include <lpc17xx_gpio.h>
#include <lpc17xx_pinsel.h>
#include "lpc17xx_uart.h"
#include "CRC_Utils.h"
#include "ConfigManager.h"


/* Private Macros ------------------------------------------------------------- */

/* Pubblic Macros ------------------------------------------------------------- */
/** @defgroup Networking_Public_Macros Networking Public Macros
 * @{
 */

/**
 * @brief		Registri contenenti gli stati dei buffer 0-7.
 */
#define BUFSIZE				50


/**
 * @brief		Funzione per leggere un valore di registro.
 */
#define FUN_READREG			0x01
/**
 * @brief		Funzione per scrivere un valore di registro.
 */
#define FUN_SETREG			0x05
/**
 * @brief		Funzione per leggere lo stato.
 */
#define FUN_READSTAT		0x06

/**
 * @brief		Funzione per svuotare tutti i buffer abilitati.
 */
#define FUN_BUFCLEAN		0x07

/**
 * @brief		Funzione per scrivere nei buffer direttamente.
 */
#define FUN_BUFWRT			0x08

/**
 * @brief		Funzione per fare homing.
 */
#define FUN_GOHOME			0x09

/**
 * @brief		Registro contenente lo stato generale del sistema.
 */
#define REG_GENSTAT			0x0000
/**
 * @brief		Registro contenente lo stato dei dip-switch.
 */
#define REG_DIPSTAT			0x0001
/**
 * @brief		Registro contenente la temperatura del motore.
 */
#define REG_TEMPMOT			0x0002
/**
 * @brief		Registro contenente la temperatura del ponte-H.
 */
#define REG_TEMPHB			0x0003
/**
 * @brief		Registro contenente la tensione del motore.
 */
#define REG_TENSMOT			0x0004
/**
 * @brief		Registro contenente la corrente del motore.
 */
#define REG_CURMOT			0x0005
/**
 * @brief		Registri contenenti gli stati dei buffer 0-7.
 */
#define REG_BUFSTAT0		0x0010
#define REG_BUFSTAT1		0x0011
#define REG_BUFSTAT2		0x0012
#define REG_BUFSTAT3		0x0013
#define REG_BUFSTAT4		0x0014
#define REG_BUFSTAT5		0x0015
#define REG_BUFSTAT6		0x0016
#define REG_BUFSTAT7		0x0017

/**
 * @brief		Registri scrittura valore buffer 0-7.
 */
#define REG_BUFVAL0			0x0020
#define REG_BUFVAL1			0x0021
#define REG_BUFVAL2			0x0022
#define REG_BUFVAL3			0x0023
#define REG_BUFVAL4			0x0024
#define REG_BUFVAL5			0x0025
#define REG_BUFVAL6			0x0026
#define REG_BUFVAL7			0x0027

/*TUTTO QUELLO SOPRA A QUESTA RIGA VIENE TENUTO IN RAM (fino a 0x002F. TUTTO QUELLO SOTTO è SALVATO IN EEPROM.*/

/**
 * @brief		Registro contenete la modalità di funzionamento, master/slave.
 */
#define REG_MODMASL			0x0030

/**
 * @brief		Modalità master.
 */
#define MOD_MASTER			0x01

/**
 * @brief		Modalità slave.
 */
#define MOD_SLAVE			0x00

/**
 * @brief		Registro contenete i buffer attivi, 0b00000001 (ogni bit un buffer. LSB=0 , MSB=7).
 */
#define REG_BUFEN			0x0031

/**
 * @brief		Registri indirizzi schede slave collegate ai vari buffer 0-7.
 */
#define REG_BUFADD0			0x0032
#define REG_BUFADD1			0x0033
#define REG_BUFADD2			0x0034
#define REG_BUFADD3			0x0035
#define REG_BUFADD4			0x0036
#define REG_BUFADD5			0x0037
#define REG_BUFADD6			0x0038
#define REG_BUFADD7			0x0039


/**
 * @brief		Registro contenete la modalità di funzionamento.
 */
#define REG_MODPID			0x0040

/**
 * @brief		Modalità PID di velocità.
 */
#define MOD_PID_SPEED		0x00

/**
 * @brief		Modalità PID di velocità e posizione.
 */
#define MOD_PID_SPEEDPOS	0x01

/**
 * @brief		Registro contenete la modalità di gestione finecorsa.
 */
#define REG_MODENDSTOP		0x0041

/**
 * @brief		Modalità automatica. Se raggiunge il finecorsa si blocca il motore.
 */
#define MOD_ENDSTOP_AUTO	0x00

/**
 * @brief		Registro contenente il tipo di stato per fine corsa 1.
 */
#define REG_STAT_ENDSTOP1	0x0078

/**
 * @brief		Registro contenente il tipo di stato per fine corsa 2.
 */
#define REG_STAT_ENDSTOP2	0x0079

/**
 * @brief		Contatto NC, gestione dell'evento al rising.
 */
#define MOD_ENDSTOP_NC		0x00

/**
 * @brief		Contatto NO, gestione dell'evento al falling.
 */
#define MOD_ENDSTOP_NO		0x01

/**
 * @brief		Registro contenente la velocità massima del motore.
 */
#define REG_MAXSPEED		0x0044 /* Occupa anche 0x0045 16bit*/

/**
 * @brief		Registro contenente il valore della costante kp della velocità.
 */
#define REG_PID_V_KP		0x0046 /* Occupa 4 byte, 0x46,0x47,0x48,0x49 */

/**
 * @brief		Registro contenente il valore della costante ki della velocità.
 */
#define REG_PID_V_KI		0x0050 /* Occupa 4 byte, 0x51,0x52,0x53,0x54 */

/**
 * @brief		Registro contenente il valore della costante kd della velocità.
 */
#define REG_PID_V_KD		0x0055 /* Occupa 4 byte, 0x55,0x56,0x57,0x58 */

/**
 * @brief		Registro contenente il valore della costante kp della posizione.
 */
#define REG_PID_P_KP		0x0059 /* Occupa 4 byte, 0x59,0x60,0x61,0x62 */

/**
 * @brief		Registro contenente il valore della costante ki della posizione.
 */
#define REG_PID_P_KI		0x0063 /* Occupa 4 byte, 0x63,0x64,0x65,0x66 */

/**
 * @brief		Registro contenente il valore della costante kd della posizione.
 */
#define REG_PID_P_KD		0x0068 /* Occupa 4 byte, 0x68,0x69,0x70,0x71 */

/**
 * @brief		Registro contenente l'intervallo di tempo ogni quanto carica i dati in ram/alle altre schede.
 */
#define REG_BUFF_UPDATE		0x0072 /* Occupa 2byte, 0x0072, 0x0073 */

/**
 * @brief		Registro contenente lo stato dei pin GPIO IN.
 */
#define REG_GPIO_IN			0x0074

/**
 * @brief		Registro contenente lo stato dei pin GPIO OUT.
 */
#define REG_GPIO_OUT		0x0075

/**
 * @brief		Registro contenente i passi dell'encoder.
 */
#define REG_ENC_RES			0x0076 /* Occupa 2byte, 0x0076, 0x0077 */

/* Registro 0x0078 e 0x0079 usati per il finecorsa */




/* ERRORI DI COMUNICAZIONE RS485/RS422 */

/**
 * @brief		Errore: registro non esistente
 */
#define ERROR_NOREG			0x01

/**
 * @brief		Errore: Funzione non esistente
 */
#define ERROR_NOFUN			0x02

/**
 * @brief		Errore: indirizzo non valido
 */
#define ERROR_NOADD			0x03

/**
 * @brief		Errore: CRC errato
 */
#define ERROR_CRC			0x04

/**
 * @brief		Errore: CRC errato
 */
#define ERROR_NOWRITE		0x05

/**
 * @brief		Nessun errore nel comando
 */
#define NO_ERROR			0x0F

/**
 * @}
 */

/* Public Types --------------------------------------------------------------- */
/** @defgroup Networking_Public_Types Networking Public Types
 * @{
 */

/**
 * @brief Struttura dati ricezione seriale
 */
typedef struct
{
	unsigned char RxData[8]; 			/* Dati ricevuti */
	unsigned char Flag;					/* Flag nuovi dati */
}RS422_RX_Type;


/**
 * @brief Struttura gestione richieste seriale
 */
typedef struct
{
	uint8_t 	Address;	 				/* Indirizzo scheda */
	uint8_t		Function;					/* Funzione richiesta */
	uint16_t 	Register;					/* Registro sul quale agire */
	uint8_t 	Length;						/* Lunghezza messaggio */
	uint8_t		Message[128];				/* Messaggio*/
	uint16_t	CRC16;						/* CRC16 */
	uint8_t		CmdError;					/* Errori nell'ecuzione del comando */
}CommandMngBuffer_Type;

/**
 * @brief Struttura contenente le impostazioni e i valori di sistema
 */
typedef struct
{
	volatile uint16_t 	GENSTAT;	 				/* Stato generale della scheda */
	uint8_t				DIPSTAT;					/* Stato dip-switch della scheda */
	volatile uint8_t	TEMPMOT;					/* Temperatura motore */
	volatile uint8_t	TEMPHB;						/* Temperatura H-Bridge */
	volatile float		TENSMOT;					/* Tensione motore */
	volatile float		CURMOT;						/* Corrente motore */
	volatile int32_t	BUFVAL0[BUFSIZE];				/* Valore buffer 0 */
	volatile int32_t	BUFVAL1[BUFSIZE];				/* Valore buffer 1 */
	volatile int32_t	BUFVAL2[BUFSIZE];				/* Valore buffer 2 */
	volatile int32_t	BUFVAL3[BUFSIZE];				/* Valore buffer 3 */
	volatile int32_t	BUFVAL4[BUFSIZE];				/* Valore buffer 4 */
	volatile int32_t	BUFVAL5[BUFSIZE];				/* Valore buffer 5 */
	volatile int32_t	BUFVAL6[BUFSIZE];				/* Valore buffer 6 */
	volatile int32_t	BUFVAL7[BUFSIZE];				/* Valore buffer 7 */
	uint8_t				BUFINPNT[8];				/* Puntatori di input buffer circolari */
	uint8_t				BUFOUTPNT[8];				/* Puntatori di output buffer circolari */
	uint8_t				BUFFREE[8];					/* Spazio libero nei vari buffer */
	uint8_t				BUFADD0;					/* Indirizzo buffer 0 */
	uint8_t				BUFADD1;					/* Indirizzo buffer 1 */
	uint8_t				BUFADD2;					/* Indirizzo buffer 2 */
	uint8_t				BUFADD3;					/* Indirizzo buffer 3 */
	uint8_t				BUFADD4;					/* Indirizzo buffer 4 */
	uint8_t				BUFADD5;					/* Indirizzo buffer 5 */
	uint8_t				BUFADD6;					/* Indirizzo buffer 6 */
	uint8_t				BUFADD7;					/* Indirizzo buffer 7 */
	uint8_t				MODMASL;					/* Modalità master/slave */
	uint8_t 			BUFEN;						/* Buffer abilitati*/
	uint8_t 			MODPID;						/* Modalità pid */
	uint8_t 			MODENDSTOP;					/* Modalità end-stop*/
	uint8_t			 	STAT_ENDSTOP1;				/* Stato elettrico end-stop, NC o NO*/
	uint8_t			 	STAT_ENDSTOP2;				/* Stato elettrico end-stop, NC o NO*/
	uint8_t				ATHOME;
	uint32_t 			MAXSPEED;					/* Velocità massima pid posizione */
	float				PID_V_KP;					/* Costante Kp pid velocità */
	float				PID_V_KI;					/* Costante Ki pid velocità */
	float				PID_V_KD;					/* Costante Kd pid velocità */
	float				PID_P_KP;					/* Costante Kp pid posizione */
	float				PID_P_KI;					/* Costante Ki pid posizione */
	float				PID_P_KD;					/* Costante Kd pid posizione */
	uint16_t			BUFF_UPDATE;				/* Costante di tempo ogni quanto vengono eseguiti i punti */
	uint16_t			ENC_RES;					/* Risoluzione encoder */
}GlobalSettingValue_Type;

/**
 * @}
 * Networking_Public_Types
 */

/* Public Functions ----------------------------------------------------------- */
/** @defgroup Networking_Public_Functions Networking Public Functions
 * @{
 */
void RS422_Init(void);
uint8_t RS422_Interrupt(RS422_RX_Type *RX_Str);
void RS485_Init();
uint8_t RS485_Interrupt(RS422_RX_Type *RX_Str);
void RS485_SendCommand(CommandMngBuffer_Type *DecodeStr);

uint8_t Net_DecodeCommand(uint8_t *RxBuffer, CommandMngBuffer_Type *DecodeStr);


void CommandExec(CommandMngBuffer_Type *DecodeStr, GlobalSettingValue_Type *GlobalVal, LPC_UART_TypeDef *UARTx);
uint32_t ReadReg(uint16_t Register, GlobalSettingValue_Type *GlobaVal);
uint8_t WriteReg(uint16_t Register,uint32_t Value, GlobalSettingValue_Type *GlobalVal);

/**
 * @}
 */


#endif /* NETWORKING_H_ */


/**
 * @}
 * NETWORKING
 */
