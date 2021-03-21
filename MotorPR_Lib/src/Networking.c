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
/** @addtogroup Networking
 * @{
 */

/* Includes ------------------------------------------------------------------- */
#include <Networking.h>
#include <UserIO.h>


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

/////////////////////

/**
 * @}
 */

/* Public Functions ----------------------------------------------------------- */
/** @addtogroup Stepper_Public_Functions
 * @{
 */

/*********************************************************************//**
 * @brief 		Inizializza la periferica per RS422.
 * @details		Inizializza le periferiche e i pin per il controllo
 * 				delle connessioni di rete. RS422 e prodocollo relativo.
 **********************************************************************/
void RS422_Init(void){

	/* Creo la struttura PinCfg per impostare la RS422 di comunicazione*/
	PINSEL_CFG_Type PinCfg;

	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.Funcnum = FUNCNUM_RS422_TX;
	PinCfg.Portnum = PORTNUM_RS422_TX;
	PinCfg.Pinnum = PINNUM_RS422_TX;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = FUNCNUM_RS422_RX;
	PinCfg.Portnum = PORTNUM_RS422_RX;
	PinCfg.Pinnum = PINNUM_RS422_RX;
	PINSEL_ConfigPin(&PinCfg);

	// UART Configuration structure variable
	UART_CFG_Type UARTConfigStruct;
	// UART FIFO configuration Struct variable
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;

	/* Initialize UART Configuration parameter structure to default state:
	 * Baudrate = 115200bps
	 * 8 data bit
	 * 1 Stop bit
	 * None parity
	 */
	UART_ConfigStructInit(&UARTConfigStruct);
	UARTConfigStruct.Baud_rate = RS422_BAUD;

	// Initialize UART0 peripheral with given to corresponding parameter
	UART_Init((LPC_UART_TypeDef *)LPC_UART1, &UARTConfigStruct);

	/* Initialize FIFOConfigStruct to default state:
	 * 				- FIFO_DMAMode = DISABLE
	 * 				- FIFO_Level = UART_FIFO_TRGLEV0
	 * 				- FIFO_ResetRxBuf = ENABLE
	 * 				- FIFO_ResetTxBuf = ENABLE
	 * 				- FIFO_State = ENABLE
	 */
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);

	//Imposto il livello di trig a 8
	UARTFIFOConfigStruct.FIFO_Level=UART_FIFO_TRGLEV2;

	// Initialize FIFO for UART0 peripheral
	UART_FIFOConfig((LPC_UART_TypeDef *)LPC_UART1, &UARTFIFOConfigStruct);

	//Enable interrupt on data ready
	UART_IntConfig  ((LPC_UART_TypeDef *)LPC_UART1, UART_INTCFG_RBR ,ENABLE) ;

	/* preemption = 3, sub-priority = 1 */
	NVIC_SetPriority(UART1_IRQn, ((0x01<<3)|0x02));

	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(UART1_IRQn);

	// Enable UART Transmit
	UART_TxCmd((LPC_UART_TypeDef *)LPC_UART1, ENABLE);

}

/*********************************************************************//**
 * @brief 		Routine di interrupt per la gestione della RS422.
 * @details		Routine di interrupt per la gestione della RS422, salva nella struttura
 * 				apposita i dati necessari.
 * @param[in]	RX_Str		Struttura che contiene l'array dove salvare i dati e lo stato
 *
 **********************************************************************/
uint8_t RS422_Interrupt(RS422_RX_Type *RX_Str){

	uint8_t intStat=0;
	intStat = (LPC_UART1->IIR & 0b1110);

	RX_Str->Flag = UART_Receive((LPC_UART_TypeDef *)LPC_UART1, RX_Str->RxData,7,0);

	if(intStat == 0b1100){
		return 1;
	}else{
		return 0;
	}

}


/*********************************************************************//**
 * @brief 		Routine di interrupt per la gestione della RS485.
 * @details		Routine di interrupt per la gestione della RS485, salva nella struttura
 * 				apposita i dati necessari.
 * @param[in]	RX_Str		Struttura che contiene l'array dove salvare i dati e lo stato
 *
 **********************************************************************/
uint8_t RS485_Interrupt(RS422_RX_Type *RX_Str){

	uint8_t intStat=0;
	intStat = (LPC_UART3->IIR & 0b1110);

	RX_Str->Flag = UART_Receive((LPC_UART_TypeDef *)LPC_UART3, RX_Str->RxData,7,0);

	if(intStat == 0b1100){
		return 1;
	}else{
		return 0;
	}

}



/*********************************************************************//**
 * @brief 		Decodifica i dati ricevuti dalla seriale.
 * @details		Gestisce i dati ricevuti dalla seriale, gestendo l'indirizzo di rete,
 * 				i comandi, funzioni, crc etc.
 * @param[in]	RxBuffer 	Puntatore al buffer contenente i dati ricevuti
 * @param[in]	DecodeStr 	Puntatore alla struttura contenente i comandi/dati.
 **********************************************************************/
uint8_t Net_DecodeCommand(uint8_t *RxBuffer, CommandMngBuffer_Type *DecodeStr){
	/* Gestisco i dati ricevuti */
	uint16_t n=0;
	uint8_t *BufPnt;
	/* Mi salvo il puntatore al buffer */
	BufPnt = RxBuffer;

	//Copio nei relativi posti i vari dati
	DecodeStr->Address = *(RxBuffer++);
	DecodeStr->Function = *(RxBuffer++);
	DecodeStr->Register = (uint16_t)(*(RxBuffer++) << 8);
	DecodeStr->Register |= (uint16_t)(*(RxBuffer++) & 0x00FF);
	DecodeStr->Length = *(RxBuffer++);
	for(n=0;n<DecodeStr->Length;n++){
		DecodeStr->Message[n] = *(RxBuffer++);
	}
	DecodeStr->CRC16 = (uint16_t)(*(RxBuffer++) << 8);
	DecodeStr->CRC16 |= (uint16_t)(*(RxBuffer++) & 0x00FF);

	/* Calcolo il CRC e controllo se combacia. */
	if(DecodeStr->CRC16 == CRC16(BufPnt,DecodeStr->Length+5)){
		DecodeStr->CmdError = NO_ERROR;
		return 1;
	}else{
		DecodeStr->CmdError = ERROR_CRC;
		return 0;
	}
}

/*********************************************************************//**
 * @brief 		Inizializza la periferica RS485.
 * @details		Inizializza le periferiche e i pin per il controllo
 * 				delle connessioni di rete. RS485 e protocollo relativo.
 **********************************************************************/
void RS485_Init(){
	/* Creo la struttura PinCfg per impostare la RS422 di comunicazione*/
	PINSEL_CFG_Type PinCfg;

	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.Funcnum = FUNCNUM_RS485_TX;
	PinCfg.Portnum = PORTNUM_RS485_TX;
	PinCfg.Pinnum = PINNUM_RS485_TX;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = FUNCNUM_RS485_RX;
	PinCfg.Portnum = PORTNUM_RS485_RX;
	PinCfg.Pinnum = PINNUM_RS485_RX;
	PINSEL_ConfigPin(&PinCfg);

	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.Funcnum = FUNCNUM_RS485_DE;
	PinCfg.Portnum = PORTNUM_RS485_DE;
	PinCfg.Pinnum = PINNUM_RS485_DE;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = FUNCNUM_RS485_RE;
	PinCfg.Portnum = PORTNUM_RS485_RE;
	PinCfg.Pinnum = PINNUM_RS485_RE;
	PINSEL_ConfigPin(&PinCfg);

	GPIO_SetDir(PORTNUM_RS485_DE,1<<PINNUM_RS485_DE ,1);
	GPIO_SetDir(PORTNUM_RS485_RE,1<<PINNUM_RS485_RE ,1);

	// UART Configuration structure variable
	UART_CFG_Type UARTConfigStruct;
	// UART FIFO configuration Struct variable
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;

	/* Initialize UART Configuration parameter structure to default state:
	 * Baudrate = 115200bps
	 * 8 data bit
	 * 1 Stop bit
	 * None parity
	 */
	UART_ConfigStructInit(&UARTConfigStruct);
	UARTConfigStruct.Baud_rate = RS485_BAUD;

	// Initialize UART3 peripheral with given to corresponding parameter
	UART_Init((LPC_UART_TypeDef *)LPC_UART3, &UARTConfigStruct);

	/* Initialize FIFOConfigStruct to default state:
	 * 				- FIFO_DMAMode = DISABLE
	 * 				- FIFO_Level = UART_FIFO_TRGLEV0
	 * 				- FIFO_ResetRxBuf = ENABLE
	 * 				- FIFO_ResetTxBuf = ENABLE
	 * 				- FIFO_State = ENABLE
	 */
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);

	//Imposto il livello di trig a 8
	UARTFIFOConfigStruct.FIFO_Level=UART_FIFO_TRGLEV2;

	// Initialize FIFO for UART0 peripheral
	UART_FIFOConfig((LPC_UART_TypeDef *)LPC_UART3, &UARTFIFOConfigStruct);

	//Enable interrupt on data ready
	UART_IntConfig  ( (LPC_UART_TypeDef *)LPC_UART3, UART_INTCFG_RBR ,ENABLE) ;

	/* preemption = 4, sub-priority = 1 */
	NVIC_SetPriority(UART3_IRQn, ((0x01<<3)|0x04));

	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(UART3_IRQn);

	// Enable UART Transmit
	UART_TxCmd((LPC_UART_TypeDef *)LPC_UART3, ENABLE);

	/* Abilito la ricezione */
	GPIO_ClearValue(PORTNUM_RS485_RE, 1<<PINNUM_RS485_RE);
	GPIO_ClearValue(PORTNUM_RS485_DE, 1<<PINNUM_RS485_DE);
}

/*********************************************************************//**
 * @brief 		Invia il comando su bus rs485 on board.
 * @details		Invia i comandi sulle schede slave attraverso il bus 485
 * @param[in]	RxBuffer 	Puntatore al buffer contenente i dati ricevuti
 * @param[in]	DecodeStr 	Puntatore alla struttura contenente i comandi/dati.
 **********************************************************************/
void RS485_SendCommand(CommandMngBuffer_Type *DecodeStr){

	uint8_t TxBuf[16];
	uint32_t cntFor;
	uint16_t crc16;

	/* Abilito la trasmissione e disattivo la ricezione */
	GPIO_SetValue(PORTNUM_RS485_RE, 1<<PINNUM_RS485_RE);
	GPIO_SetValue(PORTNUM_RS485_DE, 1<<PINNUM_RS485_DE);

	/* Impacchetto la risposta con il registro e la varia roba */
	TxBuf[0] = DecodeStr->Address;
	TxBuf[1] = DecodeStr->Function;
	TxBuf[2] = (DecodeStr->Register >> 8) & 0xFF;
	TxBuf[3] = DecodeStr->Register & 0xFF;
	TxBuf[4] = DecodeStr->Length;

	for(cntFor=0; cntFor<DecodeStr->Length; cntFor++){
		TxBuf[cntFor+5] = DecodeStr->Message[cntFor];
	}
	/* CRC calculation */
	crc16 = CRC16(TxBuf,5+DecodeStr->Length);
	TxBuf[5+DecodeStr->Length] = (uint8_t)(crc16 >> 8);
	TxBuf[6+DecodeStr->Length] = (uint8_t)(crc16 & 0xFF);
	while(UART_CheckBusy((LPC_UART_TypeDef *)LPC_UART3));
	/* Invio  */
	UART_Send((LPC_UART_TypeDef *)LPC_UART3, TxBuf, DecodeStr->Length+7, 1);
	while(UART_CheckBusy((LPC_UART_TypeDef *)LPC_UART3));
	/* Attivo la ricezione*/
	GPIO_ClearValue(PORTNUM_RS485_RE, 1<<PINNUM_RS485_RE);
	GPIO_ClearValue(PORTNUM_RS485_DE, 1<<PINNUM_RS485_DE);
}


/*********************************************************************//**
 * @brief 		Funzione che esegue il comando ricevuto.
 * @details		Funzione che esegue il comando ricevuto .
 * @param[in]	DecodeStr 	Stringa decodificata
 * @param[in]	*GlobalVal 	Indirizzo ai parametri globali
 * @param[in]	*UARTx 	Uart sulla quale rispondere
 **********************************************************************/
void CommandExec(CommandMngBuffer_Type *DecodeStr, GlobalSettingValue_Type *GlobalVal, LPC_UART_TypeDef *UARTx){

	volatile uint32_t response;
	volatile uint8_t error,BufCnt,PackLen;
	uint8_t TxBuf[16], bufEn;
	volatile uint32_t value;
	uint16_t crc16;

	/* Guardo che funzione devo fare */
	switch(DecodeStr->Function){

		/* Leggere un registro */
		case FUN_READREG:
			response = ReadReg(DecodeStr->Register, GlobalVal);
			/* Impacchetto la risposta con il registro e la varia roba */
			TxBuf[0] = DecodeStr->Address;
			TxBuf[1] = DecodeStr->Function;
			TxBuf[2] = (DecodeStr->Register >> 8) & 0xFF;
			TxBuf[3] = DecodeStr->Register & 0xFF;
			TxBuf[4] = (response >> 24) & 0xFF;
			TxBuf[5] = (response >> 16) & 0xFF;
			TxBuf[6] = (response >> 8) & 0xFF;
			TxBuf[7] = response & 0xFF;
			/* CRC calculation */
			crc16 = CRC16(TxBuf,6);
			TxBuf[8] = (crc16 >> 8) & 0xFF;
			TxBuf[9] = crc16 & 0xFF;
			PackLen = 10;
		break;

		/* Scrivere un registro */
		case FUN_SETREG:
			value = (uint32_t)(DecodeStr->Message[3] | (DecodeStr->Message[2]) << 8 | (DecodeStr->Message[1]) << 16 | (DecodeStr->Message[0]) << 24);
			error = WriteReg(DecodeStr->Register, value , GlobalVal);

			/* Impacchetto la risposta con il registro e la varia roba */
			TxBuf[0] = DecodeStr->Address;
			TxBuf[1] = DecodeStr->Function;
			TxBuf[2] = (DecodeStr->Register >> 8) & 0xFF;
			TxBuf[3] = DecodeStr->Register & 0xFF;
			TxBuf[4] = error;
			/* CRC calculation */
			crc16 = CRC16(TxBuf,5);
			TxBuf[5] = (crc16 >> 8) & 0xFF;
			TxBuf[6] = crc16 & 0xFF;
			PackLen = 7;

		break;

		/* Home */
		case FUN_GOHOME:
			GlobalVal->ATHOME = 0x0F;
			/* Impacchetto la risposta con il registro e la varia roba */
			TxBuf[0] = DecodeStr->Address;
			TxBuf[1] = DecodeStr->Function;
			TxBuf[2] = 0x00;
			TxBuf[3] = 0x00;
			TxBuf[4] = NO_ERROR;
			/* CRC calculation */
			crc16 = CRC16(TxBuf,5);
			TxBuf[5] = (crc16 >> 8) & 0xFF;
			TxBuf[6] = crc16 & 0xFF;
			PackLen = 7;

		break;
		/* Leggere lo stato generale */
		case FUN_READSTAT:
			response = GlobalVal->GENSTAT;
			/* Impacchetto la risposta con il registro e la varia roba */
			TxBuf[0] = DecodeStr->Address;
			TxBuf[1] = DecodeStr->Function;
			TxBuf[2] = (DecodeStr->Register >> 8) & 0xFF;
			TxBuf[3] = DecodeStr->Register & 0xFF;
			TxBuf[4] = (response >> 8) & 0xFF;
			TxBuf[5] = response & 0xFF;
			/* CRC calculation */
			crc16 = CRC16(TxBuf,6);
			TxBuf[6] = (crc16 >> 8) & 0xFF;
			TxBuf[7] = crc16 & 0xFF;
			PackLen = 8;
		break;

		/* Scrivere nei buffer abilitati i valori */
		case FUN_BUFWRT:
			/* Resetto il contatore dei buffer elaborati */
			BufCnt=0;
			uint64_t BuffStat=0;
			bufEn = GlobalVal->BUFEN;
			/* Buffer 0 */
			if(bufEn & 0b00000001){
				if(GlobalVal->BUFFREE[0]>=1){
					GlobalVal->BUFVAL0[GlobalVal->BUFINPNT[0]] = (uint32_t)(DecodeStr->Message[(3+BufCnt)] | (DecodeStr->Message[(2+BufCnt)]) << 8 | (DecodeStr->Message[(1+BufCnt)]) << 16 | (DecodeStr->Message[(0+BufCnt)]) << 24);
					/* Sono nell'ultima locazione, resetto il puntatore di input */
					if(GlobalVal->BUFINPNT[0]>=(BUFSIZE-1)){
						GlobalVal->BUFINPNT[0]=0;
					}else{
						++GlobalVal->BUFINPNT[0];
					}
					--GlobalVal->BUFFREE[0];
					/* Scrivo il valore di byte liberi */
					BuffStat |= (uint64_t)(GlobalVal->BUFFREE[0] << (8*BufCnt));
					BufCnt = BufCnt+4;
				}
			}
			/* Buffer 1 */
			if(bufEn & 0b00000010){
				/* Scrivo nel buffer solo se c'è ancora spazio libero */
				if(GlobalVal->BUFFREE[1]>=1){
					GlobalVal->BUFVAL1[GlobalVal->BUFINPNT[1]] = (uint32_t)(DecodeStr->Message[(3+BufCnt)] | (DecodeStr->Message[(2+BufCnt)]) << 8 | (DecodeStr->Message[(1+BufCnt)]) << 16 | (DecodeStr->Message[(0+BufCnt)]) << 24);
					if(GlobalVal->BUFINPNT[1]>=(BUFSIZE-1)){
						GlobalVal->BUFINPNT[1]=0;
					}else{
						++GlobalVal->BUFINPNT[1];
					}
					--GlobalVal->BUFFREE[1];
					BuffStat |= (uint64_t)(GlobalVal->BUFFREE[1] << (8*BufCnt));
					BufCnt = BufCnt+4;
				}
			}
			/* Buffer 2 */
			if(bufEn & 0b00000100){
				/* Scrivo nel buffer solo se c'è ancora spazio libero */
				if(GlobalVal->BUFFREE[2]>=1){
					GlobalVal->BUFVAL2[GlobalVal->BUFINPNT[2]] = (uint32_t)(DecodeStr->Message[(3+BufCnt)] | (DecodeStr->Message[(2+BufCnt)]) << 8 | (DecodeStr->Message[(1+BufCnt)]) << 16 | (DecodeStr->Message[(0+BufCnt)]) << 24);
					if(GlobalVal->BUFINPNT[2]>=(BUFSIZE-1)){
						GlobalVal->BUFINPNT[2]=0;
					}else{
						++GlobalVal->BUFINPNT[2];
					}
					--GlobalVal->BUFFREE[2];
					BuffStat |= (uint64_t)(GlobalVal->BUFFREE[2] << (8*BufCnt));

					BufCnt = BufCnt+4;
				}
			}
			/* Buffer 3 */
			if(bufEn & 0b00001000){
				/* Scrivo nel buffer solo se c'è ancora spazio libero */
				if(GlobalVal->BUFFREE[3]>=1){
					GlobalVal->BUFVAL3[GlobalVal->BUFINPNT[3]] = (uint32_t)(DecodeStr->Message[(3+BufCnt)] | (DecodeStr->Message[(2+BufCnt)]) << 8 | (DecodeStr->Message[(1+BufCnt)]) << 16 | (DecodeStr->Message[(0+BufCnt)]) << 24);
					++GlobalVal->BUFINPNT[3];
					--GlobalVal->BUFFREE[3];
					BuffStat |= (uint64_t)(GlobalVal->BUFFREE[3] << (8*BufCnt));

					BufCnt = BufCnt+4;
				}
			}
			/* Buffer 4 */
			if(bufEn & 0b00010000){
				/* Scrivo nel buffer solo se c'è ancora spazio libero */
				if(GlobalVal->BUFFREE[4]>=1){
					GlobalVal->BUFVAL4[GlobalVal->BUFINPNT[4]] = (uint32_t)(DecodeStr->Message[(3+BufCnt)] | (DecodeStr->Message[(2+BufCnt)]) << 8 | (DecodeStr->Message[(1+BufCnt)]) << 16 | (DecodeStr->Message[(0+BufCnt)]) << 24);
					++GlobalVal->BUFINPNT[4];
					--GlobalVal->BUFFREE[4];
					BuffStat |= (uint64_t)(GlobalVal->BUFFREE[4] << (8*BufCnt));

					BufCnt = BufCnt+4;
				}
			}
			/* Buffer 5 */
			if(bufEn & 0b00100000){
				/* Scrivo nel buffer solo se c'è ancora spazio libero */
				if(GlobalVal->BUFFREE[5]>=1){
					GlobalVal->BUFVAL5[GlobalVal->BUFINPNT[5]] = (uint32_t)(DecodeStr->Message[(3+BufCnt)] | (DecodeStr->Message[(2+BufCnt)]) << 8 | (DecodeStr->Message[(1+BufCnt)]) << 16 | (DecodeStr->Message[(0+BufCnt)]) << 24);
					++GlobalVal->BUFINPNT[5];
					--GlobalVal->BUFFREE[5];
					BuffStat |= (uint64_t)(GlobalVal->BUFFREE[5] << (8*BufCnt));

					BufCnt = BufCnt+4;
				}
			}
			/* Buffer 6 */
			if(bufEn & 0b01000000){
				/* Scrivo nel buffer solo se c'è ancora spazio libero */
				if(GlobalVal->BUFFREE[6]>=1){
					GlobalVal->BUFVAL6[GlobalVal->BUFINPNT[6]] = (uint32_t)(DecodeStr->Message[(3+BufCnt)] | (DecodeStr->Message[(2+BufCnt)]) << 8 | (DecodeStr->Message[(1+BufCnt)]) << 16 | (DecodeStr->Message[(0+BufCnt)]) << 24);
					++GlobalVal->BUFINPNT[6];
					--GlobalVal->BUFFREE[6];
					BuffStat |= (uint64_t)(GlobalVal->BUFFREE[6] << (8*BufCnt));
					BufCnt = BufCnt+4;
				}
			}
			/* Buffer 7*/
			if(bufEn & 0b10000000){
				/* Scrivo nel buffer solo se c'è ancora spazio libero */
				if(GlobalVal->BUFFREE[7]>=1){
					GlobalVal->BUFVAL7[GlobalVal->BUFINPNT[7]] = (uint32_t)(DecodeStr->Message[(3+BufCnt)] | (DecodeStr->Message[(2+BufCnt)]) << 8 | (DecodeStr->Message[(1+BufCnt)]) << 16 | (DecodeStr->Message[(0+BufCnt)]) << 24);
					++GlobalVal->BUFINPNT[7];
					--GlobalVal->BUFFREE[7];
					BuffStat |= (uint64_t)(GlobalVal->BUFFREE[7] << (8*BufCnt));
					BufCnt = BufCnt+4;
				}
			}

			/* Impacchetto la risposta con il registro e la varia roba */
			TxBuf[0] = DecodeStr->Address;
			TxBuf[1] = DecodeStr->Function;
			TxBuf[2] = (DecodeStr->Register >> 8) & 0xFF;
			TxBuf[3] = DecodeStr->Register & 0xFF;
			TxBuf[4] = BufCnt;
			TxBuf[5] = 0xAB;
			TxBuf[6] = 	(uint64_t)(BuffStat >> 56) & 0xFF;
			TxBuf[7] = 	(uint64_t)(BuffStat >> 48) & 0xFF;
			TxBuf[8] = 	(uint64_t)(BuffStat >> 40) & 0xFF;
			TxBuf[9] = 	(uint64_t)(BuffStat >> 32) & 0xFF;
			TxBuf[10] =	(uint64_t)(BuffStat >> 24) & 0xFF;
			TxBuf[11] = (uint64_t)(BuffStat >> 16) & 0xFF;
			TxBuf[12] = (uint64_t)(BuffStat >> 8)  & 0xFF;
			TxBuf[13] = (uint64_t)(BuffStat & 0xFF);
			/* CRC calculation */
			crc16 = CRC16(TxBuf,14);
			TxBuf[14] = (crc16 >> 8) & 0xFF;
			TxBuf[15] = crc16 & 0xFF;
			PackLen = 16;
		break;

		/* Svuotare tutti i buffer abilitati */
		case FUN_BUFCLEAN:
			bufEn = GlobalVal->BUFEN;
			BufCnt=0;
			uint8_t Cnt;
			/* Buffer 0 */
			if(bufEn & 0b00000001){
				for(Cnt=0;Cnt<BUFSIZE;Cnt++){
					GlobalVal->BUFVAL0[Cnt]=0;
				}
				GlobalVal->BUFINPNT[0]=0;
				GlobalVal->BUFOUTPNT[0]=0;
				GlobalVal->BUFFREE[0]=BUFSIZE;
				BufCnt++;
			}
			/* Buffer 1 */
			if(bufEn & 0b00000010){
				for(Cnt=0;Cnt<BUFSIZE;Cnt++){
					GlobalVal->BUFVAL1[Cnt]=0;
				}
				GlobalVal->BUFINPNT[1]=0;
				GlobalVal->BUFOUTPNT[1]=0;
				GlobalVal->BUFFREE[1]=BUFSIZE;
				BufCnt++;
			}
			/* Buffer 2 */
			if(bufEn & 0b00000100){
				for(Cnt=0;Cnt<BUFSIZE;Cnt++){
					GlobalVal->BUFVAL2[Cnt]=0;
				}
				GlobalVal->BUFINPNT[2]=0;
				GlobalVal->BUFOUTPNT[2]=0;
				GlobalVal->BUFFREE[2]=BUFSIZE;
				BufCnt++;
			}
			/* Buffer 3 */
			if(bufEn & 0b00001000){
				for(Cnt=0;Cnt<BUFSIZE;Cnt++){
					GlobalVal->BUFVAL3[Cnt]=0;
				}
				GlobalVal->BUFINPNT[3]=0;
				GlobalVal->BUFOUTPNT[3]=0;
				GlobalVal->BUFFREE[3]=BUFSIZE;
				BufCnt++;
			}
			/* Buffer 4 */
			if(bufEn & 0b00010000){
				for(Cnt=0;Cnt<BUFSIZE;Cnt++){
					GlobalVal->BUFVAL4[Cnt]=0;
				}
				GlobalVal->BUFINPNT[4]=0;
				GlobalVal->BUFOUTPNT[4]=0;
				GlobalVal->BUFFREE[4]=BUFSIZE;
				BufCnt++;
			}
			/* Buffer 5 */
			if(bufEn & 0b00100000){
				for(Cnt=0;Cnt<BUFSIZE;Cnt++){
					GlobalVal->BUFVAL5[Cnt]=0;
				}
				GlobalVal->BUFINPNT[5]=0;
				GlobalVal->BUFOUTPNT[5]=0;
				GlobalVal->BUFFREE[5]=BUFSIZE;
				BufCnt++;
			}
			/* Buffer 6 */
			if(bufEn & 0b01000000){
				for(Cnt=0;Cnt<BUFSIZE;Cnt++){
					GlobalVal->BUFVAL6[Cnt]=0;
				}
				GlobalVal->BUFINPNT[6]=0;
				GlobalVal->BUFOUTPNT[6]=0;
				GlobalVal->BUFFREE[6]=BUFSIZE;
				BufCnt++;
			}
			/* Buffer 7*/
			if(bufEn & 0b10000000){
				for(Cnt=0;Cnt<BUFSIZE;Cnt++){
					GlobalVal->BUFVAL7[Cnt]=0;
				}
				GlobalVal->BUFINPNT[7]=0;
				GlobalVal->BUFOUTPNT[7]=0;
				GlobalVal->BUFFREE[7]=BUFSIZE;
				BufCnt++;
			}
			/* Impacchetto la risposta con il registro e la varia roba */
			TxBuf[0] = DecodeStr->Address;
			TxBuf[1] = DecodeStr->Function;
			TxBuf[2] = (DecodeStr->Register >> 8) & 0xFF;
			TxBuf[3] = DecodeStr->Register & 0xFF;
			TxBuf[4] = BufCnt;
			TxBuf[5] = NO_ERROR;
			/* CRC calculation */
			crc16 = CRC16(TxBuf,6);
			TxBuf[6] = (crc16 >> 8) & 0xFF;
			TxBuf[7] = crc16 & 0xFF;
			PackLen = 8;

		break;

		/* Funzione non esistente */
		default:
			DecodeStr->CmdError = ERROR_NOFUN;
			/* Impacchetto la risposta con il registro e la varia roba */
			TxBuf[0] = DecodeStr->Address;
			TxBuf[1] = DecodeStr->Function;
			TxBuf[2] = (DecodeStr->Register >> 8) & 0xFF;
			TxBuf[3] = DecodeStr->Register & 0xFF;
			TxBuf[4] = ERROR_NOFUN;
			/* CRC calculation */
			crc16 = CRC16(TxBuf,5);
			TxBuf[5] = (crc16 >> 8) & 0xFF;
			TxBuf[6] = crc16 & 0xFF;
			PackLen = 7;
		break;

	}
	if(UARTx == LPC_UART3){
//		/* Abilito la trasmissione e disattivo la ricezione */
//		GPIO_SetValue(PORTNUM_RS485_RE, 1<<PINNUM_RS485_RE);
//		GPIO_SetValue(PORTNUM_RS485_DE, 1<<PINNUM_RS485_DE);

	}else{
		/* Risposta  */
		UART_Send(UARTx, TxBuf, PackLen, 1);
	}
	if(UARTx == LPC_UART3){
//		/* Abilito la trasmissione e disattivo la ricezione */
//		GPIO_ClearValue(PORTNUM_RS485_RE, 1<<PINNUM_RS485_RE);
//		GPIO_ClearValue(PORTNUM_RS485_DE, 1<<PINNUM_RS485_DE);

	}
}

/*********************************************************************//**
 * @brief 		Funzione che legge un registro.
 * @details		Funzione che legge un registro.
 * @param[in]	Register 	Registro da leggere
 * @param[in]	GlobalVal 	Puntatore alla struttura con i dati di sistema
 **********************************************************************/
uint32_t ReadReg(uint16_t Register, GlobalSettingValue_Type *GlobalVal){

	switch(Register){

		case REG_GENSTAT:
			return GlobalVal->GENSTAT;
		break;

		case REG_DIPSTAT:
			return GlobalVal->DIPSTAT;
		break;

		case REG_TEMPMOT:
			return GlobalVal->TEMPMOT;
		break;

		case REG_TEMPHB:
			return GlobalVal->TEMPHB;
		break;

		case REG_TENSMOT:
			return (uint32_t)(GlobalVal->TENSMOT*10.0);
		break;

		case REG_CURMOT:
			return (uint32_t)(GlobalVal->CURMOT*10.0);
		break;

		case REG_BUFADD0:
			return GlobalVal->BUFADD0;
		break;

		case REG_BUFADD1:
			return GlobalVal->BUFADD1;
		break;

		case REG_BUFADD2:
			return GlobalVal->BUFADD2;
		break;

		case REG_BUFADD3:
			return GlobalVal->BUFADD3;
		break;

		case REG_BUFADD4:
			return GlobalVal->BUFADD4;
		break;

		case REG_BUFADD5:
			return GlobalVal->BUFADD5;
		break;

		case REG_BUFADD6:
			return GlobalVal->BUFADD6;
		break;

		case REG_BUFADD7:
			return GlobalVal->BUFADD7;
		break;

		case REG_MODMASL:
			return GlobalVal->MODMASL;
		break;

		case REG_BUFEN:
			return GlobalVal->BUFEN;
		break;

		case REG_MODPID:
			return GlobalVal->MODPID;
		break;

		case REG_MODENDSTOP:
			return GlobalVal->MODENDSTOP;
		break;

		case REG_MAXSPEED:
			return GlobalVal->MAXSPEED;
		break;

		case REG_BUFF_UPDATE:
			return GlobalVal->BUFF_UPDATE;
		break;

		case REG_ENC_RES:
			return GlobalVal->ENC_RES;
		break;

		case REG_PID_V_KP:
			return (uint32_t)(GlobalVal->PID_V_KP*1000.0);
		break;

		case REG_PID_V_KI:
			return (uint32_t)(GlobalVal->PID_V_KI*1000.0);
		break;

		case REG_PID_V_KD:
			return (uint32_t)(GlobalVal->PID_V_KD*1000.0);
		break;

		case REG_PID_P_KP:
			return (uint32_t)(GlobalVal->PID_P_KP*1000.0);
		break;

		case REG_PID_P_KI:
			return (uint32_t)(GlobalVal->PID_P_KI*1000.0);
		break;

		case REG_PID_P_KD:
			return (uint32_t)(GlobalVal->PID_P_KD*1000.0);
		break;

		case REG_GPIO_IN:
			return ReadGPIO_Input();
		break;

		case REG_STAT_ENDSTOP1:
			return (uint8_t)(GlobalVal->STAT_ENDSTOP1);
		break;

		case REG_STAT_ENDSTOP2:
			return (uint8_t)(GlobalVal->STAT_ENDSTOP2);
		break;

		/* registro non esistente */
		default:
			return 0;
		break;
	}
}

/*********************************************************************//**
 * @brief 		Funzione che scrive in un registro.
 * @details		Funzione che scrive in un registro.
 * @param[in]	Register 	Registro in cui scrivere
 * @param[in]	Value	 	Valore da scrivere
 * @param[in]	GlobalVal 	Puntatore alla struttura con i dati di sistema
 **********************************************************************/
uint8_t WriteReg(uint16_t Register,uint32_t Value, GlobalSettingValue_Type *GlobalVal){

	switch(Register){

		case REG_BUFADD0:
			GlobalVal->BUFADD0 = (uint8_t)(Value);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister8(REG_BUFADD0, (uint8_t)Value);
			return NO_ERROR;
		break;

		case REG_BUFADD1:
			GlobalVal->BUFADD1 = (uint8_t)(Value);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister8(REG_BUFADD1, (uint8_t)Value);
			return NO_ERROR;
		break;

		case REG_BUFADD2:
			GlobalVal->BUFADD2 = (uint8_t)(Value);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister8(REG_BUFADD2, (uint8_t)Value);
			return NO_ERROR;
		break;

		case REG_BUFADD3:
			GlobalVal->BUFADD3 = (uint8_t)(Value);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister8(REG_BUFADD3, (uint8_t)Value);
			return NO_ERROR;
		break;

		case REG_BUFADD4:
			GlobalVal->BUFADD4 = (uint8_t)(Value);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister8(REG_BUFADD4, (uint8_t)Value);
			return NO_ERROR;
		break;

		case REG_BUFADD5:
			GlobalVal->BUFADD5 = (uint8_t)(Value);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister8(REG_BUFADD5, (uint8_t)Value);
			return NO_ERROR;
		break;

		case REG_BUFADD6:
			GlobalVal->BUFADD6 = (uint8_t)(Value);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister8(REG_BUFADD6, (uint8_t)Value);
			return NO_ERROR;
		break;

		case REG_BUFADD7:
			GlobalVal->BUFADD7 = (uint8_t)(Value);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister8(REG_BUFADD7, (uint8_t)Value);
			return NO_ERROR;
		break;

		case REG_BUFEN:
			GlobalVal->BUFEN = (uint8_t)(Value);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister8(REG_BUFEN, (uint8_t)Value);
			return NO_ERROR;
		break;

		case REG_MODPID:
			GlobalVal->MODPID = (uint8_t)(Value);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister8(REG_MODPID, (uint8_t)Value);
			return NO_ERROR;
		break;

		case REG_MODENDSTOP:
			GlobalVal->MODENDSTOP = (uint8_t)(Value);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister8(REG_MODENDSTOP, (uint8_t)Value);
			return NO_ERROR;
		break;

		case REG_MAXSPEED:
			GlobalVal->MAXSPEED = (uint16_t)(Value);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister16(REG_MAXSPEED, (uint16_t)Value);
			return NO_ERROR;
		break;

		case REG_BUFF_UPDATE:
			GlobalVal->BUFF_UPDATE = (uint16_t)(Value);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister16(REG_BUFF_UPDATE, (uint16_t)Value);
			return NO_ERROR;
		break;

		case REG_ENC_RES:
			GlobalVal->ENC_RES = (uint16_t)(Value);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister16(REG_ENC_RES, (uint16_t)Value);
			return NO_ERROR;
		break;

		case REG_PID_V_KP:
			GlobalVal->PID_V_KP = (float)(Value/1000.0);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister32(REG_PID_V_KP, Value);
			return NO_ERROR;
		break;

		case REG_PID_V_KI:
			GlobalVal->PID_V_KI = (float)(Value/1000.0);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister32(REG_PID_V_KI, Value);
			return NO_ERROR;
		break;

		case REG_PID_V_KD:
			GlobalVal->PID_V_KD= (float)(Value/1000.0);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister32(REG_PID_V_KD, Value);
			return NO_ERROR;
		break;

		case REG_PID_P_KP:
			GlobalVal->PID_P_KP = (float)(Value/1000.0);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister32(REG_PID_P_KP, Value);
			return NO_ERROR;
		break;

		case REG_PID_P_KI:
			GlobalVal->PID_P_KI = (float)(Value/1000.0);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister32(REG_PID_P_KI, Value);
			return NO_ERROR;
		break;

		case REG_PID_P_KD:
			GlobalVal->PID_P_KD = (float)(Value/1000.0);
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister32(REG_PID_P_KD, Value);
			return NO_ERROR;
		break;

		case REG_GPIO_OUT:
			SetGPIO_Output((uint8_t)(Value));
			return NO_ERROR;
		break;

		case REG_STAT_ENDSTOP1:
			GlobalVal->STAT_ENDSTOP1 = (uint8_t)Value;
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister8(REG_STAT_ENDSTOP1, (uint8_t)Value);
			return NO_ERROR;
		break;

		case REG_STAT_ENDSTOP2:
			GlobalVal->STAT_ENDSTOP2 = (uint8_t)Value;
			/* Scrivo il valore anche nella eeprom */
			EEPROM_SetRegister8(REG_STAT_ENDSTOP2, (uint8_t)Value);
			return NO_ERROR;
		break;

		/* registro non esistente o non scrivibile */
		default:
			return ERROR_NOWRITE;
		break;
	}
}

/**
 * @}
 */

/**
 * @}
 */
