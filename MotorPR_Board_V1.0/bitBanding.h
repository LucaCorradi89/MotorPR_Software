/**********************************************************************
* $Id$		bitBanding.h		2013-11-24
*//**
* @file		bitBanding.h
* @brief	Implementazione del bitbanding
* @version	0.1
* @date		24. 11. 2013
* @author	Codilupi Riccardo
*
*
***********************************************************************/
/** @defgroup bitBanding Utility bitBanding
 * @{
 */

#ifndef BITBANDING_H_
#define BITBANDING_H_

#include <stdint.h>
/**
 * @brief	Namespace che contiene le funzioni per la gestione del bitband
 */
namespace bitBand {
/*********************************************************************//**
 * @brief 		Funzione per resettare il valore di un bit
 **********************************************************************/
static inline void clearBit(volatile uint32_t* const dataAddress, uint8_t bitNumber) {
	uint32_t aliasAddress = reinterpret_cast<uint32_t>(dataAddress);
	volatile uint32_t* aliasRegister;

	// Controlla la posizione del registro
	if (aliasAddress >= 0x40000000) {
		// Periferiche
		aliasAddress = 0x42000000 + ((aliasAddress - 0x40000000) << 5) + (bitNumber << 2);
	} else if (aliasAddress >= 0x20000000) {
		// SRAM
		aliasAddress = 0x22000000 + ((aliasAddress - 0x20000000) << 5) + (bitNumber << 2);
	} else {
		// Nessun bitbanding
		*dataAddress &= (1UL << bitNumber);
		return;
	}

	aliasRegister = reinterpret_cast<volatile uint32_t*>(aliasAddress);
	*aliasRegister = 0;
}

/*********************************************************************//**
 * @brief 		Funzione per settare il valore di un bit
 **********************************************************************/
static inline void setBit(volatile const uint32_t* const dataAddress, const uint8_t bitNumber) {
	uint32_t aliasAddress = reinterpret_cast<uint32_t>(dataAddress);
	volatile uint32_t* aliasRegister;

	// Controlla la posizione del registro
	if (aliasAddress >= 0x40000000) {
		// Periferiche
		aliasAddress = 0x42000000 + ((aliasAddress - 0x40000000) << 5) + (bitNumber << 2);
	} else /*if (aliasAddress >= 0x20000000)*/ {
		// SRAM
		aliasAddress = 0x22000000 + ((aliasAddress - 0x20000000) << 5) + (bitNumber << 2);
	} /*else {
		// Nessun bitbanding
		*dataAddress |= (1UL << bitNumber);
		return;
	}*/

	aliasRegister = reinterpret_cast<volatile uint32_t*>(aliasAddress);
	*aliasRegister = 1;
}

/*********************************************************************//**
 * @brief 		Funzione per ottenere il valore di un bit
 **********************************************************************/
static inline uint32_t getBit(const uint32_t* const dataAddress, const uint8_t bitNumber) {
	uint32_t aliasAddress = reinterpret_cast<uint32_t>(dataAddress);
	const volatile uint32_t* aliasRegister;

	// Controlla la posizione del registro
	if (aliasAddress >= 0x40000000) {
		// Periferiche
		aliasAddress = 0x42000000 + ((aliasAddress - 0x40000000) << 5) + (bitNumber << 2);
	} else if (aliasAddress >= 0x20000000) {
		// SRAM
		aliasAddress = 0x22000000 + ((aliasAddress - 0x20000000) << 5) + (bitNumber << 2);
	} else {
		// Nessun bitbanding
		return ((*dataAddress >> bitNumber) & 1);
	}

	aliasRegister = reinterpret_cast<volatile uint32_t*>(aliasAddress);
	return *aliasRegister;
}

/*********************************************************************//**
 * @brief 		Funzione per ottenere il puntatore all'alias di un bit
 **********************************************************************/
static inline uint32_t* getBitAlias(const uint32_t* const dataAddress, const uint8_t bitNumber) {
	uint32_t aliasAddress = reinterpret_cast<uint32_t>(dataAddress);
	const volatile uint32_t* aliasRegister;

	// Controlla la posizione del registro
	if (aliasAddress >= 0x40000000) {
		// Periferiche
		aliasAddress = 0x42000000 + ((aliasAddress - 0x40000000) << 5) + (bitNumber << 2);
	} else if (aliasAddress >= 0x20000000) {
		// SRAM
		aliasAddress = 0x22000000 + ((aliasAddress - 0x20000000) << 5) + (bitNumber << 2);
	} else {
		aliasAddress = 0;
	}
	return reinterpret_cast<uint32_t*>(aliasAddress);
}


} // namespace bitBand
#endif /* BITBANDING_H_ */

/** @} bitBanding */
