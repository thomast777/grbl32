
// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* \li File:               eeprom.c
* \li Compiler:           IAR EWAAVR 3.10c
* \li Support mail:       avr@atmel.com
*
* \li Supported devices:  All devices with split EEPROM erase/write
*                         capabilities can be used.
*                         The example is written for ATmega48.
*
* \li AppNote:            AVR103 - Using the EEPROM Programming Modes.
*
* \li Description:        Example on how to use the split EEPROM erase/write
*                         capabilities in e.g. ATmega48. All EEPROM
*                         programming modes are tested, i.e. Erase+Write,
*                         Erase-only and Write-only.
*
*                         $Revision: 1.6 $
*                         $Date: Friday, February 11, 2005 07:16:44 UTC $
*
****************************************************************************/
/*
 *	  Copyright (c) 2018-2019 Thomas Truong
 *	  										EEPROM Simulation using Flash on STM32F103 and STM32F407
 *
 *
 *
 *
 */


#ifdef STM32
	#include <string.h>
  #include "stm32eeprom.h"
  #include "settings.h"
	unsigned char EE_Buffer[PAGE_SIZE];
#elif ATMEGA328P
  #include <avr/io.h>
  #include <avr/interrupt.h>

  /* These EEPROM bits have different names on different devices. */
  #ifndef EEPE
      #define EEPE  EEWE  //!< EEPROM program/write enable.
      #define EEMPE EEMWE //!< EEPROM master program/write enable.
  #endif

  /* These two are unfortunately not defined in the device include files. */
  #define EEPM1 5 //!< EEPROM Programming Mode Bit 1.
  #define EEPM0 4 //!< EEPROM Programming Mode Bit 0.

  /* Define to reduce code size. */
  #define EEPROM_IGNORE_SELFPROG //!< Remove SPM flag polling.
#endif


#ifdef STM32
void eeprom_flush()
{
  uint32_t nAddress = EEPROM_START_ADDRESS;
  uint16_t *pBuffer = (uint16_t *)EE_Buffer;
  uint16_t nSize = PAGE_SIZE;

  HAL_FLASH_Unlock();

#ifdef STM32F1
  //__HAL_FLASH_CLEAR_FLAG (FLASH_FLAG_EOP | FLASH_FLAG_WRPERR FLASH | FLASH_FLAG_PGERR  FLASH | FLASH_FLAG_OPTVERR);
  FLASH_PageErase(EEPROM_START_ADDRESS);
  FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE); //-- !!
  CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
#endif
#ifdef STM32F4
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
  FLASH_Erase_Sector(EEPROM_START_SECTOR, VOLTAGE_RANGE_3);
  FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
  CLEAR_BIT(FLASH->CR, (FLASH_CR_SER | FLASH_CR_SNB));
#endif

  while (nSize > 0)
  {
    if (*pBuffer != 0xffff)
    {
    	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, nAddress, *pBuffer++);
    }
    else
    {
      pBuffer++;
    }
    if (*pBuffer != 0xffff)
    {
    	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, nAddress+2, *pBuffer++);
    }
    else
    {
      pBuffer++;
    }
    nSize -= 4;
    nAddress += 4;
  }

  HAL_FLASH_Lock();

}

void eeprom_init()
{
  uint16_t VarIdx = 0;
  uint8_t *pTmp = EE_Buffer;

  for (VarIdx = 0; VarIdx < PAGE_SIZE; VarIdx++)
  {
    *pTmp++ = (*(__IO uint8_t*)(EEPROM_START_ADDRESS + VarIdx));
  }

  if (EE_Buffer[0] != SETTINGS_VERSION)
  {
    pTmp = EE_Buffer;

    for (VarIdx = 0; VarIdx < PAGE_SIZE; VarIdx++)
    {
      *pTmp++ = 0xFF;
    }
  }
}
#endif

/*! \brief  Read byte from EEPROM.
 *
 *  This function reads one byte from a given EEPROM address.
 *
 *  \note  The CPU is halted for 4 clock cycles during EEPROM read.
 *
 *  \param  addr  EEPROM address to read from.
 *  \return  The byte read from the EEPROM address.
 */
unsigned char eeprom_get_char( unsigned int addr )
{
#ifdef STM32
  return EE_Buffer[addr];
#elif ATMEGA328P
	do {} while( EECR & (1<<EEPE) ); // Wait for completion of previous write.
	EEAR = addr; // Set EEPROM address register.
	EECR = (1<<EERE); // Start EEPROM read operation.
	return EEDR; // Return the byte read from EEPROM.
#endif
}

/*! \brief  Write byte to EEPROM.
 *
 *  This function writes one byte to a given EEPROM address.
 *  The differences between the existing byte and the new value is used
 *  to select the most efficient EEPROM programming mode.
 *
 *  \note  The CPU is halted for 2 clock cycles during EEPROM programming.
 *
 *  \note  When this function returns, the new EEPROM value is not available
 *         until the EEPROM programming time has passed. The EEPE bit in EECR
 *         should be polled to check whether the programming is finished.
 *
 *  \note  The EEPROM_GetChar() function checks the EEPE bit automatically.
 *
 *  \param  addr  EEPROM address to write to.
 *  \param  new_value  New EEPROM value.
 */
void eeprom_put_char( unsigned int addr, unsigned char new_value )
{
#ifdef STM32
  EE_Buffer[addr] = new_value;
#elif ATMEGA328P

	char old_value; // Old EEPROM value.
	char diff_mask; // Difference mask, i.e. old value XOR new value.

	cli(); // Ensure atomic operation for the write operation.
	
	do {} while( EECR & (1<<EEPE) ); // Wait for completion of previous write.
	#ifndef EEPROM_IGNORE_SELFPROG
	do {} while( SPMCSR & (1<<SELFPRGEN) ); // Wait for completion of SPM.
	#endif
	
	EEAR = addr; // Set EEPROM address register.
	EECR = (1<<EERE); // Start EEPROM read operation.
	old_value = EEDR; // Get old EEPROM value.
	diff_mask = old_value ^ new_value; // Get bit differences.
	
	// Check if any bits are changed to '1' in the new value.
	if( diff_mask & new_value ) {
		// Now we know that _some_ bits need to be erased to '1'.
		
		// Check if any bits in the new value are '0'.
		if( new_value != 0xff ) {
			// Now we know that some bits need to be programmed to '0' also.
			
			EEDR = new_value; // Set EEPROM data register.
			EECR = (1<<EEMPE) | // Set Master Write Enable bit...
			       (0<<EEPM1) | (0<<EEPM0); // ...and Erase+Write mode.
			EECR |= (1<<EEPE);  // Start Erase+Write operation.
		} else {
			// Now we know that all bits should be erased.

			EECR = (1<<EEMPE) | // Set Master Write Enable bit...
			       (1<<EEPM0);  // ...and Erase-only mode.
			EECR |= (1<<EEPE);  // Start Erase-only operation.
		}
	} else {
		// Now we know that _no_ bits need to be erased to '1'.
		
		// Check if any bits are changed from '1' in the old value.
		if( diff_mask ) {
			// Now we know that _some_ bits need to the programmed to '0'.
			
			EEDR = new_value;   // Set EEPROM data register.
			EECR = (1<<EEMPE) | // Set Master Write Enable bit...
			       (1<<EEPM1);  // ...and Write-only mode.
			EECR |= (1<<EEPE);  // Start Write-only operation.
		}
	}
	
	sei(); // Restore interrupt flag state.
#endif
}

// Extensions added as part of Grbl 


void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
  unsigned char checksum = 0;
  for(; size > 0; size--) { 
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += *source;
    eeprom_put_char(destination++, *(source++)); 
  }
  eeprom_put_char(destination, checksum);

#ifdef STM32
  eeprom_flush();
#endif

}

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
  unsigned char data, checksum = 0;
  for(; size > 0; size--) { 
    data = eeprom_get_char(source++);
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += data;    
    *(destination++) = data; 
  }
  return(checksum == eeprom_get_char(source));
}

// end of file
