/*
 * GENERATED FILE - DO NOT EDIT
 * (c) Code Red Technologies Ltd, 2008-2013
 * (c) NXP Semiconductors 2013-2017
 * Generated linker script file for MKW41Z512xxx4
 * Created from memory.ldt by FMCreateLinkMemory
 * Using Freemarker v2.3.23
 * MCUXpresso IDE v10.0.0 [Build 344] [2017-03-21]  on 2017-6-29 15:03:47
 */

MEMORY
{
  /* Define each memory region */
  PROGRAM_FLASH (rx) : ORIGIN = 0x0, LENGTH = 0x80000 /* 512K bytes (alias Flash) */  
  SRAM (rwx) : ORIGIN = 0x1fff8000, LENGTH = 0x20000 /* 128K bytes (alias RAM) */  
}

  /* Define a symbol for the top of each memory region */
  __base_PROGRAM_FLASH = 0x0  ; /* PROGRAM_FLASH */  
  __base_Flash = 0x0 ; /* Flash */  
  __top_PROGRAM_FLASH = 0x0 + 0x80000 ; /* 512K bytes */  
  __top_Flash = 0x0 + 0x80000 ; /* 512K bytes */  
  __base_SRAM = 0x1fff8000  ; /* SRAM */  
  __base_RAM = 0x1fff8000 ; /* RAM */  
  __top_SRAM = 0x1fff8000 + 0x20000 ; /* 128K bytes */  
  __top_RAM = 0x1fff8000 + 0x20000 ; /* 128K bytes */  
