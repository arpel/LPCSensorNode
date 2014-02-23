/*
 * GENERATED FILE - DO NOT EDIT
 * (c) Code Red Technologies Ltd, 2008-2014
 * (c) NXP Semiconductors 2013-2014
 * Linker script memory definitions
 * Created from LinkMemoryTemplate
 * By LPCXpresso v6.1.4 [Build 194] [2014-01-15]  on Sun Feb 23 11:12:15 CET 2014)
*/

MEMORY
{
  /* Define each memory region */
  MFlash16 (rx) : ORIGIN = 0x0, LENGTH = 0x4000 /* 16K bytes */
  RamLoc4 (rwx) : ORIGIN = 0x10000000, LENGTH = 0x1000 /* 4K bytes */


}
  /* Define a symbol for the top of each memory region */
  __top_MFlash16 = 0x0 + 0x4000;
  __top_RamLoc4 = 0x10000000 + 0x1000;



