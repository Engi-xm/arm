Library folder for STM32 microcontroller.
A generic library should contain the following files from CUBE libraries:

1. cmsis_gcc.h
2. core_c<CORTEX>.h
3. core_cmFunc.h
4. core_cmInstr.h
5. startup_<MCU_D>.s
6. <MCU>_FLASH.ld
7. <MCU_D>.h
8. <MCU_F>.h
9. system_<MCU_F>.h
10. system_<MCU_F>.c

Here:
<CORTEX> - cortex version
<MCU_D> - microcontroller name as defined in main family header
<MCU> - full microcontroller name
<MCU_F> - microcontroller family name

Additional files:

1. Makefile
2. README
