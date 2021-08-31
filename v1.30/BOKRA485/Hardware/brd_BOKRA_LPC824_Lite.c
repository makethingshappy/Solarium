// ******************************************************************************************************
//    Filename: brd_BOKRA_LPC824_Lite.c
// 	   Version:	1.0
//  Created on: 07.12.2019
//		  Desc: some as board support file for BOKRA LPC824 Lite
//				includes board (HW) specific functions depends on board PCB connections
// ******************************************************************************************************
#include "project.h"


// Used MCU on board: LPC824M201JHI33

/* System oscillator rate and clock rate on the CLKIN pin */
const uint32_t OscRateIn = 12000000;
const uint32_t ExtRateIn = 0;

// ------------------------------------------------------------------------------------------------------
// Board UART Initialise
// ------------------------------------------------------------------------------------------------------
void Board_UART_Init(void)
{
	Chip_UART_Init(CFG_UART_PTRPERI);												// Enable periph. clock and reset it
	
	// Set UART comm parameters:
	Chip_UART_ConfigData(CFG_UART_PTRPERI, CFG_UART_PARITY | CFG_UART_DATABITS | CFG_UART_STOPBITS);	// configure
	
	Chip_Clock_SetUSARTNBaseClockRate( CFG_UART_BAUDRATE * 8, true);				// *16 is oversample BaudRate

	Chip_UART_SetBaud (CFG_UART_PTRPERI, CFG_UART_BAUDRATE);						// And write new BaudRate

	Chip_UART_TXEnable(CFG_UART_PTRPERI);											// Enable Transmit
	Chip_UART_Enable(CFG_UART_PTRPERI);												// Enable UART
}


// ------------------------------------------------------------------------------------------------------
// Board I2C Initialise
// ------------------------------------------------------------------------------------------------------
void Board_I2C_Init (void)
{
	uint8_t scl;
	uint32_t CLKDiv;
	// initialization code here:

	Chip_I2C_Init(CFG_I2C_PTRPERI);													// Enable periph. clock and reset it

	CLKDiv = 1;																		// start at divider = 1
	do
	{	
		Chip_I2C_SetClockDiv(CFG_I2C_PTRPERI, CLKDiv);								// Setup clock divider for I2C
		CLKDiv = Chip_I2C_GetClockDiv(CFG_I2C_PTRPERI);								// read back
		scl= Chip_Clock_GetSystemClockRate() / ( CLKDiv * (CFG_I2C_SPEEDKHZ * 1000));
		CLKDiv ++;
		if(CLKDiv > 0x10000) return;
	}while((scl < 1) || (scl > 8));													// recalculate while scl is out of range
		
	Chip_I2CM_SetBusSpeed(CFG_I2C_PTRPERI, CFG_I2C_SPEEDKHZ);						// Setup I2C Bus Speed
	
	NVIC_EnableIRQ(CFG_I2C_IRQNUM);													// Enable I2C Interrupt
	Chip_I2CM_Enable(CFG_I2C_PTRPERI);												// Enable I2C Master Mode
}


// ------------------------------------------------------------------------------------------------------
// Board CLOCK Initialise
// ------------------------------------------------------------------------------------------------------
void Board_Clock_Init(void)
{
	
	Chip_Clock_SetPLLBypass(false, false);											// EXT oscillator < 15MHz
	
    Chip_SYSCTL_PowerUp(SYSCTL_SLPWAKE_SYSOSC_PD);   								// Power Up external system oscillator
	Chip_SYSCTL_PowerUp(SYSCTL_SLPWAKE_SYSPLL_PD);   								// Power Up system PLL
	
    for (volatile int i = 0; i < 1000; i++) { }										// wait for stabilization

	// external XTAL = 12 MHz:
	//Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_SYSOSC);							// set XTAL as PLL clock Source - NOT WORKING !!!!!
	
	// Internal RC:
	Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_IRC);							// set IRC as PLL clock Source

	Chip_FMC_SetFLASHAccess(FLASHTIM_30MHZ_CPU);									// Setup FLASH access to 2 clocks (up to 30MHz)
	
	Chip_Clock_SetupSystemPLL(4, 1);	

	while (!Chip_Clock_IsSystemPLLLocked()) {}										// Wait for PLL to lock

	Chip_Clock_SetSysClockDiv(2);													// System Clock - max. 30MHz
		
	Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT); 						// Main clock as PLL Out
		
	SystemCoreClockUpdate();
}


// ------------------------------------------------------------------------------------------------------
// Board Initialization
// ------------------------------------------------------------------------------------------------------
void Board_Init(void)
{
	Board_MCU_Pin_Init();															// init and configure MCU IO pins
	
	Board_Clock_Init();
	
	// ************ SysTick
	App.SysClock = Chip_Clock_GetSystemClockRate();
	App.MainClock = Chip_Clock_GetMainClockRate();
	SysTick_Config(App.SysClock / CFG_SYSTICKRATE_HZ);								// Enable systick timer

	
	// ************ UART
	Board_UART_Init();																// Call Init
		
	
	// ************ I2C
	Board_I2C_Init();																// Call Init
}
