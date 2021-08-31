// ******************************************************************************************************
//    Filename: brd_BOKRA_LPC824_Lite.h
// 	   Version:	1.0
//  Created on: 07.12.2019
//		  Desc: some as board support file for BOKRA LPC824 Lite - header file
//				includes board (HW) specific functions
// ******************************************************************************************************
//#include "project.h"



#ifndef __BRD_BOKRA_LPC824_LITE_H_
#define __BRD_BOKRA_LPC824_LITE_H_


// ------------------------------------------------------------------------------------------------------
// Board hardware connection
// ------------------------------------------------------------------------------------------------------
#define	BRD_LED_GREEN_PORTPIN			19											// Pin for LED Green. Port always:0
#define	BRD_LED_YELLOW_PORTPIN			18											// Pin for LED Yellow. Port always:0

// temporary:
#define BRD_nLEDRST_PORTPIN				12											// LED RST

// ------------------------------------------------------------------------------------------------------
// User configuration
// ------------------------------------------------------------------------------------------------------
// Systick: CFG_SYSTICKRATE_HZ
// UART User Config: CFG_UART_PTRPERI, CFG_UART_BAUDRATE, CFG_UART_PARITY, CFG_UART_DATABITS, CFG_UART_STOPBITS
// I2C User Config:  CFG_I2C_SPEEDKHZ




extern void Board_Init(void);														// called after reset, or as users action

// ------------------------------------------------------------------------------------------------------
// Board MCU IO Pins Initialization - init, configure, SWM remap,...
// ------------------------------------------------------------------------------------------------------
static inline void Board_MCU_Pin_Init(void)
{
	Chip_GPIO_Init (LPC_GPIO_PORT);													// Init GPIO - run CLK also
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);									// Enable CLK for SWM Block
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);								// Enable CLK for IOCON Block
	
	// ToDo - insert code: Select default value for SWM! SWM is reset after PUR!
	
	// Configure UART0-TX pin
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 4, 1);									// deactivate pin
	Chip_SWM_MovablePinAssign( SWM_U0_TXD_O, 4);									// SWM move TX0 to P0.4
	// Configure UART0-RX pin
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 0, 1);									// deactivate pin
	Chip_SWM_MovablePinAssign( SWM_U0_RXD_I, 0);									// SWM move RX0 to P0.0
	Chip_IOCON_PinSetMode(LPC_IOCON, IOCON_PIO(0), PIN_MODE_PULLUP);				// Enable Pull Up to Rx
	
	// Configure SWDIO pin
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 2, 1);									// deactivate pin
	Chip_SWM_FixedPinEnable( SWM_FIXED_SWDIO, true);								// SWM enable fixed function SWDIO - P0.2
	// Configure SWDCLK pin
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 3, 1);									// deactivate pin
	Chip_SWM_FixedPinEnable( SWM_FIXED_SWCLK, true);								// SWM enable fixed function SWDCLK - P0.3

	// Configure Reset pin
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 5, 1);									// deactivate pin
	Chip_SWM_FixedPinEnable( SWM_FIXED_RST, true);									// SWM enable fixed function Reset - P0.5
	// Configure XTALIN pin
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 8, 1);									// deactivate pin
	Chip_SWM_FixedPinEnable( SWM_FIXED_XTALIN, true);								// SWM enable fixed function XTALIN - P0.8
	Chip_IOCON_PinSetMode( LPC_IOCON, IOCON_PIO8, PIN_MODE_INACTIVE);
	
	// Configure XTALOUT pin
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 9, 1);									// deactivate pin
	Chip_SWM_FixedPinEnable( SWM_FIXED_XTALOUT, true);								// SWM enable fixed function XTALOUT - P0.9
	Chip_IOCON_PinSetMode( LPC_IOCON, IOCON_PIO9, PIN_MODE_INACTIVE);

	// Configure I2C-SDA pin
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 11, 1);									// deactivate pin
	Chip_SWM_FixedPinEnable( SWM_FIXED_I2C0_SDA, true);								// SWM enable fixed function I2C_SDA - P0.11
	Chip_IOCON_PinSetI2CMode(LPC_IOCON, IOCON_PIO(11), PIN_I2CMODE_STDFAST);		// set I2C Pin mode to Standard speed - 400kHz
	
	// Configure I2C-SCL pin
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 10, 1);									// deactivate pin
	Chip_SWM_FixedPinEnable( SWM_FIXED_I2C0_SCL, true);								// SWM enable fixed function I2C_SCL - P0.10
	Chip_IOCON_PinSetI2CMode(LPC_IOCON, IOCON_PIO(10), PIN_I2CMODE_STDFAST);		// set I2C Pin mode to Standard speed - 400kHz

	// Configure LED GREEN - GPIO0.19 pin
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, BRD_LED_GREEN_PORTPIN, 0);				// deactivate pin
	Chip_GPIO_SetPinDIR( LPC_GPIO_PORT, 0, BRD_LED_GREEN_PORTPIN, IO_OUTPUT);		// set pin direction
	Chip_IOCON_PinSetOpenDrainMode(LPC_IOCON, IOCON_PIO(BRD_LED_GREEN_PORTPIN), true); 	// set mode as opendrain
	
	// Configure LED YELLOW - GPIO0.18 pin
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, BRD_LED_YELLOW_PORTPIN, 0);				// deactivate pin
	Chip_GPIO_SetPinDIR( LPC_GPIO_PORT, 0, BRD_LED_YELLOW_PORTPIN, IO_OUTPUT);		// set pin direction
	Chip_IOCON_PinSetOpenDrainMode(LPC_IOCON, IOCON_PIO(BRD_LED_YELLOW_PORTPIN), true);	// set mode as opendrain
	
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);								// Disable CLK for SWM Block
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_IOCON);								// Disable CLK for IOCON Block
}	

#endif // __BRD_BOKRA_LPC824_LITE_H_
