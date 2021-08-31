// ******************************************************************************************************
//    Filename: main.c
// 	   Version:	1.0
//  Created on: 03.12.2019
//		  Desc: obviously main c file... 
// ******************************************************************************************************
#include "project.h"

App_t									App;

// ------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------
// APP helpers
// ------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------
static uint8_t rxbuff[CFG_UART_RB_SIZE], txbuff[CFG_UART_RB_SIZE];					// UART Transmit and receive buffers
static I2CM_XFER_T 						I2CXfer;									// I2C Xfer struct

#define LED_GREEN_ON_TIME				{App.GreenLed_Alive_Count = 100; Chip_GPIO_WritePortBit( LPC_GPIO_PORT, 0, BRD_LED_GREEN_PORTPIN, true );}
#define LED_YELLOW_ON_TIME				{App.YellowLed_Alive_Count = 100; Chip_GPIO_WritePortBit( LPC_GPIO_PORT, 0, BRD_LED_YELLOW_PORTPIN, true );}
#define LED_GREEN_OFF					{Chip_GPIO_WritePortBit( LPC_GPIO_PORT, 0, BRD_LED_GREEN_PORTPIN, false ); App.GreenLed_Off = false;}
#define LED_YELLOW_OFF					{Chip_GPIO_WritePortBit( LPC_GPIO_PORT, 0, BRD_LED_YELLOW_PORTPIN, false ); App.YellowLed_Off = false;}


// ------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------
// Application's support functions 
// ------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------------------
// Function to wait for I2CM transfer completion
// ------------------------------------------------------------------------------------------------------
static void WaitForI2cXferComplete(I2CM_XFER_T *xferRec)
{
	while ((xferRec->status == I2CM_STATUS_BUSY) || (xferRec->txSz) || (xferRec->rxSz))	// Test for still transferring data
	{}
}

// ------------------------------------------------------------------------------------------------------
// Write 1 byte into MCP23008
// ------------------------------------------------------------------------------------------------------
void MCP23008_Write8(uint8_t RegAdr, uint8_t RegValue)
{
	static uint8_t tmpTxBuff[2];
	
//	Chip_GPIO_WritePortBit( LPC_GPIO_PORT, 0, BRD_LED_GREEN_PORTPIN, true );		// turn ON indicator
	
	I2CXfer.slaveAddr = CFG_I2C_MCP23008_ADR >> 1;									// SET I2C Slave address
	tmpTxBuff[0] = RegAdr;															// select dst register
	tmpTxBuff[1] = RegValue;														// select register value
	I2CXfer.txBuff = &tmpTxBuff[0];													// select dst register
	I2CXfer.txSz = 2;																// write 2 bytes
	I2CXfer.rxSz = 0;																// we will not read
	I2CXfer.status = 0;
	
	Chip_I2CM_Disable(CFG_I2C_PTRPERI);
	Chip_I2CM_Enable(CFG_I2C_PTRPERI);	
	Chip_I2CM_Xfer(CFG_I2C_PTRPERI, &I2CXfer);
	Chip_I2C_EnableInt(CFG_I2C_PTRPERI, I2C_INTENSET_MSTPENDING | I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR | I2C_INTENSET_EVENTTIMEOUT);
	WaitForI2cXferComplete(&I2CXfer);
	StopWatch_DelayTicks(100);
	Chip_I2C_DisableInt(CFG_I2C_PTRPERI, I2C_INTENSET_MSTPENDING | I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR | I2C_INTENSET_EVENTTIMEOUT);
	
//	Chip_I2CM_XferBlocking(CFG_I2C_PTRPERI, &I2CXfer);

//	Chip_GPIO_WritePortBit( LPC_GPIO_PORT, 0, BRD_LED_GREEN_PORTPIN, false );		// turn OFF indicator
}

// ------------------------------------------------------------------------------------------------------
// Read 1 byte from MCP23008
// ------------------------------------------------------------------------------------------------------
uint8_t MCP23008_Read8(uint8_t RegAdr)
{
	uint8_t tmpRxBuff;
	
//	Chip_GPIO_WritePortBit( LPC_GPIO_PORT, 0, BRD_LED_GREEN_PORTPIN, true );		// turn ON indicator

	I2CXfer.slaveAddr = CFG_I2C_MCP23008_ADR >> 1;									// SET I2C Slave address
	I2CXfer.txBuff = &RegAdr;														// select dst register
	I2CXfer.txSz = 1;																// write 1 bytes
	I2CXfer.rxSz = 1;																// we will read 1 byte
	I2CXfer.status = 0;

	Chip_I2CM_Disable(CFG_I2C_PTRPERI);
	Chip_I2CM_Enable(CFG_I2C_PTRPERI);	

	I2CXfer.rxBuff = &tmpRxBuff;													// select local destination for read

	Chip_I2CM_Xfer(CFG_I2C_PTRPERI, &I2CXfer);
	Chip_I2C_EnableInt(CFG_I2C_PTRPERI, I2C_INTENSET_MSTPENDING | I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR | I2C_INTENSET_EVENTTIMEOUT);
	WaitForI2cXferComplete(&I2CXfer);
	StopWatch_DelayTicks(100);
	Chip_I2C_DisableInt(CFG_I2C_PTRPERI, I2C_INTENSET_MSTPENDING | I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR | I2C_INTENSET_EVENTTIMEOUT);
	
//	Chip_I2CM_XferBlocking(CFG_I2C_PTRPERI, &I2CXfer);
	
//	Chip_GPIO_WritePortBit( LPC_GPIO_PORT, 0, BRD_LED_GREEN_PORTPIN, false );		// turn OFF indicator

	return(tmpRxBuff);
}


// ------------------------------------------------------------------------------------------------------
// MCP23008 Initialization do default application configuration
// ------------------------------------------------------------------------------------------------------
void MCP23008_Init(void)
{
//	uint16_t arr[12];
//	for(uint8_t i=0; i<12; i++)
//	{
//		MCP23008_Write8(i, 255);
//	}
//	
//	for(uint8_t i=0; i<12; i++)
//	{
//		arr[i] = MCP23008_Read8(i);
//	}	
//	for(uint8_t i=0; i<12; i++)
//	{
//		arr[i] = MCP23008_Read8(i);
//	}	

	// Set array to regs values:
	MCP23008_Write8 (CFG_I2C_MCP23008_REG_IOCON, 0x20);								// Firstly, Disable sequentional access!
	MCP23008_Write8 (CFG_I2C_MCP23008_REG_IODIR, 0x03);								// G0-GP1 as Input, GP4-GP5 as Output
	MCP23008_Write8 (CFG_I2C_MCP23008_REG_IPOL, 0x00);								// normal polarity for all Inp. : 1 as active (power present on physical input terminal)
	MCP23008_Write8 (CFG_I2C_MCP23008_REG_GPINTEN, 0x00);							// Interrupt is not used
	MCP23008_Write8 (CFG_I2C_MCP23008_REG_DEFVAL, 0x00);							// Interrupt is not used
	MCP23008_Write8 (CFG_I2C_MCP23008_REG_INTCON, 0x00);							// Interrupt is not used
	MCP23008_Write8 (CFG_I2C_MCP23008_REG_GPPU, 0x00);								// PullUp disabled
	MCP23008_Write8 (CFG_I2C_MCP23008_REG_INTCAP, 0x00);							// Interrupt Capture - R/O
	MCP23008_Write8 (CFG_I2C_MCP23008_REG_GPIO, 0x00);								// GPIO set to inactive
	MCP23008_Write8 (CFG_I2C_MCP23008_REG_OLAT, 0x00);								// Output inactive
}











// ------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------
// App flows
// ------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------


// ------------------------------------------------------------------------------------------------------
// Read I2C MCP23008 Inputs and arrange it into Logic structure
// ------------------------------------------------------------------------------------------------------
void Reload_Inputs(void)
{
	uint8_t tmpInp = MCP23008_Read8(CFG_I2C_MCP23008_REG_GPIO) & 0x0f;				// Read I2C Inputs and wait for result
	
	// In this FW is useable only channel 1 and 2. But I read also 3 and 4 :-)
	
	App.Logic[0].Input = 0;
	App.Logic[1].Input = (tmpInp & (1 << 0))? 1: 0;
	App.Logic[2].Input = (tmpInp & (1 << 1))? 1: 0;
	App.Logic[3].Input = (tmpInp & (1 << 2))? 1: 0;
	App.Logic[4].Input = (tmpInp & (1 << 3))? 1: 0;
}

// ------------------------------------------------------------------------------------------------------
// Write into I2C MCP23008 Outputs from Logic structure
// ------------------------------------------------------------------------------------------------------
void Rewrite_Outputs(void)
{
	uint8_t tmpOut = 0;
	
	// In this FW is useable only channel 1 and 2. But I write also 3 and 4 :-)
	
	tmpOut |=  (App.Logic[1].Output & 0x01) << 0;
	tmpOut |=  (App.Logic[2].Output & 0x01) << 1;
	tmpOut |=  (App.Logic[3].Output & 0x01) << 2;
	tmpOut |=  (App.Logic[4].Output & 0x01) << 3;
	tmpOut = tmpOut << 4;															// Output used only on four upper bits
	
	MCP23008_Write8( CFG_I2C_MCP23008_REG_OLAT, tmpOut);							// Write into MCP23008
}
						
// ------------------------------------------------------------------------------------------------------
// Decode RS485 Commands. ON, OFF, ASK, ADI, ADO, SRT
// ------------------------------------------------------------------------------------------------------
void Decode_RxCMD(void)
{
	uint32_t olen; //, cmds = 0;
	char tmpBuff[16];
	char *rxCMD, *rxChannel, *rxParm;
	uint32_t Channel, Parm, Response;
	char *p;
	
	rxCMD = strtok((char *) &App.RxCMD[0]," ,\0");									// return first token till space,','or '\0'
	rxChannel = strtok( NULL, " ,\0");												// return next token
	rxParm = strtok( NULL, " ,\0");													// aaaaand return last token
	Channel = strtoul(rxChannel, &p, 10);											// return channel as integer
	Parm = strtoul(rxParm, &p, 10);													// return parameter as integer
	
	// Turn On Output Channel
	// COMMAND "ON xx,ttt" ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	if(strncmp((char *) rxCMD, "ON", strlen("ON")) == 0)							// command found with parameters!
	{
		//LED_YELLOW_ON_TIME;															// YELLOW LED ON
		
		// if length > CMD (missing divider after CMD), or Time (Parm) is out of range, ignore telegram
		if((strlen(rxCMD) == strlen("ON")) || ((Parm < CFG_CMD_PARM_TIME_MAX) && (Parm >= CFG_CMD_PARM_TIME_MIN)))
		{
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, (const void*) & "RSP ", strlen("RSP "));	// send answer
			olen = sprintf( &tmpBuff[0], "%02d", Channel);							// convert Channel into strings with 2 chars
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, tmpBuff, olen);	// send string
		
			// check if channel is out of range
			if((Channel >= CFG_LOGIC_CHANNEL_COUNT) || \
			(App.Logic[Channel].Enabled == false)) Response = 2;					// and check active channel is enabled (CONST !)
			else																	// execute command and read response 
			{
				if(App.Logic[Channel].Output)										// Channel Activated ?
				{	
					Response = 1;													// channel was already activated
				}
				else
				{	
					App.Logic[Channel].PresetTime = Parm;							// Save time as PresetTime. In Main will be started
					App.Logic[Channel].RemoteEnable = true;							// Chennel enabled remotelly
					Response = 0;													// Return OK
				}
			}
			// prepare and send response
			olen = sprintf( &tmpBuff[0], ",%1d", Response);							// convert Channel into strings with 2 chars
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, tmpBuff, olen);	// send string
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, &(uint8_t *) {CFG_TXCMD_ENDCHAR}, 1);	// send END CHAR
		}
		memset(&App.RxCMD[0],0x00, CFG_RXCMD_MAXLEN);								// clear in array
		return;
	}

	// Turn Off Output Channel
	// COMMAND "OFF xx"	+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	if(strncmp((char *) rxCMD, "OFF", strlen("OFF")) == 0)							// command found with parameters!
	{
		//LED_YELLOW_ON_TIME;															// YELLOW LED ON	
		// if length > CMD (missing divider after CMD), ignore telegram
		if(strlen(rxCMD) == strlen("OFF"))
		{
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, (const void*) & "RSP ", strlen("RSP "));	// send answer
			olen = sprintf( &tmpBuff[0], "%02d", Channel);							// convert Channel into strings with 2 chars
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, tmpBuff, olen);	// send string
		
			// check if channel is out of range
			if((Channel >= CFG_LOGIC_CHANNEL_COUNT) || \
			(App.Logic[Channel].Enabled == false)) Response = 2;					// and check active channel is enabled (CONST !)
			else																	// execute command and read response 
			{
				if(App.Logic[Channel].RemoteEnable)									// Channel Enablked remotelly ?
				{	
					App.Logic[Channel].RemoteEnable = false;						// Chennel enabled remotelly
//					App.Logic[Channel].Output = 0;									// Deactivate channel Output relay
//					App.Write_Output = true;										// Request for update MCP23008 Output pins
					Response = 0;													// return OK
				}
				else Response = 1;													// channel was already deactivated
			}	
			// prepare and send response
			olen = sprintf( &tmpBuff[0], ",%1d", Response);							// convert Channel into strings with 2 chars
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, tmpBuff, olen);	// send string
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, &(uint8_t *) {CFG_TXCMD_ENDCHAR}, 1);	// send END CHAR
		}
		memset(&App.RxCMD[0],0x00, CFG_RXCMD_MAXLEN);								// clear in array
		return;
	}

	// Report Adapter Status
	// COMMAND "ASK" ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	if(strncmp((char *) rxCMD, "ASK", strlen("ASK")) == 0)							// command found 
	{
		//LED_YELLOW_ON_TIME;															// YELLOW LED ON
		// if length > CMD (missing divider after CMD), ignore telegram
		if(strlen(rxCMD) == strlen("ASK"))
		{
			App.Alive_CountDown = CFG_ALIVE_TIME;									// Reset CountDown Timer
			
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, (const void*) & "SOK", strlen("SOK"));// send answer
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, &(uint8_t *) {CFG_TXCMD_ENDCHAR}, 1);	// send END CHAR
		}
		memset(&App.RxCMD[0],0x00, CFG_RXCMD_MAXLEN);								// clear in array
		return;
	}	
	
	// Report Input Channel Status
	// COMMAND "ADI xx"	+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	if(strncmp((char *) rxCMD, "ADI", strlen("ADI")) == 0)							// command found with parameters!
	{
		//LED_YELLOW_ON_TIME;															// YELLOW LED ON
		// if length > CMD (missing divider after CMD), ignore telegram
		if(strlen(rxCMD) == strlen("ADI"))
		{
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, (const void*) & "SDI ", strlen("SDI "));	// send answer
			olen = sprintf( &tmpBuff[0], "%02d", Channel);							// convert Channel into strings with 2 chars
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, tmpBuff, olen);	// send string
		
			// check if channel is out of range
			if((Channel >= CFG_LOGIC_CHANNEL_COUNT) || \
			(App.Logic[Channel].Enabled == false)) Response = 2;					// and check active channel is enabled (CONST !)
			else																	// execute command and read response 
			{
				Response = App.Logic[Channel].Input;								// return Input channel state 0/1
			}
			// prepare and send response
			olen = sprintf( &tmpBuff[0], ",%1d", Response);							// convert Channel into strings with 2 chars
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, tmpBuff, olen);	// send string
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, &(uint8_t *) {CFG_TXCMD_ENDCHAR}, 1);	// send END CHAR
		}
		memset(&App.RxCMD[0],0x00, CFG_RXCMD_MAXLEN);								// clear in array
		return;
	}

	// Report Output Channel Status
	// COMMAND "ADO xx"	+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	if(strncmp((char *) rxCMD, "ADO", strlen("ADO")) == 0)							// command found with parameters!
	{
		//LED_YELLOW_ON_TIME;															// YELLOW LED ON
		// if length > CMD (missing divider after CMD), ignore telegram
		if(strlen(rxCMD) == strlen("ADO"))
		{
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, (const void*) & "SDO ", strlen("SDO "));	// send answer
			olen = sprintf( &tmpBuff[0], "%02d", Channel);							// convert Channel into strings with 2 chars
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, tmpBuff, olen);	// send string
		
			// check if channel is out of range
			if((Channel >= CFG_LOGIC_CHANNEL_COUNT) || \
			(App.Logic[Channel].Enabled == false)) Response = 2;					// and check active channel is enabled (CONST !)
			else																	// execute command and read response 
			{
				Response = App.Logic[Channel].Output;								// return Output channel state 0/1
				if(App.Logic[Channel].PresetTime) App.Alive_CountDown = CFG_ALIVE_TIME;	// If Channel was enabled, also Reset CountDown Timer
			}
			// prepare and send response
			olen = sprintf( &tmpBuff[0], ",%1d", Response);							// convert Channel into strings with 2 chars
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, tmpBuff, olen);	// send string
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, &(uint8_t *) {CFG_TXCMD_ENDCHAR}, 1);	// send END CHAR
		}
		memset(&App.RxCMD[0],0x00, CFG_RXCMD_MAXLEN);								// clear in array
		return;
	}

	
	// Report unsaved output channel uptime
	// COMMAND "SRT xx"	+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	if(strncmp((char *) rxCMD, "SRT", strlen("SRT")) == 0)							// command found with parameters!
	{
		//LED_YELLOW_ON_TIME;															// YELLOW LED ON
		// if length > CMD (missing divider after CMD), ignore telegram
		if(strlen(rxCMD) == strlen("SRT"))
		{
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, (const void*) & "TIM ", strlen("TIM "));	// send answer
			olen = sprintf( &tmpBuff[0], "%02d", Channel);							// convert Channel into strings with 2 chars
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, tmpBuff, olen);	// send string
		
			// check if channel is out of range
			if((Channel >= CFG_LOGIC_CHANNEL_COUNT) || \
			(App.Logic[Channel].Enabled == false)) Response = 0;					// and check active channel is enabled (CONST !)
			else																	// execute command and read response 
			{
				Response = (App.Logic[Channel].OnTime/(CFG_SYSTICKRATE_HZ/CFG_APP_SCANLOGICRATE_HZ));// Operating Time return in second
				if(App.Logic[Channel].Output == 0) App.Logic[Channel].OnTime = 0;	// and clear it... but only if Output not active!!
			}
			// prepare and send response
			olen = sprintf( &tmpBuff[0], ",%03d", Response);						// convert Channel into strings with 2 chars
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, tmpBuff, olen);	// send string
			Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, &(uint8_t *) {CFG_TXCMD_ENDCHAR}, 1);	// send END CHAR
		}
		memset(&App.RxCMD[0],0x00, CFG_RXCMD_MAXLEN);								// clear in array
		return;
	}

	// if no command decode, clear it:
	memset(&App.RxCMD[0],0x00, CFG_RXCMD_MAXLEN);									// clear in array, command not resolved
	Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, (const void*) & "ERR 01", strlen("ERR 01"));	// send error flag
	Chip_UART_SendRB( CFG_UART_PTRPERI, &App.UART_txRing, &(uint8_t *) {CFG_TXCMD_ENDCHAR}, 1);	// send END CHAR
}


// ------------------------------------------------------------------------------------------------------
// SysTick Interrupt Handler - called each 1/CFG_SYSTICKRATE_HZ sec....
// ------------------------------------------------------------------------------------------------------
void App_Init(void)
{
	memset(&App.Logic[0], 0x00, sizeof(App.Logic[0]) * CFG_LOGIC_CHANNEL_COUNT);	// RESET to zero all channel variables.
	App.Logic[1].Enabled = true;													// Enable channel 1
	App.Logic[2].Enabled = true;													// Enable channel 2
	App.Alive_CountDown = CFG_ALIVE_TIME;											// Enable Countdown after restart device
	App.Write_Output =  true;														// request to write default value into MCP23008
	
	App.TickCount = 0;																// Clear TickCounter
}


// ------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------
// Interrupt Routines
// ------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------


// ------------------------------------------------------------------------------------------------------
// SysTick Interrupt Handler - called each 1/CFG_SYSTICKRATE_HZ sec....
// ------------------------------------------------------------------------------------------------------
void SysTick_Handler(void)
{
	App.TickCount += 1;																// system counter
	if((App.TickCount % CFG_SYSTICKRATE_HZ) == 0) App.Second_Flag = true;			// each second
	if((App.TickCount % CFG_APP_SCANLOGICRATE_HZ) == 1) App.ScanLogic_Flag = true;	// each second + 1 ms
	
	if(App.GreenLed_Alive_Count) App.GreenLed_Alive_Count --;						// Green LED lit on. Count to off
	else App.GreenLed_Off = true;

	if(App.YellowLed_Alive_Count) App.YellowLed_Alive_Count --;						// Yellow LED lit on. Count to off
	else App.YellowLed_Off = true;
}


// ------------------------------------------------------------------------------------------------------
// UART Interrupt Handler
// ------------------------------------------------------------------------------------------------------
void USART0_IRQHandler (void)
{
	Chip_UART_IRQRBHandler(LPC_USART0, &App.UART_rxRing, &App.UART_txRing);			// Call RingBuffer UART Handler
	LED_GREEN_ON_TIME;																// GREEN LED ON
}

// ------------------------------------------------------------------------------------------------------
// I2C Interrupt Handler
// ------------------------------------------------------------------------------------------------------
void I2C_IRQHandler(void)
{
	Chip_I2CM_XferHandler(LPC_I2C, &I2CXfer); 										// Call I2CM ISR function with the I2C device and transfer rec
}

// ------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------
// .... And MAIN
// ------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------
int main(void)
{
	uint8_t Chnl = 0;
	
	__disable_irq();																// don't intrude me...
	
	SystemCoreClockUpdate();														// re-read current clock settings
	
	Board_Init();																	// Initialize MCU Board
	Chip_GPIO_WritePortBit( LPC_GPIO_PORT, 0, BRD_LED_GREEN_PORTPIN, false );		// GREEN LED OFF
	Chip_GPIO_WritePortBit( LPC_GPIO_PORT, 0, BRD_LED_YELLOW_PORTPIN, false );		// Yellow LED OFF

	// Application buffers init
	RingBuffer_Init(&App.UART_rxRing, rxbuff, 1, CFG_UART_RB_SIZE);					// initialize Rx RingBuffer for UART
	RingBuffer_Init(&App.UART_txRing, txbuff, 1, CFG_UART_RB_SIZE);					// initialize Tx RingBuffer for UART

	App_Init();																		// Now have to initialize application variables, because it used in interrupt also
	__enable_irq();

	StopWatch_Init();																// Initialize stopwatch

	StopWatch_DelayMs(500);															// first blink after 0.5 sec
	Chip_GPIO_WritePortBit( LPC_GPIO_PORT, 0, BRD_LED_GREEN_PORTPIN, true );		// GREEN LED ON	
	StopWatch_DelayMs(500);															// Lit ON 0.5 sec


	MCP23008_Init();																// Initialize MCP23008

	Chip_GPIO_WritePortBit( LPC_GPIO_PORT, 0, BRD_LED_GREEN_PORTPIN, false );		// GREEN LED OFF
	StopWatch_DelayMs(500);															// wait 0.5 sec
	
	Chip_UART_IntEnable(CFG_UART_PTRPERI, UART_INTEN_RXRDY);						// Enable receive data and line status interrupt
	Chip_UART_IntDisable(CFG_UART_PTRPERI, UART_INTEN_TXRDY);						// Disable transmit data interrupt for now
	NVIC_EnableIRQ(UART0_IRQn);														// Enable IRQ

	Chip_GPIO_WritePortBit( LPC_GPIO_PORT, 0, BRD_LED_GREEN_PORTPIN, true );		// GREEN LED ON
	StopWatch_DelayMs(500);															// Lit ON 0.5 sec
	
	
	Chip_GPIO_WritePortBit( LPC_GPIO_PORT, 0, BRD_LED_GREEN_PORTPIN, false );		// GREEN LED OFF
	StopWatch_DelayMs(500);															// wait 0.5 sec


	while (1) 
	{
		// Chip_I2CM_Xfer
		if((App.TickCount % 100) == 0) 												// each  50ms
		{
			Reload_Inputs();														// Read I2C Inputs
		}
				

		// UART Received data process:
		if(RingBuffer_GetCount(&App.UART_rxRing))									// some data received?
		{
			uint8_t rxchar;
			RingBuffer_Pop(&App.UART_rxRing, &rxchar);								// load char from RxRB
			if(rxchar == CFG_RXCMD_ENDCHAR)											// wait for end char
			{
				LED_YELLOW_ON_TIME;													// Turn On Yellow indication LED
				Decode_RxCMD();														// ACTION !
				App.RxCMDLen = 0;													// Point back to start of array
			}
			else if(App.RxCMDLen <= CFG_RXCMD_MAXLEN) App.RxCMD[App.RxCMDLen++] = toupper(rxchar);// change to upper and store to RxCMD buff
		}	
		
		
		// Logic scanner interval checks:
		if(App.ScanLogic_Flag)
		{
			App.ScanLogic_Flag = false;												// clearerr flag
		
			for(Chnl = 0; Chnl < CFG_LOGIC_CHANNEL_COUNT; Chnl ++)					// check all 99 channels
			{
				if(App.Logic[Chnl].Input)											// Input Active ?
				{	
					// Time is set? And Output still not set? Remote enabled? But Alive is not zero...
					if((App.Logic[Chnl].PresetTime) && (App.Logic[Chnl].Output == 0) && (App.Alive_CountDown) && (App.Logic[Chnl].RemoteEnable))
					{
						App.Logic[Chnl].OnTime = 0;									// Reset OnTime OnTime
						App.Logic[Chnl].Output = 1;									// Yes, switch ON relay and start counting
						App.Write_Output = true;									// Request for update MCP23008 Output pins
					}
				}
				else																// pin deactivated
				{
					// STOPPER: PIN Deactivated
					if((App.Logic[Chnl].PresetTime) && (App.Logic[Chnl].Output))	// Time was set? Output Activated?
					{
						App.Logic[Chnl].Output = 0;									// Yes, switch OFF relay and stop counting
						App.Logic[Chnl].PresetTime = 0;								// clear also Preset time.
						App.Logic[Chnl].RemoteEnable = false;						// Clear remote enable flag
						App.Write_Output = true;									// Request for update MCP23008 Output pins
					}
				}

				// STOPPER: Reached Preset time?
				if(App.Logic[Chnl].OnTime == (App.Logic[Chnl].PresetTime*(CFG_SYSTICKRATE_HZ/CFG_APP_SCANLOGICRATE_HZ)))
				{
					App.Logic[Chnl].Output = 0;										// Yes, switch Off relay and stop counting
					App.Logic[Chnl].PresetTime = 0;									// clear also Preset time.
					App.Logic[Chnl].RemoteEnable = false;							// Clear remote enable flag
					App.Write_Output = true;										// Request for update MCP23008 Output pins
				}
				else App.Logic[Chnl].OnTime += App.Logic[Chnl].Output;				// No, If Output enabled, Counting OnTime Upp by 1.

				
				// STOPPER: OFF Command
				if(App.Logic[Chnl].RemoteEnable == false)							// Alive CountDown reached?
				{
					App.Logic[Chnl].Output = 0;										// Yes, switch OFF relay and stop counting
					App.Logic[Chnl].PresetTime = 0;									// clear also Preset time.
					App.Logic[Chnl].RemoteEnable = false;							// Clear remote enable flag
					
					App.Write_Output = true;										// Request for update MCP23008 Output pins
				}
				
				// STOPPER: Alive timer
				if(App.Alive_CountDown == 0)										// Alive CountDown reached?
				{
					App.Logic[Chnl].Output = 0;										// Yes, switch OFF relay and stop counting
					App.Logic[Chnl].PresetTime = 0;									// clear also Preset time.
					App.Logic[Chnl].RemoteEnable = false;							// Clear remote enable flag
					
					App.Write_Output = true;										// Request for update MCP23008 Output pins
				}				
			}
		}


		
		// One Second Interval:		
		if(App.Second_Flag)
		{
			// Alive timer reached 0. All Outputs have to switch OFF
			// This is a higher priority settings.
			if(App.Alive_CountDown) App.Alive_CountDown--;							// decrease if can...
			App.Second_Flag = false;
	
			//Chip_GPIO_SetPinToggle( LPC_GPIO_PORT, 0, BRD_LED_YELLOW_PORTPIN);		// RUN indicator
		}

		if(App.GreenLed_Off) LED_GREEN_OFF;
		if(App.YellowLed_Off) LED_YELLOW_OFF;
		
		
		if(App.Write_Output) 													// Write I2C Outputs
		{
			Rewrite_Outputs();													// Put into MCP23008
			App.Write_Output = false;											// clear request flag
		}
		
	}	// End While(1)
} 		// End main()
