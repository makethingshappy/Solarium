#line 1 "lpc_chip_82x\\src\\adc_8xx.c"





























 

#line 1 ".\\lpc_chip_82x\\inc\\chip.h"





























 




#line 1 ".\\lpc_chip_82x\\inc\\lpc_types.h"





























 




#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 36 ".\\lpc_chip_82x\\inc\\lpc_types.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdbool.h"
 






 





#line 25 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdbool.h"



#line 37 ".\\lpc_chip_82x\\inc\\lpc_types.h"




 



 



 
typedef enum {FALSE = 0, TRUE = !FALSE} Bool;



 






 
typedef enum {RESET = 0, SET = !RESET} FlagStatus, IntStatus, SetState;




 
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;




 
typedef enum {ERROR = 0, SUCCESS = !ERROR} Status;



 
typedef enum {
	NONE_BLOCKING = 0,		 
	BLOCKING,				 
} TRANSFER_BLOCK_T;

 
typedef void (*PFV)();

 
typedef int32_t (*PFI)();



 



 




 

 





 

 














 

 


 




 


 

 


#line 150 ".\\lpc_chip_82x\\inc\\lpc_types.h"



 

 


 

 
typedef char CHAR;

 
typedef uint8_t UNS_8;

 
typedef int8_t INT_8;

 
typedef uint16_t UNS_16;

 
typedef int16_t INT_16;

 
typedef uint32_t UNS_32;

 
typedef int32_t INT_32;

 
typedef int64_t INT_64;

 
typedef uint64_t UNS_64;






 
typedef _Bool BOOL_32;

 
typedef _Bool BOOL_16;

 
typedef _Bool BOOL_8;










 



 

#line 36 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\sys_config.h"



























 




 



#line 37 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\cmsis.h"





























 




#line 36 ".\\lpc_chip_82x\\inc\\cmsis.h"
#line 37 ".\\lpc_chip_82x\\inc\\cmsis.h"








 



  #pragma diag_suppress 2525

  #pragma anon_unions
#line 63 ".\\lpc_chip_82x\\inc\\cmsis.h"








 

 








 




 

typedef enum {
	 
	Reset_IRQn                    = -15,	 
	NonMaskableInt_IRQn           = -14,	 
	HardFault_IRQn                = -13,	 
	SVCall_IRQn                   = -5,		 
	PendSV_IRQn                   = -2,		 
	SysTick_IRQn                  = -1,		 

	 
	SPI0_IRQn                     = 0,		 
	SPI1_IRQn                     = 1,		 
	Reserved0_IRQn                = 2,		 
	UART0_IRQn                    = 3,		 
	UART1_IRQn                    = 4,		 
	UART2_IRQn                    = 5,		 
	Reserved1_IRQn                = 6,		 
	I2C1_IRQn                     = 7,		 
	I2C0_IRQn                     = 8,		 
	I2C_IRQn                      = 8,		 
	SCT_IRQn                      = 9,		 
	MRT_IRQn                      = 10,		 
	CMP_IRQn                      = 11,		 
	WDT_IRQn                      = 12,		 
	BOD_IRQn                      = 13,		 
	FLASH_IRQn                    = 14,		 
	WKT_IRQn                      = 15,		 
	ADC_SEQA_IRQn                 = 16,		 
	ADC_SEQB_IRQn                 = 17,		 
	ADC_THCMP_IRQn                = 18,		 
	ADC_OVR_IRQn                  = 19,		 
	DMA_IRQn                      = 20,		 
	I2C2_IRQn                     = 21,		 
	I2C3_IRQn                     = 22,		 
	Reserved2_IRQn                = 23,		 
	PININT0_IRQn                  = 24,		 
	PIN_INT0_IRQn                 = 24,		 
	PININT1_IRQn                  = 25,		 
	PIN_INT1_IRQn                 = 25,		 
	PININT2_IRQn                  = 26,		 
	PIN_INT2_IRQn                 = 26,		 
	PININT3_IRQn                  = 27,		 
	PIN_INT3_IRQn                 = 27,		 
	PININT4_IRQn                  = 28,		 
	PIN_INT4_IRQn                 = 28,		 
	PININT5_IRQn                  = 29,		 
	PIN_INT5_IRQn                 = 29,		 
	PININT6_IRQn                  = 30,		 
	PIN_INT6_IRQn                 = 30,		 
	PININT7_IRQn                  = 31,		 
	PIN_INT7_IRQn                 = 31,		 
} IRQn_Type;



 

#line 1 ".\\_CMSIS\\v5.20\\Include\\core_cm0plus.h"
 




 
















 










#line 35 ".\\_CMSIS\\v5.20\\Include\\core_cm0plus.h"

















 




 



 

#line 1 ".\\_CMSIS\\v5.20\\Include\\cmsis_version.h"
 




 
















 










 
#line 64 ".\\_CMSIS\\v5.20\\Include\\core_cm0plus.h"
 
 









 







#line 114 ".\\_CMSIS\\v5.20\\Include\\core_cm0plus.h"

#line 1 ".\\_CMSIS\\v5.20\\Include\\cmsis_compiler.h"
 




 
















 




#line 29 ".\\_CMSIS\\v5.20\\Include\\cmsis_compiler.h"



 
#line 1 ".\\_CMSIS\\v5.20\\Include\\cmsis_armcc.h"
 




 
















 









 













   
   


 
#line 100 ".\\_CMSIS\\v5.20\\Include\\cmsis_armcc.h"

 



 





 
 






 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}


#line 335 ".\\_CMSIS\\v5.20\\Include\\cmsis_armcc.h"


#line 373 ".\\_CMSIS\\v5.20\\Include\\cmsis_armcc.h"



 


 



 




 






 







 






 








 










 










 






                  





 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int16_t __REVSH(int16_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 




__attribute__((always_inline)) static __inline uint32_t __RBIT(uint32_t value)
{
  uint32_t result;
  uint32_t s = (4U   * 8U) - 1U;  

  result = value;                       
  for (value >>= 1U; value != 0U; value >>= 1U)
  {
    result <<= 1U;
    result |= value & 1U;
    s--;
  }
  result <<= s;                         
  return result;
}








 



#line 730 ".\\_CMSIS\\v5.20\\Include\\cmsis_armcc.h"







 
__attribute__((always_inline)) static __inline int32_t __SSAT(int32_t val, uint32_t sat)
{
  if ((sat >= 1U) && (sat <= 32U))
  {
    const int32_t max = (int32_t)((1U << (sat - 1U)) - 1U);
    const int32_t min = -1 - max ;
    if (val > max)
    {
      return max;
    }
    else if (val < min)
    {
      return min;
    }
  }
  return val;
}







 
__attribute__((always_inline)) static __inline uint32_t __USAT(int32_t val, uint32_t sat)
{
  if (sat <= 31U)
  {
    const uint32_t max = ((1U << sat) - 1U);
    if (val > (int32_t)max)
    {
      return max;
    }
    else if (val < 0)
    {
      return 0U;
    }
  }
  return (uint32_t)val;
}




   


 



 

#line 864 ".\\_CMSIS\\v5.20\\Include\\cmsis_armcc.h"
 


#line 35 ".\\_CMSIS\\v5.20\\Include\\cmsis_compiler.h"




 
#line 254 ".\\_CMSIS\\v5.20\\Include\\cmsis_compiler.h"




#line 116 ".\\_CMSIS\\v5.20\\Include\\core_cm0plus.h"

















 
#line 160 ".\\_CMSIS\\v5.20\\Include\\core_cm0plus.h"

 






 
#line 176 ".\\_CMSIS\\v5.20\\Include\\core_cm0plus.h"

 




 











 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:28;               
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:15;               
    uint32_t T:1;                         
    uint32_t _reserved1:3;                
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 





















 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t _reserved1:30;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 







 



 
typedef struct
{
  volatile uint32_t ISER[1U];                
        uint32_t RESERVED0[31U];
  volatile uint32_t ICER[1U];                
        uint32_t RSERVED1[31U];
  volatile uint32_t ISPR[1U];                
        uint32_t RESERVED2[31U];
  volatile uint32_t ICPR[1U];                
        uint32_t RESERVED3[31U];
        uint32_t RESERVED4[64U];
  volatile uint32_t IP[8U];                  
}  NVIC_Type;

 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    

  volatile uint32_t VTOR;                    



  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
        uint32_t RESERVED1;
  volatile uint32_t SHP[2U];                 
  volatile uint32_t SHCSR;                   
} SCB_Type;

 















 




























 




 















 









 






 



 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 

#line 602 ".\\_CMSIS\\v5.20\\Include\\core_cm0plus.h"








 
 







 






 







 


 







 

 














 









 


 



 





 

#line 693 ".\\_CMSIS\\v5.20\\Include\\core_cm0plus.h"
 
 
#line 701 ".\\_CMSIS\\v5.20\\Include\\core_cm0plus.h"
 





#line 716 ".\\_CMSIS\\v5.20\\Include\\core_cm0plus.h"




 
 










 
static __inline void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
  }
}









 
static __inline uint32_t __NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0U] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}







 
static __inline void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);
  }
}









 
static __inline uint32_t __NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0U] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}







 
static __inline void __NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
  }
}







 
static __inline void __NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
  }
}










 
static __inline void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )]  = ((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )]  & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
  else
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] = ((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
}










 
static __inline uint32_t __NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2)));
  }
  else
  {
    return((uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2)));
  }
}











 
static __inline void __NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{

  uint32_t *vectors = (uint32_t *)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;



  vectors[(int32_t)IRQn + 16] = vector;
}









 
static __inline uint32_t __NVIC_GetVector(IRQn_Type IRQn)
{

  uint32_t *vectors = (uint32_t *)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;



  return vectors[(int32_t)IRQn + 16];

}





 
static __inline void __NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FAUL << 16U) |
                 (1UL << 2U));
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 

 







 





 








 
static __inline uint32_t SCB_GetFPUType(void)
{
    return 0U;            
}


 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  __NVIC_SetPriority (SysTick_IRQn, (1UL << 2) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 










#line 147 ".\\lpc_chip_82x\\inc\\cmsis.h"



 





#line 38 ".\\lpc_chip_82x\\inc\\chip.h"


















 

 






 
#line 75 ".\\lpc_chip_82x\\inc\\chip.h"

#line 88 ".\\lpc_chip_82x\\inc\\chip.h"

 







#line 116 ".\\lpc_chip_82x\\inc\\chip.h"


 
#line 127 ".\\lpc_chip_82x\\inc\\chip.h"

 




 
#line 145 ".\\lpc_chip_82x\\inc\\chip.h"



 



 






 
extern const uint32_t OscRateIn;






 
extern const uint32_t ExtRateIn;



 

 
#line 1 ".\\lpc_chip_82x\\inc\\romapi_8xx.h"





























 




#line 1 ".\\lpc_chip_82x\\inc\\iap.h"





























 











 

 
#line 57 ".\\lpc_chip_82x\\inc\\iap.h"

 
#line 79 ".\\lpc_chip_82x\\inc\\iap.h"

 
typedef void (*IAP_ENTRY_T)(unsigned int[], unsigned int[]);









 
uint8_t Chip_IAP_PreSectorForReadWrite(uint32_t strSector, uint32_t endSector);









 
uint8_t Chip_IAP_CopyRamToFlash(uint32_t dstAdd, uint32_t *srcAdd, uint32_t byteswrt);







 
uint8_t Chip_IAP_EraseSector(uint32_t strSector, uint32_t endSector);







 




uint8_t Chip_IAP_BlankCheckSector(uint32_t strSector, uint32_t endSector);




 
uint32_t Chip_IAP_ReadPID(void);




 
uint32_t Chip_IAP_ReadBootCode(void);









 
uint8_t Chip_IAP_Compare(uint32_t dstAdd, uint32_t srcAdd, uint32_t bytescmp);




 
uint8_t Chip_IAP_ReinvokeISP(void);




 
uint32_t Chip_IAP_ReadUID(uint32_t* uid);







 





uint8_t Chip_IAP_ErasePage(uint32_t strPage, uint32_t endPage);



 





#line 36 ".\\lpc_chip_82x\\inc\\romapi_8xx.h"
#line 1 ".\\lpc_chip_82x\\inc\\error_8xx.h"





























 







 







 
typedef enum
{
    LPC_OK = 0,		 
    LPC_ERROR,			 

   
  ERR_ISP_BASE = 0x00000000,
    ERR_ISP_INVALID_COMMAND = ERR_ISP_BASE + 1,
    ERR_ISP_SRC_ADDR_ERROR,			 
    ERR_ISP_DST_ADDR_ERROR,			 
    ERR_ISP_SRC_ADDR_NOT_MAPPED,
    ERR_ISP_DST_ADDR_NOT_MAPPED,
    ERR_ISP_COUNT_ERROR,				 
    ERR_ISP_INVALID_SECTOR,
    ERR_ISP_SECTOR_NOT_BLANK,
    ERR_ISP_SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION,
    ERR_ISP_COMPARE_ERROR,
    ERR_ISP_BUSY,						 
    ERR_ISP_PARAM_ERROR,				 
    ERR_ISP_ADDR_ERROR,				 
    ERR_ISP_ADDR_NOT_MAPPED,
    ERR_ISP_CMD_LOCKED,				 
    ERR_ISP_INVALID_CODE,				 
    ERR_ISP_INVALID_BAUD_RATE,
    ERR_ISP_INVALID_STOP_BIT,
    ERR_ISP_CODE_READ_PROTECTION_ENABLED,

   
  ERR_I2C_BASE = 0x00060000,
    ERR_I2C_NAK = ERR_I2C_BASE + 1,		 
    ERR_I2C_BUFFER_OVERFLOW,				 
    ERR_I2C_BYTE_COUNT_ERR,				 
    ERR_I2C_LOSS_OF_ARBRITRATION,			 
    ERR_I2C_SLAVE_NOT_ADDRESSED,			 
    ERR_I2C_LOSS_OF_ARBRITRATION_NAK_BIT,	 
    ERR_I2C_GENERAL_FAILURE,				 
    ERR_I2C_REGS_SET_TO_DEFAULT,			 
    ERR_I2C_TIMEOUT,						 

   
    ERR_NO_ERROR = LPC_OK,					 
  ERR_UART_BASE = 0x00080000,
    ERR_UART_RXD_BUSY = ERR_UART_BASE + 1,	 
    ERR_UART_TXD_BUSY,						 
    ERR_UART_OVERRUN_FRAME_PARITY_NOISE,	 
    ERR_UART_UNDERRUN,						 
    ERR_UART_PARAM,						 
} ErrorCode_t;



 

#line 37 ".\\lpc_chip_82x\\inc\\romapi_8xx.h"
#line 1 ".\\lpc_chip_82x\\inc\\rom_i2c_8xx.h"





























 











 



 
typedef void *I2C_HANDLE_T;



 
typedef void  (*I2C_CALLBK_T)(uint32_t err_code, uint32_t n);



 
typedef struct I2C_PARAM {
	uint32_t        num_bytes_send;		 
	uint32_t        num_bytes_rec;		 
	uint8_t         *buffer_ptr_send;	 
	uint8_t         *buffer_ptr_rec;	 
	I2C_CALLBK_T    func_pt;			 
	uint8_t         stop_flag;			 
	uint8_t         dummy[3];
} I2C_PARAM_T;



 
typedef struct I2C_RESULT {
	uint32_t n_bytes_sent;	 
	uint32_t n_bytes_recd;	 
} I2C_RESULT_T;



 
typedef enum CHIP_I2C_MODE {
	IDLE,			 
	MASTER_SEND,	 
	MASTER_RECEIVE,	 
	SLAVE_SEND,		 
	SLAVE_RECEIVE	 
} CHIP_I2C_MODE_T;



 
typedef struct  I2CD_API {
	 
	void (*i2c_isr_handler)(I2C_HANDLE_T *handle);

	 
	ErrorCode_t (*i2c_master_transmit_poll)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
	ErrorCode_t (*i2c_master_receive_poll)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
	ErrorCode_t (*i2c_master_tx_rx_poll)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
	ErrorCode_t (*i2c_master_transmit_intr)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
	ErrorCode_t (*i2c_master_receive_intr)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
	ErrorCode_t (*i2c_master_tx_rx_intr)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);

	 
	ErrorCode_t (*i2c_slave_receive_poll)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
	ErrorCode_t (*i2c_slave_transmit_poll)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
	ErrorCode_t (*i2c_slave_receive_intr)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
	ErrorCode_t (*i2c_slave_transmit_intr)(I2C_HANDLE_T *handle, I2C_PARAM_T *param, I2C_RESULT_T *result);
	ErrorCode_t (*i2c_set_slave_addr)(I2C_HANDLE_T *handle, uint32_t slave_addr_0_3, uint32_t slave_mask_0_3);

	 
	uint32_t        (*i2c_get_mem_size)(void);
	I2C_HANDLE_T *  (*i2c_setup)( uint32_t  i2c_base_addr, uint32_t * start_of_ram);
	ErrorCode_t     (*i2c_set_bitrate)(I2C_HANDLE_T *handle, uint32_t  p_clk_in_hz, uint32_t bitrate_in_bps);
	uint32_t        (*i2c_get_firmware_version)(void);
	CHIP_I2C_MODE_T (*i2c_get_status)(I2C_HANDLE_T *handle);
	ErrorCode_t     (*i2c_set_timeout)(I2C_HANDLE_T *handle, uint32_t timeout);
} I2CD_API_T;



 





#line 38 ".\\lpc_chip_82x\\inc\\romapi_8xx.h"
#line 1 ".\\lpc_chip_82x\\inc\\rom_pwr_8xx.h"





























 











 



 







 








 







 






 
typedef struct PWRD_API {
	void (*set_pll)(uint32_t cmd[], uint32_t resp[]);	 
	void (*set_power)(uint32_t cmd[], uint32_t resp[]);	 
} PWRD_API_T;



 





#line 39 ".\\lpc_chip_82x\\inc\\romapi_8xx.h"
#line 1 ".\\lpc_chip_82x\\inc\\rom_uart_8xx.h"





























 











 



 








 
 


 





 
 
 


 
 


 
 


 
 




 






 
typedef void UART_HANDLE_T;



 
typedef void (*UART_CALLBK_T)(uint32_t err_code, uint32_t n);



 
typedef void (*UART_DMA_REQ_T)(uint32_t src_adr, uint32_t dst_adr, uint32_t size);



 
typedef struct {
	uint32_t sys_clk_in_hz;		 
	uint32_t baudrate_in_hz;	 
	uint8_t  config;			 
								 
								 
								 
	uint8_t sync_mod;			 
								 
								 
								 
								 
								 
								 
								 
	uint16_t error_en;			 
								 
								 
								 
								 
								 
} UART_CONFIG_T;



 
typedef struct {
	uint8_t         *buffer;		 
	uint32_t        size;			 
	uint16_t        transfer_mode;	 
									 
									 
									 
									 
									 
									 
									 
									 
	uint16_t        driver_mode;	 
									 
									 
									 
	UART_CALLBK_T   callback_func_pt;	 
	UART_DMA_REQ_T  dma_req_func_pt;	 
} UART_PARAM_T;



 
typedef struct UARTD_API {
	 
	uint32_t        (*uart_get_mem_size)(void);	 
	UART_HANDLE_T * (*uart_setup)(uint32_t base_addr, uint8_t * ram);	 
	uint32_t        (*uart_init)(UART_HANDLE_T *handle, UART_CONFIG_T *set);	 

	 
	uint8_t         (*uart_get_char)(UART_HANDLE_T *handle);	 
	void            (*uart_put_char)(UART_HANDLE_T *handle, uint8_t data);	 
	uint32_t        (*uart_get_line)(UART_HANDLE_T *handle, UART_PARAM_T *param);	 
	uint32_t        (*uart_put_line)(UART_HANDLE_T *handle, UART_PARAM_T *param);	 

	 
	void            (*uart_isr)(UART_HANDLE_T *handle);	 
} UARTD_API_T;



 





#line 40 ".\\lpc_chip_82x\\inc\\romapi_8xx.h"








 



 
typedef struct ROM_API {
	const uint32_t    unused[3];
	const PWRD_API_T  *pPWRD;	 
	const uint32_t    p_dev1;
	const I2CD_API_T  *pI2CD;	 
	const uint32_t    p_dev3;
	const uint32_t    p_dev4;
	const uint32_t    p_dev5;
	const UARTD_API_T *pUARTD;	 
} LPC_ROM_API_T;

 



 


 


 


 




 
static __inline void iap_entry(unsigned int cmd_param[], unsigned int status_result[])
{
	((IAP_ENTRY_T) 0X1FFF1FF1UL)(cmd_param, status_result);
}



 





#line 176 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\syscon_8xx.h"





























 











 



 








 
#line 69 ".\\lpc_chip_82x\\inc\\syscon_8xx.h"



 





 
#line 88 ".\\lpc_chip_82x\\inc\\syscon_8xx.h"



 




 
typedef struct {
	volatile uint32_t SYSMEMREMAP;			 
	volatile uint32_t PRESETCTRL;			 
	volatile uint32_t SYSPLLCTRL;			 
	volatile uint32_t SYSPLLSTAT;			 
	uint32_t RESERVED0[4];
	volatile uint32_t SYSOSCCTRL;			 
	volatile uint32_t WDTOSCCTRL;			 
	volatile uint32_t IRCCTRL;               
	uint32_t RESERVED1[1];
	volatile uint32_t SYSRSTSTAT;			 
	uint32_t RESERVED2[3];
	volatile uint32_t SYSPLLCLKSEL;			 
	volatile uint32_t SYSPLLCLKUEN;			 
	uint32_t RESERVED3[10];
	volatile uint32_t MAINCLKSEL;			 
	volatile uint32_t MAINCLKUEN;			 
	volatile uint32_t SYSAHBCLKDIV;			 
	uint32_t RESERVED4[1];
	volatile uint32_t SYSAHBCLKCTRL;		 
	uint32_t RESERVED5[4];
	volatile uint32_t UARTCLKDIV;			 
	uint32_t RESERVED6[18];
	volatile uint32_t CLKOUTSEL;			 
	volatile uint32_t CLKOUTUEN;			 
	volatile uint32_t CLKOUTDIV;			 
	uint32_t RESERVED7;
	volatile uint32_t UARTFRGDIV;			 
	volatile uint32_t UARTFRGMULT;			 
	uint32_t RESERVED8[1];
	volatile uint32_t EXTTRACECMD;			 
	volatile uint32_t PIOPORCAP0;			 
	uint32_t RESERVED9[12];
	volatile uint32_t IOCONCLKDIV[7];		 
	volatile uint32_t BODCTRL;				 
	volatile uint32_t SYSTCKCAL;			 
	uint32_t RESERVED10[6];
	volatile uint32_t IRQLATENCY;			 
	volatile uint32_t NMISRC;				 
	volatile uint32_t PINTSEL[8];			 
	uint32_t RESERVED11[27];
	volatile uint32_t STARTERP0;			 
	uint32_t RESERVED12[3];
	volatile uint32_t STARTERP1;			 
	uint32_t RESERVED13[6];
	volatile uint32_t PDSLEEPCFG;			 
	volatile uint32_t PDAWAKECFG;			 
	volatile uint32_t PDRUNCFG;				 
	uint32_t RESERVED14[111];
	volatile const  uint32_t DEVICEID;				 
} LPC_SYSCTL_T;




 
typedef enum CHIP_PIN_CLKDIV {
	IOCONCLKDIV0 = 0,				 
	IOCONCLKDIV1,					 
	IOCONCLKDIV2,					 
	IOCONCLKDIV3,					 
	IOCONCLKDIV4,					 
	IOCONCLKDIV5,					 
	IOCONCLKDIV6,					 
	IOCONCLK_MAX = IOCONCLKDIV6		 
} CHIP_PIN_CLKDIV_T;

 
#line 201 ".\\lpc_chip_82x\\inc\\syscon_8xx.h"
 
 
 
 



 
typedef enum CHIP_SYSCTL_BOOT_MODE_REMAP {
	REMAP_BOOT_LOADER_MODE,	 
	REMAP_USER_RAM_MODE,	 
	REMAP_USER_FLASH_MODE	 
} CHIP_SYSCTL_BOOT_MODE_REMAP_T;



 
typedef enum {
	RESET_SPI0,			 
	RESET_SPI1,			 
	RESET_UARTFBRG,		 
	RESET_USART0,		 
	RESET_USART1,		 
	RESET_USART2,		 
	RESET_I2C0,			 
	RESET_MRT,			 
	RESET_SCT,			 
	RESET_WKT,			 
	RESET_GPIO,			 
	RESET_FLASH,		 
	RESET_ACMP,			 
	RESET_I2C1 = 14,	 
	RESET_I2C2,			 
	RESET_I2C3,			 
} CHIP_SYSCTL_PERIPH_RESET_T;

 




 
typedef enum CHIP_SYSCTL_BODRSTLVL {
	SYSCTL_BODRSTLVL_0,	 
	SYSCTL_BODRSTLVL_1,	 
	SYSCTL_BODRSTLVL_2,	 
	SYSCTL_BODRSTLVL_3,	 
} CHIP_SYSCTL_BODRSTLVL_T;



 
typedef enum CHIP_SYSCTL_BODRINTVAL {
	SYSCTL_BODINTVAL_LVL0,	 
	SYSCTL_BODINTVAL_LVL1,	 
	SYSCTL_BODINTVAL_LVL2,	 
	SYSCTL_BODINTVAL_LVL3,	 
} CHIP_SYSCTL_BODRINTVAL_T;





 
static __inline void Chip_SYSCTL_Map(CHIP_SYSCTL_BOOT_MODE_REMAP_T remap)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->SYSMEMREMAP = (uint32_t) remap;
}







 
static __inline void Chip_SYSCTL_AssertPeriphReset(CHIP_SYSCTL_PERIPH_RESET_T periph)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->PRESETCTRL &= ~((1 << (uint32_t) periph) | 0xfffe2000);
}





 
static __inline void Chip_SYSCTL_DeassertPeriphReset(CHIP_SYSCTL_PERIPH_RESET_T periph)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->PRESETCTRL = (1 << (uint32_t) periph) | (((LPC_SYSCTL_T *) (0x40048000UL))->PRESETCTRL & ~0xfffe2000);
}





 
static __inline void Chip_SYSCTL_PeriphReset(CHIP_SYSCTL_PERIPH_RESET_T periph)
{
	Chip_SYSCTL_AssertPeriphReset(periph);
	Chip_SYSCTL_DeassertPeriphReset(periph);
}





 
static __inline uint32_t Chip_SYSCTL_GetSystemRSTStatus(void)
{
	return ((LPC_SYSCTL_T *) (0x40048000UL))->SYSRSTSTAT & ~(~0x1f);
}






 
static __inline void Chip_SYSCTL_ClearSystemRSTStatus(uint32_t reset)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->SYSRSTSTAT = reset;
}





 
static __inline uint32_t Chip_SYSCTL_GetPORPIOStatus(void)
{
	return ((LPC_SYSCTL_T *) (0x40048000UL))->PIOPORCAP0 & ~0xfffc0000;
}








 
static __inline void Chip_SYSCTL_SetBODLevels(CHIP_SYSCTL_BODRSTLVL_T rstlvl,
											CHIP_SYSCTL_BODRINTVAL_T intlvl)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->BODCTRL = ((uint32_t) rstlvl) | (((uint32_t) intlvl) << 2);
}




 
static __inline void Chip_SYSCTL_EnableBODReset(void)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->BODCTRL = (1 << 4) | (((LPC_SYSCTL_T *) (0x40048000UL))->BODCTRL & ~(~0x1f));
}




 
static __inline void Chip_SYSCTL_DisableBODReset(void)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->BODCTRL &= ~((1 << 4) | (~0x1f));
}





 
static __inline void Chip_SYSCTL_SetSYSTCKCAL(uint32_t sysCalVal)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->SYSTCKCAL = sysCalVal;
}







 
static __inline void Chip_SYSCTL_SetIRQLatency(uint32_t latency)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->IRQLATENCY = latency;
}




 
static __inline uint32_t Chip_SYSCTL_GetIRQLatency(void)
{
	return ((LPC_SYSCTL_T *) (0x40048000UL))->IRQLATENCY & ~(~0xff);
}







 
static __inline void Chip_SYSCTL_SetNMISource(uint32_t intsrc)
{
     
    ((LPC_SYSCTL_T *) (0x40048000UL))->NMISRC &= ~(((uint32_t) 1 << 31) | (~(0x1f|(1u<<31))));
    
     
	((LPC_SYSCTL_T *) (0x40048000UL))->NMISRC = intsrc;
}




 
static __inline void Chip_SYSCTL_EnableNMISource(void)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->NMISRC = ((uint32_t) 1 << 31) | (((LPC_SYSCTL_T *) (0x40048000UL))->NMISRC & ~(~(0x1f|(1u<<31))));
}




 
static __inline void Chip_SYSCTL_DisableNMISource(void)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->NMISRC &= ~(((uint32_t) 1 << 31) | (~(0x1f|(1u<<31))));
}









 
static __inline void Chip_SYSCTL_SetPinInterrupt(uint32_t intno, uint32_t pin)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->PINTSEL[intno] = (uint32_t) pin;
}







 
static __inline void Chip_SYSCTL_EnablePINTWakeup(uint32_t pin)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->STARTERP0 = (1 << pin) | (((LPC_SYSCTL_T *) (0x40048000UL))->STARTERP0 & ~(~0xff));
}






 
static __inline void Chip_SYSCTL_DisablePINTWakeup(uint32_t pin)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->STARTERP0 &= ~((1 << pin) | (~0xff));
}





 
static __inline void Chip_SYSCTL_EnablePeriphWakeup(uint32_t periphmask)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->STARTERP1 = periphmask | (((LPC_SYSCTL_T *) (0x40048000UL))->STARTERP0 & ~(~0xff));
}





 
static __inline void Chip_SYSCTL_DisablePeriphWakeup(uint32_t periphmask)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->STARTERP1 &= ~(periphmask | ((1<<2)|(1<<6)|(7<<9)|(1<<14)|0xff9f0000));
}





 
static __inline uint32_t Chip_SYSCTL_GetDeepSleepPD(void)
{
	return ((LPC_SYSCTL_T *) (0x40048000UL))->PDSLEEPCFG;
}





 
static __inline uint32_t Chip_SYSCTL_GetWakeup(void)
{
	return ((LPC_SYSCTL_T *) (0x40048000UL))->PDAWAKECFG;
}





 
static __inline uint32_t Chip_SYSCTL_GetPowerStates(void)
{
	return ((LPC_SYSCTL_T *) (0x40048000UL))->PDRUNCFG;
}




 
static __inline uint32_t Chip_SYSCTL_GetDeviceID(void)
{
	return ((LPC_SYSCTL_T *) (0x40048000UL))->DEVICEID;
}










 
void Chip_SYSCTL_SetDeepSleepPD(uint32_t sleepmask);










 
void Chip_SYSCTL_SetWakeup(uint32_t wakeupmask);





 
void Chip_SYSCTL_PowerDown(uint32_t powerdownmask);





 
void Chip_SYSCTL_PowerUp(uint32_t powerupmask);



 





#line 177 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\clock_8xx.h"





























 











 

 







 
typedef enum CHIP_SYSCTL_PLLCLKSRC {
	SYSCTL_PLLCLKSRC_IRC = 0,		 
	SYSCTL_PLLCLKSRC_SYSOSC,		 
	SYSCTL_PLLCLKSRC_RESERVED,
	SYSCTL_PLLCLKSRC_EXT_CLKIN,	 
} CHIP_SYSCTL_PLLCLKSRC_T;




 
typedef enum CHIP_WDTLFO_OSC {
	WDTLFO_OSC_ILLEGAL,
	WDTLFO_OSC_0_60,	 
	WDTLFO_OSC_1_05,	 
	WDTLFO_OSC_1_40,	 
	WDTLFO_OSC_1_75,	 
	WDTLFO_OSC_2_10,	 
	WDTLFO_OSC_2_40,	 
	WDTLFO_OSC_2_70,	 
	WDTLFO_OSC_3_00,	 
	WDTLFO_OSC_3_25,	 
	WDTLFO_OSC_3_50,	 
	WDTLFO_OSC_3_75,	 
	WDTLFO_OSC_4_00,	 
	WDTLFO_OSC_4_20,	 
	WDTLFO_OSC_4_40,	 
	WDTLFO_OSC_4_60		 
} CHIP_WDTLFO_OSC_T;



 
typedef enum CHIP_SYSCTL_MAINCLKSRC {
	SYSCTL_MAINCLKSRC_IRC = 0,		 
	SYSCTL_MAINCLKSRC_PLLIN,		 
	SYSCTL_MAINCLKSRC_WDTOSC,		 
	SYSCTL_MAINCLKSRC_PLLOUT,		 
} CHIP_SYSCTL_MAINCLKSRC_T;



 
typedef enum CHIP_SYSCTL_CLOCK {
	SYSCTL_CLOCK_SYS = 0,	 
	SYSCTL_CLOCK_ROM,		 
	SYSCTL_CLOCK_RAM,		 
	SYSCTL_CLOCK_FLASHREG,	 
	SYSCTL_CLOCK_FLASH,		 
	SYSCTL_CLOCK_I2C0,		 
	SYSCTL_CLOCK_GPIO,		 
	SYSCTL_CLOCK_SWM,		 
	SYSCTL_CLOCK_SCT,		 
	SYSCTL_CLOCK_WKT,		 
	SYSCTL_CLOCK_MRT,		 
	SYSCTL_CLOCK_SPI0,		 
	SYSCTL_CLOCK_SPI1,		 
	SYSCTL_CLOCK_CRC,		 
	SYSCTL_CLOCK_UART0,		 
	SYSCTL_CLOCK_UART1,		 
	SYSCTL_CLOCK_UART2,		 
	SYSCTL_CLOCK_WWDT,		 
	SYSCTL_CLOCK_IOCON,		 
	SYSCTL_CLOCK_ACOMP,		 

	 
	SYSCTL_CLOCK_I2C1 = 21,  
	SYSCTL_CLOCK_I2C2,       
	SYSCTL_CLOCK_I2C3,       
	SYSCTL_CLOCK_ADC,        
	SYSCTL_CLOCK_MTB = 26,   
	SYSCTL_CLOCK_DMA = 29,   
} CHIP_SYSCTL_CLOCK_T;

 





 
typedef enum CHIP_SYSCTL_CLKOUTSRC {
	SYSCTL_CLKOUTSRC_IRC = 0,		 
	SYSCTL_CLKOUTSRC_SYSOSC,		 
	SYSCTL_CLKOUTSRC_WDTOSC,		 
	SYSCTL_CLKOUTSRC_MAINSYSCLK,	 
} CHIP_SYSCTL_CLKOUTSRC_T;







 
static __inline void Chip_Clock_SetupSystemPLL(uint8_t msel, uint8_t psel)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->SYSPLLCTRL = (msel & 0x1F) | ((psel & 0x3) << 5);
}




 
static __inline _Bool Chip_Clock_IsSystemPLLLocked(void)
{
	return (_Bool) ((((LPC_SYSCTL_T *) (0x40048000UL))->SYSPLLSTAT & 1) != 0);
}







 
static __inline void Chip_Clock_SetWDTOSC(CHIP_WDTLFO_OSC_T wdtclk, uint8_t div)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->WDTOSCCTRL  = (((uint32_t) wdtclk) << 5) | ((div >> 1) - 1);
}




 
static __inline CHIP_SYSCTL_MAINCLKSRC_T Chip_Clock_GetMainClockSource(void)
{
	return (CHIP_SYSCTL_MAINCLKSRC_T) (((LPC_SYSCTL_T *) (0x40048000UL))->MAINCLKSEL & ~(~3));
}







 
static __inline void Chip_Clock_SetSysClockDiv(uint32_t div)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->SYSAHBCLKDIV  = div;
}





 
static __inline void Chip_Clock_EnablePeriphClock(CHIP_SYSCTL_CLOCK_T clk)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->SYSAHBCLKCTRL = (1 << clk) | (((LPC_SYSCTL_T *) (0x40048000UL))->SYSAHBCLKCTRL & ~0xda100000);
}





 
static __inline void Chip_Clock_DisablePeriphClock(CHIP_SYSCTL_CLOCK_T clk)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->SYSAHBCLKCTRL &= ~((1 << clk) | 0xda100000);
}







 
static __inline void Chip_Clock_SetUARTClockDiv(uint32_t div)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->UARTCLKDIV = div;
}





 
static __inline uint32_t Chip_Clock_GetUARTClockDiv(void)
{
	return ((LPC_SYSCTL_T *) (0x40048000UL))->UARTCLKDIV & ~(~0xff);
}





 
static __inline void Chip_SYSCTL_SetUSARTFRGDivider(uint8_t div)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->UARTFRGDIV = (uint32_t) div;
}




 
static __inline uint32_t Chip_SYSCTL_GetUSARTFRGDivider(void)
{
	return ((LPC_SYSCTL_T *) (0x40048000UL))->UARTFRGDIV & ~(~0xff);
}





 
static __inline void Chip_SYSCTL_SetUSARTFRGMultiplier(uint8_t mult)
{
	((LPC_SYSCTL_T *) (0x40048000UL))->UARTFRGMULT = (uint32_t) mult;
}




 
static __inline uint32_t Chip_SYSCTL_GetUSARTFRGMultiplier(void)
{
	return ((LPC_SYSCTL_T *) (0x40048000UL))->UARTFRGMULT & ~(~0xff);
}

















 
uint32_t Chip_Clock_SetUSARTNBaseClockRate(uint32_t rate, _Bool fEnable);




 
uint32_t Chip_Clock_GetUSARTNBaseClockRate(void);




 
static __inline uint32_t Chip_Clock_GetMainOscRate(void)
{
	return OscRateIn;
}




 
static __inline uint32_t Chip_Clock_GetIntOscRate(void)
{
	return (12000000);
}




 
static __inline uint32_t Chip_Clock_GetExtClockInRate(void)
{
	return ExtRateIn;
}







 
void Chip_Clock_SetSystemPLLSource(CHIP_SYSCTL_PLLCLKSRC_T src);









 
void Chip_Clock_SetPLLBypass(_Bool bypass, _Bool highfr);







 
void Chip_Clock_SetMainClockSource(CHIP_SYSCTL_MAINCLKSRC_T src);










 
void Chip_Clock_SetCLKOUTSource(CHIP_SYSCTL_CLKOUTSRC_T src, uint32_t div);





 
uint32_t Chip_Clock_GetWDTOSCRate(void);




 
uint32_t Chip_Clock_GetSystemPLLInClockRate(void);




 
uint32_t Chip_Clock_GetSystemPLLOutClockRate(void);




 
uint32_t Chip_Clock_GetMainClockRate(void);




 
uint32_t Chip_Clock_GetSystemClockRate(void);






 
uint32_t Chip_Clock_GetIOCONCLKDIVClockRate(CHIP_PIN_CLKDIV_T reg);







 
void Chip_Clock_SetIOCONCLKDIV(CHIP_PIN_CLKDIV_T reg, uint8_t div);



 





#line 178 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\iocon_8xx.h"





























 











 





 
typedef struct {
	uint32_t pin:8;			 
	uint32_t modefunc:24;	 
} PINMUX_GRP_T;








 
typedef struct {		 
	volatile uint32_t PIO0[(29) + 2];  
} LPC_IOCON_T;



 
 



 



 



 



 



 



 











 
typedef enum CHIP_PINx {
	IOCON_PIO0  =  0x11,	 
	IOCON_PIO1  =  0x0B,	 
	IOCON_PIO2  =  0x06,	 
	IOCON_PIO3  =  0x05,	 
	IOCON_PIO4  =  0x04,	 
	IOCON_PIO5  =  0x03,	 
	 
	IOCON_PIO6  =  0x10,	 
	IOCON_PIO7  =  0x0F,	 
	IOCON_PIO8  =  0x0E,	 
	IOCON_PIO9  =  0x0D,	 
	IOCON_PIO10 =  0x08,	 
	IOCON_PIO11 =  0x07,	 
	IOCON_PIO12 =  0x02,	 
	IOCON_PIO13 =  0x01,	 
	 
	IOCON_PIO14 =  0x12,	 
	IOCON_PIO15 =  0x0A,	 
	IOCON_PIO16 =  0x09,	 
	IOCON_PIO17 =  0x00,	 
	IOCON_PIO_NUL0 = 0x0C,	 

	 
	IOCON_PIO18 =  0x1E,	 
	IOCON_PIO19 =  0x1D,	 
	IOCON_PIO20 =  0x1C,	 
	IOCON_PIO21 =  0x1B,	 
	IOCON_PIO22 =  0x1A,	 
	IOCON_PIO23 =  0x19,	 
	IOCON_PIO24 =  0x18,	 
	IOCON_PIO25 =  0x17,	 
	IOCON_PIO26 =  0x16,	 
	IOCON_PIO27 =  0x15,	 
	IOCON_PIO28 =  0x14,	 
	IOCON_PIO_NUL1 = 0x13,	 
} CHIP_PINx_T;



 
typedef enum CHIP_PIN_MODE {
	PIN_MODE_INACTIVE = 0,	 
	PIN_MODE_PULLDN = 1,	 
	PIN_MODE_PULLUP = 2,	 
	PIN_MODE_REPEATER = 3	 
} CHIP_PIN_MODE_T;



 
typedef enum CHIP_PIN_SMODE {
	PIN_SMODE_BYPASS = 0,	 
	PIN_SMODE_CYC1 = 1,		 
	PIN_SMODE_CYC2 = 2,		 
	PIN_SMODE_CYC3 = 3		 
} CHIP_PIN_SMODE_T;



 
typedef enum CHIP_PIN_I2CMODE {
	PIN_I2CMODE_STDFAST = 0,	 
	PIN_I2CMODE_GPIO = 1,		 
	PIN_I2CMODE_FASTPLUS = 2	 
} CHIP_PIN_I2CMODE_T;







 
static __inline void Chip_IOCON_PinMuxSet(LPC_IOCON_T *pIOCON, uint8_t pin, uint32_t modefunc)
{
	pIOCON->PIO0[pin] = modefunc;
}







 
void Chip_IOCON_SetPinMuxing(LPC_IOCON_T *pIOCON, const PINMUX_GRP_T* pinArray, uint32_t arrayLength);








 
void Chip_IOCON_PinSetMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, CHIP_PIN_MODE_T mode);








 
void Chip_IOCON_PinSetHysteresis(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, _Bool enable);







 
static __inline void Chip_IOCON_PinEnableHysteresis(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin)
{
	pIOCON->PIO0[pin] |= (0x1 << 5);
}







 
static __inline void Chip_IOCON_PinDisableHysteresis(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin)
{
	pIOCON->PIO0[pin] &= ~(0x1 << 5);
}







 
void Chip_IOCON_PinSetInputInverted(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, _Bool invert);






 
static __inline void Chip_IOCON_PinEnableInputInverted(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin)
{
	pIOCON->PIO0[pin] |= (0x1 << 6);
}






 
static __inline void Chip_IOCON_PinDisableInputInverted(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin)
{
	pIOCON->PIO0[pin] &= ~(0x1 << 6);
}








 
void Chip_IOCON_PinSetOpenDrainMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, _Bool open_drain);






 
static __inline void Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin)
{
	pIOCON->PIO0[pin] |= (0x1 << 10);
}






 
static __inline void Chip_IOCON_PinDisableOpenDrainMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin)
{
	pIOCON->PIO0[pin] &= ~(0x1 << 10);
}







 
void Chip_IOCON_PinSetSampleMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, CHIP_PIN_SMODE_T smode);







 
void Chip_IOCON_PinSetClockDivisor(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, CHIP_PIN_CLKDIV_T clkdiv);








 
void Chip_IOCON_PinSetI2CMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, CHIP_PIN_I2CMODE_T mode);



 





#line 179 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\swm_8xx.h"





























 











 



 
typedef struct {

	volatile uint32_t PINASSIGN[12];	 
	volatile const  uint32_t RESERVED0[100];




	volatile uint32_t PINENABLE0;		 
} LPC_SWM_T;









 

typedef enum CHIP_SWM_PIN_MOVABLE {
	SWM_U0_TXD_O,                 
	SWM_U0_RXD_I,                 
	SWM_U0_RTS_O,                 
	SWM_U0_CTS_I,                 

	SWM_U0_SCLK_IO = 0x10,        
	SWM_U1_TXD_O,                 
	SWM_U1_RXD_I,                 
	SWM_U1_RTS_O,                 

	SWM_U1_CTS_I = 0x20,          
	SWM_U1_SCLK_IO,               
	SWM_U2_TXD_O,                 
	SWM_U2_RXD_I,                 

	SWM_U2_RTS_O = 0x30,          
	SWM_U2_CTS_I,                 
	SWM_U2_SCLK_IO,               
	SWM_SPI0_SCK_IO,              

	SWM_SPI0_MOSI_IO = 0x40,      
	SWM_SPI0_MISO_IO,             
	SWM_SPI0_SSEL0_IO,            
	SWM_SPI0_SSEL1_IO,            

	SWM_SPI0_SSEL2_IO = 0x50,     
	SWM_SPI0_SSEL3_IO,            
	SWM_SPI1_SCK_IO,              
	SWM_SPI1_MOSI_IO,             

	SWM_SPI1_MISO_IO = 0x60,      
	SWM_SPI1_SSEL0_IO,            
	SWM_SPI1_SSEL1_IO,            
	SWM_SCT_IN0_I,                

	SWM_SCT_IN1_I = 0x70,         
	SWM_SCT_IN2_I,                
	SWM_SCT_IN3_I,                
	SWM_SCT_OUT0_O,               

	SWM_SCT_OUT1_O = 0x80,        
	SWM_SCT_OUT2_O,               
	SWM_SCT_OUT3_O,               
	SWM_SCT_OUT4_O,               

	SWM_SCT_OUT5_O = 0x90,        
	SWM_I2C1_SDA_IO,              
	SWM_I2C1_SCL_IO,              
	SWM_I2C2_SDA_IO,              

	SWM_I2C2_SCL_IO = 0xA0,       
	SWM_I2C3_SDA_IO,              
	SWM_I2C3_SCL_IO,              
	SWM_ADC_PINTRIG0_I,           

	SWM_ADC_PINTRIG1_I = 0xB0,    
	SWM_ACMP_O_O,                 
	SWM_CLKOUT_O,                 
	SWM_GPIO_INT_BMAT_O,          

} CHIP_SWM_PIN_MOVABLE_T;
#line 170 ".\\lpc_chip_82x\\inc\\swm_8xx.h"



 

typedef enum CHIP_SWM_PIN_FIXED    {
	SWM_FIXED_ACMP_I1 = 0,	 
	SWM_FIXED_ACMP_I2 = 1,	 
	SWM_FIXED_ACMP_I3 = 2,	 
	SWM_FIXED_ACMP_I4 = 3,	 
	SWM_FIXED_SWCLK   = 4,	 
	SWM_FIXED_SWDIO   = 5,	 
	SWM_FIXED_XTALIN  = 6,	 
	SWM_FIXED_XTALOUT = 7,	 
	SWM_FIXED_RST     = 8,	 
	SWM_FIXED_CLKIN   = 9,	 
	SWM_FIXED_VDDCMP  = 10,	 
	SWM_FIXED_I2C0_SDA  = 11,	 
	SWM_FIXED_I2C0_SCL  = 12,	 
	SWM_FIXED_ADC0    = 13,	 
	SWM_FIXED_ADC1    = 14,	 
	SWM_FIXED_ADC2    = 15,	 
	SWM_FIXED_ADC3    = 16,	 
	SWM_FIXED_ADC4    = 17,	 
	SWM_FIXED_ADC5    = 18,	 
	SWM_FIXED_ADC6    = 19,	 
	SWM_FIXED_ADC7    = 20,	 
	SWM_FIXED_ADC8    = 21,	 
	SWM_FIXED_ADC9    = 22,	 
	SWM_FIXED_ADC10   = 23,	 
	SWM_FIXED_ADC11   = 24,	 
} CHIP_SWM_PIN_FIXED_T;
#line 215 ".\\lpc_chip_82x\\inc\\swm_8xx.h"





 
static __inline void Chip_SWM_Init(void)
{
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
}





 
static __inline void Chip_SWM_Deinit(void)
{
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
}






 
void Chip_SWM_MovablePinAssign(CHIP_SWM_PIN_MOVABLE_T movable, uint8_t assign);






 
void Chip_SWM_FixedPinEnable(CHIP_SWM_PIN_FIXED_T pin, _Bool enable);





 
static __inline void Chip_SWM_EnableFixedPin(CHIP_SWM_PIN_FIXED_T pin)
{
	((LPC_SWM_T *) (0x4000C000UL))->PINENABLE0 &= ~((1 << (uint32_t) pin) | (~0x1ffffff));
}





 
static __inline void Chip_SWM_DisableFixedPin(CHIP_SWM_PIN_FIXED_T pin)
{
	((LPC_SWM_T *) (0x4000C000UL))->PINENABLE0 = (1 << (uint32_t) pin) | (((LPC_SWM_T *) (0x4000C000UL))->PINENABLE0 & ~(~0x1ffffff));
}





 
static __inline _Bool Chip_SWM_IsEnabled(CHIP_SWM_PIN_FIXED_T pin)
{
	return (_Bool) ((((LPC_SWM_T *) (0x4000C000UL))->PINENABLE0 & (1 << (uint32_t) pin)) == 0);
}



 





#line 180 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\fmc_8xx.h"





























 











 



 
typedef struct {
	volatile const  uint32_t  RESERVED1[4];
	volatile uint32_t  FLASHCFG;		 
	volatile const  uint32_t  RESERVED2[3];
	volatile uint32_t  FMSSTART;		 
	volatile uint32_t  FMSSTOP;			 
	volatile const  uint32_t  RESERVED3;
	volatile const  uint32_t  FMSW[1];			 
} LPC_FMC_T;

 






 
typedef enum {
	FLASHTIM_20MHZ_CPU = 0,		 
	FLASHTIM_30MHZ_CPU = 1, 	 
} FMC_FLASHTIM_T;







 
static __inline void Chip_FMC_SetFLASHAccess(FMC_FLASHTIM_T clks)
{
	uint32_t tmp = ((LPC_FMC_T *) (0x40040000UL))->FLASHCFG & (~((0x3)|(~3)));

	 
	((LPC_FMC_T *) (0x40040000UL))->FLASHCFG = tmp | clks;
}

 











 
static __inline void Chip_FMC_ComputeSignature(uint32_t start, uint32_t stop)
{
	((LPC_FMC_T *) (0x40040000UL))->FMSSTART = (start >> 4);
	((LPC_FMC_T *) (0x40040000UL))->FMSSTOP = (stop >> 4) | (1UL << 31);
}










 
static __inline void Chip_FMC_ComputeSignatureBlocks(uint32_t start, uint32_t blocks)
{
	Chip_FMC_ComputeSignature(start, (start + (blocks * 16)));
}




 
static __inline _Bool Chip_FMC_IsSignatureBusy(void)
{
	return (_Bool) ((((LPC_FMC_T *) (0x40040000UL))->FMSSTOP & (1UL << 31)) != 0);
}





 
static __inline uint32_t Chip_FMC_GetSignature(int index)
{
	return ((LPC_FMC_T *) (0x40040000UL))->FMSW[index];
}



 





#line 181 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\pinint_8xx.h"





























 











 



 
typedef struct {			 
	volatile uint32_t ISEL;		 
	volatile uint32_t IENR;		 
	volatile uint32_t SIENR;	 
	volatile uint32_t CIENR;	 
	volatile uint32_t IENF;		 
	volatile uint32_t SIENF;	 
	volatile uint32_t CIENF;	 
	volatile uint32_t RISE;		 
	volatile uint32_t FALL;		 
	volatile uint32_t IST;		 
	volatile uint32_t PMCTRL;	 
	volatile uint32_t PMSRC;	 
	volatile uint32_t PMCFG;	 
} LPC_PIN_INT_T;

 
#line 77 ".\\lpc_chip_82x\\inc\\pinint_8xx.h"




 
 



 



 





 
#line 106 ".\\lpc_chip_82x\\inc\\pinint_8xx.h"



 
typedef enum Chip_PININT_BITSLICE {
	PININTBITSLICE0 = 0,	 
	PININTBITSLICE1 = 1,	 
	PININTBITSLICE2 = 2,	 
	PININTBITSLICE3 = 3,	 
	PININTBITSLICE4 = 4,	 
	PININTBITSLICE5 = 5,	 
	PININTBITSLICE6 = 6,	 
	PININTBITSLICE7 = 7	 
} Chip_PININT_BITSLICE_T;



 
typedef enum Chip_PININT_BITSLICE_CFG {
    PININT_PATTERNCONST1           = 0x0,	 
    PININT_PATTERNRISING           = 0x1,	     
    PININT_PATTERNFALLING          = 0x2,	     
    PININT_PATTERNRISINGRFALLING   = 0x3,	     
    PININT_PATTERNHIGH             = 0x4,	     
    PININT_PATTERNLOW              = 0x5,	     
    PININT_PATTERCONST0            = 0x6,	     
    PININT_PATTEREVENT             = 0x7	     
} Chip_PININT_BITSLICE_CFG_T;






 
static __inline void Chip_PININT_Init(LPC_PIN_INT_T *pPININT) {}





 
static __inline void Chip_PININT_DeInit(LPC_PIN_INT_T *pPININT) {}






 
static __inline void Chip_PININT_SetPinModeEdge(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
    pPININT->ISEL &= ~(pins | (~0xff));
}






 
static __inline void Chip_PININT_SetPinModeLevel(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
    pPININT->ISEL = pins | (pPININT->ISEL & ~(~0xff));
}








 
static __inline uint32_t Chip_PININT_GetHighEnabled(LPC_PIN_INT_T *pPININT)
{
    return pPININT->IENR & ~(~0xff);
}






 
static __inline void Chip_PININT_EnableIntHigh(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
    pPININT->SIENR = pins;
}






 
static __inline void Chip_PININT_DisableIntHigh(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
    pPININT->CIENR = pins;
}








 
static __inline uint32_t Chip_PININT_GetLowEnabled(LPC_PIN_INT_T *pPININT)
{
    return pPININT->IENF & ~(~0xff);
}






 
static __inline void Chip_PININT_EnableIntLow(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
    pPININT->SIENF = pins;
}






 
static __inline void Chip_PININT_DisableIntLow(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
    pPININT->CIENF = pins;
}





 
static __inline uint32_t Chip_PININT_GetRiseStates(LPC_PIN_INT_T *pPININT)
{
    return pPININT->RISE & ~(~0xff);
}






 
static __inline void Chip_PININT_ClearRiseStates(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
		pPININT->RISE = pins;
}





 
static __inline uint32_t Chip_PININT_GetFallStates(LPC_PIN_INT_T *pPININT)
{
    return pPININT->FALL & ~(~0xff);
}






 
static __inline void Chip_PININT_ClearFallStates(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
		pPININT->FALL = pins;
}





 
static __inline uint32_t Chip_PININT_GetIntStatus(LPC_PIN_INT_T *pPININT)
{
    return pPININT->IST& ~(~0xff);
}






 
static __inline void Chip_PININT_ClearIntStatus(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
    pPININT->IST = pins;
}







 
void Chip_PININT_SetPatternMatchSrc(LPC_PIN_INT_T *pPININT, uint8_t chan, Chip_PININT_BITSLICE_T slice);








 
void Chip_PININT_SetPatternMatchConfig(LPC_PIN_INT_T *pPININT, Chip_PININT_BITSLICE_T slice, 
        Chip_PININT_BITSLICE_CFG_T slice_cfg, _Bool end_point);





 
static __inline void Chip_PININT_EnablePatternMatch(LPC_PIN_INT_T *pPININT)
{
    pPININT->PMCTRL = (1 << 0) | (pPININT->PMCTRL & ~(~0xff000003));
}





 
static __inline void Chip_PININT_DisablePatternMatch(LPC_PIN_INT_T *pPININT)
{
    pPININT->PMCTRL &= ~((1 << 0) | (~0xff000003));
}





 
static __inline void Chip_PININT_EnablePatternMatchRxEv(LPC_PIN_INT_T *pPININT)
{
    pPININT->PMCTRL = (1 << 1) | (pPININT->PMCTRL & ~(~0xff000003));
}





 
static __inline void Chip_PININT_DisablePatternMatchRxEv(LPC_PIN_INT_T *pPININT)
{
    pPININT->PMCTRL &= ~((1 << 1) | (~0xff000003));
}
    


 





#line 182 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\pmu_8xx.h"





























 











 



 
typedef struct {
	volatile uint32_t PCON;		 
	volatile uint32_t GPREG[4];	 
	volatile uint32_t DPDCTRL;	 
} LPC_PMU_T;

 





 
typedef enum CHIP_PMU_MCUPOWER {
	PMU_MCU_SLEEP = 0,		 
	PMU_MCU_DEEP_SLEEP,		 
	PMU_MCU_POWER_DOWN,		 
	PMU_MCU_DEEP_PWRDOWN	 
} CHIP_PMU_MCUPOWER_T;



 
#line 77 ".\\lpc_chip_82x\\inc\\pmu_8xx.h"



 











 
static __inline void Chip_PMU_WriteGPREG(LPC_PMU_T *pPMU, uint8_t regIndex, uint32_t value)
{
	pPMU->GPREG[regIndex] = value;
}






 
static __inline uint32_t Chip_PMU_ReadGPREG(LPC_PMU_T *pPMU, uint8_t regIndex)
{
	return pPMU->GPREG[regIndex];
}







 
void Chip_PMU_SleepState(LPC_PMU_T *pPMU);









 
void Chip_PMU_DeepSleepState(LPC_PMU_T *pPMU);










 
void Chip_PMU_PowerDownState(LPC_PMU_T *pPMU);











 
void Chip_PMU_DeepPowerDownState(LPC_PMU_T *pPMU);






 
void Chip_PMU_Sleep(LPC_PMU_T *pPMU, CHIP_PMU_MCUPOWER_T SleepMode);







 
static __inline void Chip_PMU_DisableDeepPowerDown(LPC_PMU_T *pPMU)
{
	pPMU->PCON = (1 << 3) | (pPMU->PCON & ~((0xf<<4)|(0x6<<8)|0xfffff000));
}







 
static __inline uint32_t Chip_PMU_GetSleepFlags(LPC_PMU_T *pPMU)
{
	return (pPMU->PCON & ((1 << 8) | (1 << 11)));
}








 
static __inline void Chip_PMU_ClearSleepFlags(LPC_PMU_T *pPMU, uint32_t flags)
{
	pPMU->PCON |= (flags & (~((0xf<<4)|(0x6<<8)|0xfffff000)));
}










 
static __inline void Chip_PMU_SetPowerDownControl(LPC_PMU_T *pPMU, uint32_t flags)
{
	pPMU->DPDCTRL = flags | (pPMU->DPDCTRL & ~(~0xf));
}










 
static __inline void Chip_PMU_ClearPowerDownControl(LPC_PMU_T *pPMU, uint32_t flags)
{
	pPMU->DPDCTRL &= ~(flags | (~0xf));
}



 





#line 183 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\acmp_8xx.h"





























 











 



 
typedef struct {			 
	volatile uint32_t  CTRL;	 
	volatile uint32_t  LAD;		 
} LPC_CMP_T;

 








 
#line 69 ".\\lpc_chip_82x\\inc\\acmp_8xx.h"

 
typedef enum {
	ACMP_EDGESEL_FALLING = (0 << 3),	 
	ACMP_EDGESEL_RISING  = (1 << 3),	 
	ACMP_EDGESEL_BOTH    = (2 << 3)		 
} ACMP_EDGESEL_T;

 
typedef enum {
	ACMP_HYS_NONE = (0 << 25),	 
	ACMP_HYS_5MV  = (1 << 25),	 
	ACMP_HYS_10MV = (2 << 25),	 
	ACMP_HYS_20MV = (3 << 25)	 
} ACMP_HYS_T;



 
typedef enum CHIP_ACMP_POS_INPUT {
	ACMP_POSIN_VLO      = (0 << 8),	 
	ACMP_POSIN_ACMP_I1  = (1 << 8),	 
	ACMP_POSIN_ACMP_I2  = (2 << 8),	 
	ACMP_POSIN_ACMP_I3  = (3 << 8),	 
	ACMP_POSIN_ACMP_I4  = (4 << 8),	 

	ACMP_POSIN_INT_REF  = (5 << 8),	 
	ACMP_POSIN_ADC_0    = (6 << 8),	 



} ACMP_POS_INPUT_T;



 
typedef enum CHIP_ACMP_NEG_INPUT {
	ACMP_NEGIN_VLO     = (0 << 11),	 
	ACMP_NEGIN_ACMP_I1 = (1 << 11),	 
	ACMP_NEGIN_ACMP_I2 = (2 << 11),	 
	ACMP_NEGIN_ACMP_I3 = (3 << 11),	 
	ACMP_NEGIN_ACMP_I4 = (4 << 11),	 

	ACMP_NEGIN_INT_REF = (5 << 11),	 
	ACMP_NEGIN_ADC_0   = (6 << 11),	 



} ACMP_NEG_INPUT_T;





 
void Chip_ACMP_Init(LPC_CMP_T *pACMP);





 
void Chip_ACMP_Deinit(LPC_CMP_T *pACMP);





 
static __inline uint32_t Chip_ACMP_GetCompStatus(LPC_CMP_T *pACMP)
{
	return pACMP->CTRL & ((1 << 21) | (1 << 23));
}





 
void Chip_ACMP_EdgeClear(LPC_CMP_T *pACMP);






 
void Chip_ACMP_SetEdgeSelection(LPC_CMP_T *pACMP, ACMP_EDGESEL_T edgeSel);





 
static __inline void Chip_ACMP_EnableSyncCompOut(LPC_CMP_T *pACMP)
{
	pACMP->CTRL = (1 << 6) | (pACMP->CTRL & ~(7|(1<<5)|(1<<7)|(0x3f<<14)|(1<<22)|(1<<24)|(0x1fu<<27)));
}





 
static __inline void Chip_ACMP_DisableSyncCompOut(LPC_CMP_T *pACMP)
{
	pACMP->CTRL &= ~((1 << 6) | (7|(1<<5)|(1<<7)|(0x3f<<14)|(1<<22)|(1<<24)|(0x1fu<<27)));
}






 
void Chip_ACMP_SetPosVoltRef(LPC_CMP_T *pACMP, ACMP_POS_INPUT_T Posinput);






 
void Chip_ACMP_SetNegVoltRef(LPC_CMP_T *pACMP, ACMP_NEG_INPUT_T Neginput);






 
void Chip_ACMP_SetHysteresis(LPC_CMP_T *pACMP, ACMP_HYS_T hys);









 
void Chip_ACMP_SetupAMCPRefs(LPC_CMP_T *pACMP, ACMP_EDGESEL_T edgeSel,
							 ACMP_POS_INPUT_T Posinput, ACMP_NEG_INPUT_T Neginput,
							 ACMP_HYS_T hys);








 
void Chip_ACMP_SetupVoltLadder(LPC_CMP_T *pACMP, uint32_t ladsel, _Bool ladrefVDDCMP);





 
static __inline void Chip_ACMP_EnableVoltLadder(LPC_CMP_T *pACMP)
{
	pACMP->LAD = (1 << 0) | (pACMP->LAD & ~(~0x7f));
}





 
static __inline void Chip_ACMP_DisableVoltLadder(LPC_CMP_T *pACMP)
{
	pACMP->LAD &= ~((1 << 0) | (~0x7f));
}



 





#line 184 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\crc_8xx.h"





























 











 



 
typedef struct {					 
	volatile    uint32_t    MODE;		 
	volatile    uint32_t    SEED;		 
	union {
		volatile const     uint32_t    SUM;	 
		volatile     uint32_t    WRDATA32;	 
		volatile     uint16_t    WRDATA16;	 
		volatile     uint8_t     WRDATA8;	 
	};

} LPC_CRC_T;



 
#line 72 ".\\lpc_chip_82x\\inc\\crc_8xx.h"











 
typedef enum IP_CRC_001_POLY {
	CRC_POLY_CCITT = (0x00),	 
	CRC_POLY_CRC16 = (0x01),	 
	CRC_POLY_CRC32 = (0x02),	 
	CRC_POLY_LAST,
} CRC_POLY_T;




 
void Chip_CRC_Init(void);




 
void Chip_CRC_Deinit(void);








 
static __inline void Chip_CRC_SetPoly(CRC_POLY_T poly, uint32_t flags)
{
	((LPC_CRC_T *) (0x50000000UL))->MODE = (uint32_t) poly | flags;
}




 
static __inline void Chip_CRC_UseCRC16(void)
{
	((LPC_CRC_T *) (0x50000000UL))->MODE = (0x15);
	((LPC_CRC_T *) (0x50000000UL))->SEED = (0x00000000);
}




 
static __inline void Chip_CRC_UseCRC32(void)
{
	((LPC_CRC_T *) (0x50000000UL))->MODE = (0x36);
	((LPC_CRC_T *) (0x50000000UL))->SEED = (0xFFFFFFFF);
}




 
static __inline void Chip_CRC_UseCCITT(void)
{
	((LPC_CRC_T *) (0x50000000UL))->MODE = (0x00);
	((LPC_CRC_T *) (0x50000000UL))->SEED = (0x0000FFFF);
}





 
void Chip_CRC_UseDefaultConfig(CRC_POLY_T poly);





 
static __inline void Chip_CRC_SetMode(uint32_t mode)
{
	((LPC_CRC_T *) (0x50000000UL))->MODE = mode;
}




 
static __inline uint32_t Chip_CRC_GetMode(void)
{
	return ((LPC_CRC_T *) (0x50000000UL))->MODE;
}





 
static __inline void Chip_CRC_SetSeed(uint32_t seed)
{
	((LPC_CRC_T *) (0x50000000UL))->SEED = seed;
}




 
static __inline uint32_t Chip_CRC_GetSeed(void)
{
	return ((LPC_CRC_T *) (0x50000000UL))->SEED;
}





 
static __inline void Chip_CRC_Write8(uint8_t data)
{
	((LPC_CRC_T *) (0x50000000UL))->WRDATA8 = data;
}





 
static __inline void Chip_CRC_Write16(uint16_t data)
{
	((LPC_CRC_T *) (0x50000000UL))->WRDATA16 = data;
}





 
static __inline void Chip_CRC_Write32(uint32_t data)
{
	((LPC_CRC_T *) (0x50000000UL))->WRDATA32 = data;
}




 
static __inline uint32_t Chip_CRC_Sum(void)
{
	return ((LPC_CRC_T *) (0x50000000UL))->SUM;
}






 
uint32_t Chip_CRC_CRC8(const uint8_t *data, uint32_t bytes);






 
uint32_t Chip_CRC_CRC16(const uint16_t *data, uint32_t hwords);






 
uint32_t Chip_CRC_CRC32(const uint32_t *data, uint32_t words);



 





#line 185 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\gpio_8xx.h"





























 











 



 
typedef struct {				 
	volatile uint8_t B[128][32];	 
	volatile uint32_t W[32][32];	 
	volatile uint32_t DIR[32];		 
	volatile uint32_t MASK[32];		 
	volatile uint32_t PIN[32];		 
	volatile uint32_t MPIN[32];		 
	volatile uint32_t SET[32];		 
	volatile  uint32_t CLR[32];		 
	volatile  uint32_t NOT[32];		 
	volatile  uint32_t DIRSET[32];    
	volatile  uint32_t DIRCLR[32];    
	volatile  uint32_t DIRNOT[32];    
} LPC_GPIO_T;





 
void Chip_GPIO_Init(LPC_GPIO_T *pGPIO);





 
void Chip_GPIO_DeInit(LPC_GPIO_T *pGPIO);









 
static __inline void Chip_GPIO_WritePortBit(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t pin, _Bool setting)
{
	pGPIO->B[port][pin] = setting;
}









 
static __inline void Chip_GPIO_SetPinState(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin, _Bool setting)
{
	pGPIO->B[port][pin] = setting;
}








 
static __inline _Bool Chip_GPIO_ReadPortBit(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t pin)
{
	return (_Bool) pGPIO->B[port][pin];
}








 
static __inline _Bool Chip_GPIO_GetPinState(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
	return (_Bool) pGPIO->B[port][pin];
}










 
static __inline void Chip_GPIO_WriteDirBit(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t pin, _Bool setting)
{
	if (setting) {
		pGPIO->DIR[port] |= 1UL << pin;
	}
	else {
		pGPIO->DIR[port] &= ~(1UL << pin);
	}
}







 
static __inline void Chip_GPIO_SetPinDIROutput(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{

	pGPIO->DIRSET[port] = 1UL << pin;



}







 
static __inline void Chip_GPIO_SetPinDIRInput(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{

	pGPIO->DIRCLR[port] = 1UL << pin;



}







 
static __inline void Chip_GPIO_TogglePinDIR(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{

	pGPIO->DIRNOT[port] = 1UL << pin;



}








 
static __inline void Chip_GPIO_SetPinDIR(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin, _Bool output)
{
	if (output) {
		Chip_GPIO_SetPinDIROutput(pGPIO, port, pin);
	}
	else {
		Chip_GPIO_SetPinDIRInput(pGPIO, port, pin);
	}
}








 
static __inline _Bool Chip_GPIO_ReadDirBit(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t bit)
{
	return (_Bool) (((pGPIO->DIR[port]) >> bit) & 1);
}







 
static __inline _Bool Chip_GPIO_GetPinDIR(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
	return Chip_GPIO_ReadDirBit(pGPIO, port, pin);
}










 
static __inline void Chip_GPIO_SetDir(LPC_GPIO_T *pGPIO, uint8_t portNum, uint32_t bitValue, uint8_t out)
{
	if (out) {
		pGPIO->DIR[portNum] |= bitValue;
	}
	else {
		pGPIO->DIR[portNum] &= ~bitValue;
	}
}









 
static __inline void Chip_GPIO_SetPortDIROutput(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pinMask)
{

	pGPIO->DIRSET[port] = pinMask;



}









 
static __inline void Chip_GPIO_SetPortDIRInput(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pinMask)
{

	pGPIO->DIRCLR[port] = pinMask;



}









 
static __inline void Chip_GPIO_TogglePortDIR(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pinMask)
{

	pGPIO->DIRNOT[port] = pinMask;



}










 
static __inline void Chip_GPIO_SetPortDIR(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pinMask, _Bool outSet)
{
	if (outSet) {
		Chip_GPIO_SetPortDIROutput(pGPIO, port, pinMask);
	}
	else {
		Chip_GPIO_SetPortDIRInput(pGPIO, port, pinMask);
	}
}












 
static __inline void Chip_GPIO_SetPortDIRMask(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pinMask, _Bool outMask)
{
	Chip_GPIO_SetPortDIRInput(pGPIO, port, pinMask & ~outMask);
	Chip_GPIO_SetPortDIROutput(pGPIO, port, pinMask & outMask);
}








 
static __inline uint32_t Chip_GPIO_GetPortDIR(LPC_GPIO_T *pGPIO, uint8_t port)
{
	return pGPIO->DIR[port];
}










 
static __inline void Chip_GPIO_SetPortMask(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t mask)
{
	pGPIO->MASK[port] = mask;
}








 
static __inline uint32_t Chip_GPIO_GetPortMask(LPC_GPIO_T *pGPIO, uint8_t port)
{
	return pGPIO->MASK[port];
}







 
static __inline void Chip_GPIO_SetPortValue(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t value)
{
	pGPIO->PIN[port] = value;
}






 
static __inline uint32_t Chip_GPIO_GetPortValue(LPC_GPIO_T *pGPIO, uint8_t port)
{
	return pGPIO->PIN[port];
}







 
static __inline void Chip_GPIO_SetMaskedPortValue(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t value)
{
	pGPIO->MPIN[port] = value;
}






 
static __inline uint32_t Chip_GPIO_GetMaskedPortValue(LPC_GPIO_T *pGPIO, uint8_t port)
{
	return pGPIO->MPIN[port];
}










 
static __inline void Chip_GPIO_SetValue(LPC_GPIO_T *pGPIO, uint8_t portNum, uint32_t bitValue)
{
	pGPIO->SET[portNum] = bitValue;
}









 
static __inline void Chip_GPIO_SetPortOutHigh(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pins)
{
	pGPIO->SET[port] = pins;
}









 
static __inline void Chip_GPIO_SetPinOutHigh(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
	pGPIO->SET[port] = (1 << pin);
}










 
static __inline void Chip_GPIO_ClearValue(LPC_GPIO_T *pGPIO, uint8_t portNum, uint32_t bitValue)
{
	pGPIO->CLR[portNum] = bitValue;
}









 
static __inline void Chip_GPIO_SetPortOutLow(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pins)
{
	pGPIO->CLR[port] = pins;
}









 
static __inline void Chip_GPIO_SetPinOutLow(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
	pGPIO->CLR[port] = (1 << pin);
}









 
static __inline void Chip_GPIO_PortToggle(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pins)
{
	pGPIO->NOT[port] = pins;
}









 
static __inline void Chip_GPIO_SetPinToggle(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
	pGPIO->NOT[port] = (1 << pin);
}









 
static __inline uint32_t Chip_GPIO_ReadValue(LPC_GPIO_T *pGPIO, uint8_t portNum)
{
	return pGPIO->PIN[portNum];
}



 





#line 186 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\mrt_8xx.h"





























 











 



 





 
typedef struct {
	volatile uint32_t INTVAL;	 
	volatile  uint32_t TIMER;	 
	volatile uint32_t CTRL;		 
	volatile uint32_t STAT;		 
} LPC_MRT_CH_T;



 
typedef struct {
	LPC_MRT_CH_T CHANNEL[(4)];
	uint32_t unused[45];
	volatile  uint32_t IDLE_CH;
	volatile uint32_t IRQ_FLAG;
} LPC_MRT_T;

 





 
typedef enum MRT_MODE {
	MRT_MODE_REPEAT =  (0 << 1),	 
	MRT_MODE_ONESHOT = (1 << 1)		 
} MRT_MODE_T;



 
 



 



 



 






 









 
static __inline void Chip_MRT_Init(void)
{
	 
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_MRT);

	 
	Chip_SYSCTL_PeriphReset(RESET_MRT);
}




 
static __inline void Chip_MRT_DeInit(void)
{
	 
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_MRT);
}





 
static __inline LPC_MRT_CH_T *Chip_MRT_GetRegPtr(uint8_t ch)
{
	return ((LPC_MRT_CH_T *) &((LPC_MRT_T *) (0x40004000UL))->CHANNEL[(ch)]);
}





 
static __inline uint32_t Chip_MRT_GetInterval(LPC_MRT_CH_T *pMRT)
{
	return pMRT->INTVAL;
}











 
static __inline void Chip_MRT_SetInterval(LPC_MRT_CH_T *pMRT, uint32_t interval)
{
	pMRT->INTVAL = interval;
}





 
static __inline uint32_t Chip_MRT_GetTimer(LPC_MRT_CH_T *pMRT)
{
	return pMRT->TIMER;
}





 
static __inline _Bool Chip_MRT_GetEnabled(LPC_MRT_CH_T *pMRT)
{
	return (_Bool) ((pMRT->CTRL & (0x01)) != 0);
}





 
static __inline void Chip_MRT_SetEnabled(LPC_MRT_CH_T *pMRT)
{
	pMRT->CTRL = (0x01) | (pMRT->CTRL & ~(~7));
}





 
static __inline void Chip_MRT_SetDisabled(LPC_MRT_CH_T *pMRT)
{
	pMRT->CTRL &= ~((0x01) | (~7));
}





 
static __inline MRT_MODE_T Chip_MRT_GetMode(LPC_MRT_CH_T *pMRT)
{
	return (MRT_MODE_T) (pMRT->CTRL & (0x06));
}






 
static __inline void Chip_MRT_SetMode(LPC_MRT_CH_T *pMRT, MRT_MODE_T mode)
{
	uint32_t reg;

	reg = pMRT->CTRL & ~((0x06) | (~7));
	pMRT->CTRL = reg | (uint32_t) mode;
}





 
static __inline _Bool Chip_MRT_IsRepeatMode(LPC_MRT_CH_T *pMRT)
{
	return ((pMRT->CTRL & (0x06)) != 0) ? 0 : 1;
}





 
static __inline _Bool Chip_MRT_IsOneShotMode(LPC_MRT_CH_T *pMRT)
{
	return ((pMRT->CTRL & (0x06)) != 0) ? 1 : 0;
}





 
static __inline _Bool Chip_MRT_IntPending(LPC_MRT_CH_T *pMRT)
{
	return (_Bool) ((pMRT->STAT & (0x01)) != 0);
}





 
static __inline void Chip_MRT_IntClear(LPC_MRT_CH_T *pMRT)
{
	pMRT->STAT = (0x01) | (pMRT->STAT & ~(~3));
}





 
static __inline _Bool Chip_MRT_Running(LPC_MRT_CH_T *pMRT)
{
	return (_Bool) ((pMRT->STAT & (0x02)) != 0);
}




 
static __inline uint8_t Chip_MRT_GetIdleChannel(void)
{
	return (uint8_t) (((LPC_MRT_T *) (0x40004000UL))->IDLE_CH);
}




 
static __inline uint8_t Chip_MRT_GetIdleChannelShifted(void)
{
	return (uint8_t) (Chip_MRT_GetIdleChannel() >> 4);
}




 
static __inline uint32_t Chip_MRT_GetIntPending(void)
{
	return ((LPC_MRT_T *) (0x40004000UL))->IRQ_FLAG;
}





 
static __inline _Bool Chip_MRT_GetIntPendingByChannel(uint8_t ch)
{
	return (_Bool) (((((LPC_MRT_T *) (0x40004000UL))->IRQ_FLAG >> ch) & 1) != 0);
}









 
static __inline void Chip_MRT_ClearIntPending(uint32_t mask)
{
	((LPC_MRT_T *) (0x40004000UL))->IRQ_FLAG = mask;
}



 





#line 187 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\uart_8xx.h"





























 








#line 1 ".\\lpc_chip_82x\\inc\\ring_buffer.h"





























 




#line 36 ".\\lpc_chip_82x\\inc\\ring_buffer.h"




 



 
typedef struct {
	void *data;
	int count;
	int itemSz;
	uint32_t head;
	uint32_t tail;
} RINGBUFF_T;




 





 












 
int RingBuffer_Init(RINGBUFF_T *RingBuff, void *buffer, int itemSize, int count);





 
static __inline void RingBuffer_Flush(RINGBUFF_T *RingBuff)
{
	RingBuff->head = RingBuff->tail = 0;
}





 
static __inline int RingBuffer_GetSize(RINGBUFF_T *RingBuff)
{
	return RingBuff->count;
}





 
static __inline int RingBuffer_GetCount(RINGBUFF_T *RingBuff)
{
	return (*(volatile uint32_t *) &(RingBuff)->head) - (*(volatile uint32_t *) &(RingBuff)->tail);
}





 
static __inline int RingBuffer_GetFree(RINGBUFF_T *RingBuff)
{
	return RingBuff->count - RingBuffer_GetCount(RingBuff);
}





 
static __inline int RingBuffer_IsFull(RINGBUFF_T *RingBuff)
{
	return (RingBuffer_GetCount(RingBuff) >= RingBuff->count);
}





 
static __inline int RingBuffer_IsEmpty(RINGBUFF_T *RingBuff)
{
	return (*(volatile uint32_t *) &(RingBuff)->head) == (*(volatile uint32_t *) &(RingBuff)->tail);
}









 
int RingBuffer_Insert(RINGBUFF_T *RingBuff, const void *data);










 
int RingBuffer_InsertMult(RINGBUFF_T *RingBuff, const void *data, int num);









 
int RingBuffer_Pop(RINGBUFF_T *RingBuff, void *data);









 
int RingBuffer_PopMult(RINGBUFF_T *RingBuff, void *data, int num);




 

#line 40 ".\\lpc_chip_82x\\inc\\uart_8xx.h"




 



 
typedef struct {
	volatile uint32_t  CFG;				 
	volatile uint32_t  CTRL;			 
	volatile uint32_t  STAT;			 
	volatile uint32_t  INTENSET;		 
	volatile  uint32_t  INTENCLR;		 
	volatile const  uint32_t  RXDATA;			 
	volatile const  uint32_t  RXDATA_STAT;		 
	volatile uint32_t  TXDATA;			 
	volatile uint32_t  BRG;				 
	volatile uint32_t  INTSTAT;			 
	volatile uint32_t  OSR;              
	volatile uint32_t  ADDR;             
} LPC_USART_T;



 
#line 81 ".\\lpc_chip_82x\\inc\\uart_8xx.h"

#line 93 ".\\lpc_chip_82x\\inc\\uart_8xx.h"



 
#line 108 ".\\lpc_chip_82x\\inc\\uart_8xx.h"



 
#line 132 ".\\lpc_chip_82x\\inc\\uart_8xx.h"



 
#line 155 ".\\lpc_chip_82x\\inc\\uart_8xx.h"





 
static __inline void Chip_UART_Enable(LPC_USART_T *pUART)
{
	pUART->CFG = (0x01 << 0) | (pUART->CFG & ~((1<<1)|(1<<7)|(1<<8)|(1<<10)|(1<<13)|(3 << 16)|(0xffu<<24)));
}





 
static __inline void Chip_UART_Disable(LPC_USART_T *pUART)
{
	pUART->CFG &= ~(((1<<1)|(1<<7)|(1<<8)|(1<<10)|(1<<13)|(3 << 16)|(0xffu<<24)) | (0x01 << 0));
}





 
static __inline void Chip_UART_TXEnable(LPC_USART_T *pUART)
{
	pUART->CTRL &= ~((0xFFFEFCB9U) | (0x01 << 6));
}





 
static __inline void Chip_UART_TXDisable(LPC_USART_T *pUART)
{
	pUART->CTRL = (0x01 << 6) | (pUART->CTRL & ~(0xFFFEFCB9U));
}








 
static __inline void Chip_UART_SendByte(LPC_USART_T *pUART, uint8_t data)
{
	pUART->TXDATA = (uint32_t) data;
}








 
static __inline uint32_t Chip_UART_ReadByte(LPC_USART_T *pUART)
{
	 
	return (uint32_t) (pUART->RXDATA & 0x000001FF);
}








 
static __inline void Chip_UART_IntEnable(LPC_USART_T *pUART, uint32_t intMask)
{
	pUART->INTENSET = intMask;
}








 
static __inline void Chip_UART_IntDisable(LPC_USART_T *pUART, uint32_t intMask)
{
	pUART->INTENCLR = intMask;
}








 
static __inline uint32_t Chip_UART_GetIntsEnabled(LPC_USART_T *pUART)
{
	return (pUART->INTENSET & ~((1<<1)|(1<<4)|(1<<7)|(3<<9)|(0xfffeu<<16)));
}








 
static __inline uint32_t Chip_UART_GetIntStatus(LPC_USART_T *pUART)
{
	return (pUART->INTSTAT & ~((1<<1)|(1<<4)|(1<<7)|(3<<9)|(0xfffeu<<16)));
}











 
static __inline void Chip_UART_ConfigData(LPC_USART_T *pUART, uint32_t config)
{
	uint32_t reg;

	reg = pUART->CFG & ~((0x3 << 2) | (0x3 << 4) | (0x1 << 6) | ((1<<1)|(1<<7)|(1<<8)|(1<<10)|(1<<13)|(3 << 16)|(0xffu<<24)));
	pUART->CFG = reg | config;
}








 
static __inline uint32_t Chip_UART_GetStatus(LPC_USART_T *pUART)
{
	return (pUART->STAT & ~((1<<7)|(1<<9)|(0xFFFEU<<16)));
}









 
static __inline void Chip_UART_ClearStatus(LPC_USART_T *pUART, uint32_t stsMask)
{
	pUART->STAT = stsMask;
}







 
static __inline void Chip_UART_SetOSR(LPC_USART_T *pUART, uint32_t ovrVal)
{
	pUART->OSR = ovrVal - 1;
}







 
static __inline void Chip_UART_SetAddr(LPC_USART_T *pUART, uint32_t addr)
{
	pUART->ADDR = addr;
}





 
void Chip_UART_Init(LPC_USART_T *pUART);





 
void Chip_UART_DeInit(LPC_USART_T *pUART);











 
int Chip_UART_Send(LPC_USART_T *pUART, const void *data, int numBytes);










 
int Chip_UART_Read(LPC_USART_T *pUART, void *data, int numBytes);






 
void Chip_UART_SetBaud(LPC_USART_T *pUART, uint32_t baudrate);









 
int Chip_UART_SendBlocking(LPC_USART_T *pUART, const void *data, int numBytes);










 
int Chip_UART_ReadBlocking(LPC_USART_T *pUART, void *data, int numBytes);









 
void Chip_UART_RXIntHandlerRB(LPC_USART_T *pUART, RINGBUFF_T *pRB);









 
void Chip_UART_TXIntHandlerRB(LPC_USART_T *pUART, RINGBUFF_T *pRB);











 
uint32_t Chip_UART_SendRB(LPC_USART_T *pUART, RINGBUFF_T *pRB, const void *data, int count);











 
int Chip_UART_ReadRB(LPC_USART_T *pUART, RINGBUFF_T *pRB, void *data, int bytes);










 
void Chip_UART_IRQRBHandler(LPC_USART_T *pUART, RINGBUFF_T *pRXRB, RINGBUFF_T *pTXRB);



 





#line 188 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\wkt_8xx.h"





























 











 



 
typedef struct {
	volatile uint32_t  CTRL;	 
	uint32_t  Reserved[2];
	volatile uint32_t  COUNT;	 
} LPC_WKT_T;





 






 
typedef enum  {
	WKT_CLKSRC_DIVIRC = 0,	 
	WKT_CLKSRC_10KHZ = 1	 
} WKT_CLKSRC_T;





 
static __inline WKT_CLKSRC_T Chip_WKT_GetClockSource(LPC_WKT_T *pWKT)
{
	return (WKT_CLKSRC_T) (pWKT->CTRL & ((uint32_t) (1 << 0)));
}






 
void Chip_WKT_SetClockSource(LPC_WKT_T *pWKT, WKT_CLKSRC_T clkSrc);





 
uint32_t Chip_WKT_GetClockRate(LPC_WKT_T *pWKT);





 
static __inline _Bool Chip_WKT_GetIntStatus(LPC_WKT_T *pWKT)
{
	return (_Bool) ((pWKT->CTRL & ((uint32_t) (1 << 1))) != 0);
}





 
static __inline void Chip_WKT_ClearIntStatus(LPC_WKT_T *pWKT)
{
	pWKT->CTRL = ((uint32_t) (1 << 1)) | (pWKT->CTRL & ~(~7));
}





 
static __inline void Chip_WKT_Stop(LPC_WKT_T *pWKT)
{
	pWKT->CTRL = ((uint32_t) (1 << 2)) | (pWKT->CTRL & ~(~7));
}







 
static __inline void Chip_WKT_LoadCount(LPC_WKT_T *pWKT, uint32_t count)
{
	pWKT->COUNT = count;
}








 
void Chip_WKT_Start(LPC_WKT_T *pWKT, WKT_CLKSRC_T clkSrc, uint32_t cntVal);



 





#line 189 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\wwdt_8xx.h"





























 











 

 




 
typedef struct {				 
	volatile uint32_t  MOD;			 
	volatile uint32_t  TC;			 
	volatile  uint32_t  FEED;		 
	volatile const  uint32_t  TV;			 
	volatile const  uint32_t  RESERVED0;
	volatile uint32_t  WARNINT;		 
	volatile uint32_t  WINDOW;		 
} LPC_WWDT_T;

 
#line 67 ".\\lpc_chip_82x\\inc\\wwdt_8xx.h"



 
 

 

 

 

 






 
void Chip_WWDT_Init(LPC_WWDT_T *pWWDT);





 
void Chip_WWDT_DeInit(LPC_WWDT_T *pWWDT);






 
static __inline void Chip_WWDT_SetTimeOut(LPC_WWDT_T *pWWDT, uint32_t timeout)
{
	pWWDT->TC = timeout;
}







 
static __inline void Chip_WWDT_Feed(LPC_WWDT_T *pWWDT)
{
	pWWDT->FEED = 0xAA;
	pWWDT->FEED = 0x55;
}








 
static __inline void Chip_WWDT_SetWarning(LPC_WWDT_T *pWWDT, uint32_t timeout)
{
	pWWDT->WARNINT = timeout;
}









 
static __inline void Chip_WWDT_SetWindow(LPC_WWDT_T *pWWDT, uint32_t timeout)
{
	pWWDT->WINDOW = timeout;
}











 
static __inline void Chip_WWDT_SetOption(LPC_WWDT_T *pWWDT, uint32_t options)
{
	pWWDT->MOD = options | (pWWDT->MOD & ~(~0x3f));
}









 
static __inline void Chip_WWDT_UnsetOption(LPC_WWDT_T *pWWDT, uint32_t options)
{
	pWWDT->MOD &= (~options) & ((uint32_t) 0x1F);
}





 
static __inline void Chip_WWDT_Start(LPC_WWDT_T *pWWDT)
{
	Chip_WWDT_SetOption(pWWDT, ((uint32_t) (1 << 0)));
	Chip_WWDT_Feed(pWWDT);
}





 
static __inline uint32_t Chip_WWDT_GetStatus(LPC_WWDT_T *pWWDT)
{
	return pWWDT->MOD;
}








 
void Chip_WWDT_ClearStatusFlag(LPC_WWDT_T *pWWDT, uint32_t status);





 
static __inline uint32_t Chip_WWDT_GetCurrentCount(LPC_WWDT_T *pWWDT)
{
	return pWWDT->TV;
}



 





#line 190 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\sct_8xx.h"





























 











 



 






 
typedef struct {
	volatile  uint32_t CONFIG;				 
	union {
		volatile uint32_t CTRL_U;			 
		struct {
			volatile uint16_t CTRL_L;		 
			volatile uint16_t CTRL_H;		 
		};
	};
	union {
		volatile uint32_t LIMIT_U;			 
		struct {
			volatile uint16_t LIMIT_L;		 
			volatile uint16_t LIMIT_H;		 
		};
	};

	union {
		volatile uint32_t HALT_U;			 
		struct {
			volatile uint16_t HALT_L;		 
			volatile uint16_t HALT_H;		 
		};
	};

	union {
		volatile uint32_t STOP_U;			 
		struct {
			volatile uint16_t STOP_L;		 
			volatile uint16_t STOP_H;		 
		};

	};

	union {
		volatile uint32_t START_U;			 
		struct {
			volatile uint16_t START_L;		 
			volatile uint16_t START_H;		 
		};

	};

	uint32_t RESERVED1[10];				 

	union {
		volatile uint32_t COUNT_U;			 
		struct {
			volatile uint16_t COUNT_L;		 
			volatile uint16_t COUNT_H;		 
		};
	};

	union {
		volatile uint32_t STATE_U;			 
		struct {
			volatile uint16_t STATE_L;		 
			volatile uint16_t STATE_H;		 
		};
	};

	volatile const  uint32_t INPUT;				 
	union {
		volatile uint32_t REGMODE_U;		 
		struct {
			volatile uint16_t REGMODE_L;	 
			volatile uint16_t REGMODE_H;	 
		};
	};

	volatile uint32_t OUTPUT;				 
	volatile uint32_t OUTPUTDIRCTRL;		 
	volatile uint32_t RES;					 
	volatile uint32_t DMAREQ0;				 
	volatile uint32_t DMAREQ1;				 

	uint32_t RESERVED2[35];				 

	volatile uint32_t EVEN;					 
	volatile uint32_t EVFLAG;				 
	volatile uint32_t CONEN;				 
	volatile uint32_t CONFLAG;				 
	union {
		volatile union {					 
			uint32_t U;					 
			struct {
				uint16_t L;				 
				uint16_t H;				 
			};
		} MATCH[(8)];

		volatile const union {
			uint32_t U;					 
			struct {
				uint16_t L;				 
				uint16_t H;				 
			};
		} CAP[(8)];
	};

	uint32_t RESERVED3[56];				 

	union {
		volatile union {					 
			uint32_t U;					 
			struct {
				uint16_t L;				 
				uint16_t H;				 
			};
		} MATCHREL[(8)];

		volatile union {
			uint32_t U;					 
			struct {
				uint16_t L;				 
				uint16_t H;				 
			};
		} CAPCTRL[(8)];
	};

	uint32_t RESERVED4[56];				 

	volatile struct {						 
		uint32_t STATE;					 
		uint32_t CTRL;					 
	} EV[(8)];

	uint32_t RESERVED5[112];			 

	volatile struct {						 
		uint32_t SET;					 
		uint32_t CLR;					 
	} OUT[(6)];

} LPC_SCT_T;

 
#line 212 ".\\lpc_chip_82x\\inc\\sct_8xx.h"



 


















 









#line 251 ".\\lpc_chip_82x\\inc\\sct_8xx.h"



 







 
typedef enum CHIP_SCT_MATCH_REG {
	SCT_MATCH_0 = 0,	 
	SCT_MATCH_1 = 1,	 
	SCT_MATCH_2 = 2,	 
	SCT_MATCH_3 = 3,	 
	SCT_MATCH_4 = 4		 
} CHIP_SCT_MATCH_REG_T;



 
typedef enum CHIP_SCT_EVENT {
	SCT_EVT_0  = (1 << 0),	 
	SCT_EVT_1  = (1 << 1),	 
	SCT_EVT_2  = (1 << 2),	 
	SCT_EVT_3  = (1 << 3),	 
	SCT_EVT_4  = (1 << 4)	 
} CHIP_SCT_EVENT_T;






 
static __inline void Chip_SCT_Config(LPC_SCT_T *pSCT, uint32_t value)
{
	pSCT->CONFIG = value;
}












 
void Chip_SCT_SetClrControl(LPC_SCT_T *pSCT, uint32_t value, FunctionalState ena);














 
void Chip_SCT_SetConflictResolution(LPC_SCT_T *pSCT, uint8_t outnum, uint8_t value);






 
static __inline void Chip_SCT_SetCount(LPC_SCT_T *pSCT, uint32_t count)
{
	pSCT->COUNT_U = count;
}






 
static __inline void Chip_SCT_SetCountL(LPC_SCT_T *pSCT, uint16_t count)
{
	pSCT->COUNT_L = count;
}






 
static __inline void Chip_SCT_SetCountH(LPC_SCT_T *pSCT, uint16_t count)
{
	pSCT->COUNT_H = count;
}







 
static __inline void Chip_SCT_SetMatchCount(LPC_SCT_T *pSCT, CHIP_SCT_MATCH_REG_T n, uint32_t value)
{
	pSCT->MATCH[n].U = value;
}







 
static __inline void Chip_SCT_SetMatchReload(LPC_SCT_T *pSCT, CHIP_SCT_MATCH_REG_T n, uint32_t value)
{
	pSCT->MATCHREL[n].U = value;
}






 
static __inline void Chip_SCT_EnableEventInt(LPC_SCT_T *pSCT, CHIP_SCT_EVENT_T evt)
{
	pSCT->EVEN = evt | (pSCT->EVEN & ~(~0x3f));
}






 
static __inline void Chip_SCT_DisableEventInt(LPC_SCT_T *pSCT, CHIP_SCT_EVENT_T evt)
{
	pSCT->EVEN &= ~(evt | (~0x3f));
}






 
static __inline void Chip_SCT_ClearEventFlag(LPC_SCT_T *pSCT, CHIP_SCT_EVENT_T evt)
{
	pSCT->EVFLAG = evt | (pSCT->EVFLAG & ~(~0x3f));
}






 
static __inline void Chip_SCT_SetControl(LPC_SCT_T *pSCT, uint32_t value)
{
	pSCT->CTRL_U = value | (pSCT->CTRL_U & ~((7<<13)|(7u<<29)));
}






 
static __inline void Chip_SCT_ClearControl(LPC_SCT_T *pSCT, uint32_t value)
{
	pSCT->CTRL_U &= ~(value | ((7<<13)|(7u<<29)));
}





 
void Chip_SCT_Init(LPC_SCT_T *pSCT);





 
void Chip_SCT_DeInit(LPC_SCT_T *pSCT);



 






#line 191 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\sct_pwm_8xx.h"





























 
















 







 
static __inline uint32_t Chip_SCTPWM_GetTicksPerCycle(LPC_SCT_T *pSCT)
{
	return pSCT->MATCHREL[0].U;
}










 
static __inline uint32_t Chip_SCTPWM_PercentageToTicks(LPC_SCT_T *pSCT, uint8_t percent)
{
	return (Chip_SCTPWM_GetTicksPerCycle(pSCT) * percent) / 100;
}










 
static __inline uint32_t Chip_SCTPWM_GetDutyCycle(LPC_SCT_T *pSCT, uint8_t index)
{
	return pSCT->MATCHREL[index].U;
}












 
static __inline void Chip_SCTPWM_SetDutyCycle(LPC_SCT_T *pSCT, uint8_t index, uint32_t ticks)
{
	Chip_SCT_SetMatchReload(pSCT, (CHIP_SCT_MATCH_REG_T)index, ticks);
}





 
static __inline void Chip_SCTPWM_Init(LPC_SCT_T *pSCT)
{
	Chip_SCT_Init(pSCT);
}











 
static __inline void Chip_SCTPWM_Start(LPC_SCT_T *pSCT)
{
	Chip_SCT_ClearControl(pSCT, (1 << 2) | (1 << 18));
}





 
static __inline void Chip_SCTPWM_Stop(LPC_SCT_T *pSCT)
{
	 
	Chip_SCT_SetControl(pSCT, (1 << 2) | (1 << 18));

	 
	Chip_SCT_SetControl(pSCT, (1 << 3) | (1 << 19));
}






 
void Chip_SCTPWM_SetRate(LPC_SCT_T *pSCT, uint32_t freq);











 
void Chip_SCTPWM_SetOutPin(LPC_SCT_T *pSCT, uint8_t index, uint8_t pin);



 






#line 192 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\spi_8xx.h"





























 




#line 1 ".\\lpc_chip_82x\\inc\\spi_common_8xx.h"





























 











 


 
typedef struct {					 
	volatile uint32_t  CFG;				 
	volatile uint32_t  DLY;				 
	volatile uint32_t  STAT;			 
	volatile uint32_t  INTENSET;		 
	volatile  uint32_t  INTENCLR;		 
	volatile const  uint32_t  RXDAT;			 
	volatile uint32_t  TXDATCTL;		 
	volatile uint32_t  TXDAT;			 
	volatile uint32_t  TXCTRL;			 
	volatile uint32_t  DIV;				 
	volatile const  uint32_t  INTSTAT;			 
} LPC_SPI_T;

 
#line 72 ".\\lpc_chip_82x\\inc\\spi_common_8xx.h"



 
#line 90 ".\\lpc_chip_82x\\inc\\spi_common_8xx.h"



 








 
#line 113 ".\\lpc_chip_82x\\inc\\spi_common_8xx.h"



 
#line 124 ".\\lpc_chip_82x\\inc\\spi_common_8xx.h"



 
#line 135 ".\\lpc_chip_82x\\inc\\spi_common_8xx.h"



 
#line 145 ".\\lpc_chip_82x\\inc\\spi_common_8xx.h"



 
#line 160 ".\\lpc_chip_82x\\inc\\spi_common_8xx.h"



 




 
#line 179 ".\\lpc_chip_82x\\inc\\spi_common_8xx.h"



 




 
#line 195 ".\\lpc_chip_82x\\inc\\spi_common_8xx.h"

 
#line 205 ".\\lpc_chip_82x\\inc\\spi_common_8xx.h"








 
static __inline void Chip_SPI_SetCFGRegBits(LPC_SPI_T *pSPI, uint32_t bits)
{
	 
	pSPI->CFG = bits | (pSPI->CFG & (0x1BD));
}








 
static __inline void Chip_SPI_ClearCFGRegBits(LPC_SPI_T *pSPI, uint32_t bits)
{
	 
	pSPI->CFG = ~bits & (pSPI->CFG & (0x1BD));
}





 
static __inline void Chip_SPI_Init(LPC_SPI_T *pSPI)
{
	 
	if (pSPI == ((LPC_SPI_T *) (0x4005C000UL))) {
		Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SPI1);
		Chip_SYSCTL_PeriphReset(RESET_SPI1);
	} else {
		Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SPI0);
		Chip_SYSCTL_PeriphReset(RESET_SPI0);
	}
}





 
static __inline void Chip_SPI_Disable(LPC_SPI_T *pSPI)
{
	Chip_SPI_ClearCFGRegBits(pSPI, (1 << 0));
}






 
static __inline void Chip_SPI_DeInit(LPC_SPI_T *pSPI)
{
	Chip_SPI_Disable(pSPI);
	if (pSPI == ((LPC_SPI_T *) (0x4005C000UL))) {
		Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SPI1);
	} else {
		Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SPI0);
	}
}





 
static __inline void Chip_SPI_Enable(LPC_SPI_T *pSPI)
{
	Chip_SPI_SetCFGRegBits(pSPI, (1 << 0));
}







 
static __inline void Chip_SPI_EnableMasterMode(LPC_SPI_T *pSPI)
{
	Chip_SPI_SetCFGRegBits(pSPI, (1 << 2));

	 
	pSPI->TXCTRL = (0xF << 16);
}






 
static __inline void Chip_SPI_EnableSlaveMode(LPC_SPI_T *pSPI)
{
	Chip_SPI_ClearCFGRegBits(pSPI, (1 << 2));
}





 
static __inline void Chip_SPI_EnableLSBFirst(LPC_SPI_T *pSPI)
{
	Chip_SPI_SetCFGRegBits(pSPI, (1 << 3));
}





 
static __inline void Chip_SPI_EnableMSBFirst(LPC_SPI_T *pSPI)
{
	Chip_SPI_ClearCFGRegBits(pSPI, (1 << 3));
}







 
static __inline void Chip_SPI_SetSPIMode(LPC_SPI_T *pSPI, uint32_t mode)
{
	Chip_SPI_ClearCFGRegBits(pSPI, ((1 << 5) | (1 << 4)));
	Chip_SPI_SetCFGRegBits(pSPI, (uint32_t) mode);
}







 
static __inline void Chip_SPI_SetCSPolHigh(LPC_SPI_T *pSPI, uint8_t csNum)
{
	Chip_SPI_SetCFGRegBits(pSPI, (1 << ((csNum) + 8)));
}







 
static __inline void Chip_SPI_SetCSPolLow(LPC_SPI_T *pSPI, uint8_t csNum)
{
	Chip_SPI_ClearCFGRegBits(pSPI, (1 << ((csNum) + 8)));
}









 
static __inline void Chip_SPI_ConfigureSPI(LPC_SPI_T *pSPI, uint32_t config)
{
	Chip_SPI_ClearCFGRegBits(pSPI, (1 << 2) | (1 << 3) |
			(1 << 4) | (1 << 5));
	Chip_SPI_SetCFGRegBits(pSPI, config);

	 
	pSPI->TXCTRL = (0xF << 16);
}







 
static __inline uint32_t Chip_SPI_GetStatus(LPC_SPI_T *pSPI)
{
	return pSPI->STAT & ~(~0x1ff);
}








 
static __inline void Chip_SPI_ClearStatus(LPC_SPI_T *pSPI, uint32_t Flag)
{
	pSPI->STAT = Flag;
}






 
static __inline void Chip_SPI_EnableInts(LPC_SPI_T *pSPI, uint32_t Flag)
{
	pSPI->INTENSET = Flag;
}






 
static __inline void Chip_SPI_DisableInts(LPC_SPI_T *pSPI, uint32_t Flag)
{
	pSPI->INTENCLR = Flag;
}







 
static __inline uint32_t Chip_SPI_GetEnabledInts(LPC_SPI_T *pSPI)
{
	return pSPI->INTENSET & ~(~0x3f);
}







 
static __inline uint32_t Chip_SPI_GetPendingInts(LPC_SPI_T *pSPI)
{
	return pSPI->INTSTAT & ~(~0x3f);
}





 
static __inline void Chip_SPI_FlushFifos(LPC_SPI_T *pSPI)
{
	Chip_SPI_Disable(pSPI);
	Chip_SPI_Enable(pSPI);
}





 
static __inline uint32_t Chip_SPI_ReadRawRXFifo(LPC_SPI_T *pSPI)
{
	return pSPI->RXDAT & ~((7<<17)|(0x7ffu<<21));
}









 
static __inline uint32_t Chip_SPI_ReadRXData(LPC_SPI_T *pSPI)
{
	return pSPI->RXDAT & 0xFFFF;
}






 
static __inline void Chip_SPI_WriteTXData(LPC_SPI_T *pSPI, uint16_t data)
{
	pSPI->TXDAT = (uint32_t) data;
}








 
static __inline void Chip_SPI_SetTXCTRLRegBits(LPC_SPI_T *pSPI, uint32_t bits)
{
	pSPI->TXCTRL = bits | (pSPI->TXCTRL & (0xF710000));
}








 
static __inline void Chip_SPI_ClearTXCTRLRegBits(LPC_SPI_T *pSPI, uint32_t bits)
{
	pSPI->TXCTRL = ~bits & (pSPI->TXCTRL & (0xF710000));
}








 
static __inline void Chip_SPI_SetTXCtl(LPC_SPI_T *pSPI, uint32_t ctrlBits)
{
	Chip_SPI_SetTXCTRLRegBits(pSPI, ctrlBits);
}








 
static __inline void Chip_SPI_ClearTXCtl(LPC_SPI_T *pSPI, uint32_t ctrlBits)
{
	Chip_SPI_ClearTXCTRLRegBits(pSPI, ctrlBits);
}






 
static __inline void Chip_SPI_SetXferSize(LPC_SPI_T *pSPI, uint32_t ctrlBits)
{
	Chip_SPI_ClearTXCTRLRegBits(pSPI, (0xF << 24));
	Chip_SPI_SetTXCTRLRegBits(pSPI, ((((ctrlBits) - 1) & 0x0F) << 24));
}



 





#line 36 ".\\lpc_chip_82x\\inc\\spi_8xx.h"








 

 
typedef enum {
	SPI_MODE_MASTER = (1 << 2),		 
	SPI_MODE_SLAVE = (0 << 0),			 
} SPI_MODE_T;

 
typedef enum IP_SPI_DATA_ORDER {
	SPI_DATA_MSB_FIRST = (0 << 3),			 
	SPI_DATA_LSB_FIRST = (1 << 3),			 
} SPI_DATA_ORDER_T;

 
typedef enum IP_SPI_SSEL_POL {
	SPI_SSEL_ACTIVE_LO = (0 << 8),			 
	SPI_SSEL_ACTIVE_HI = (1 << 8),			 
} SPI_SSEL_POL_T;



 
typedef struct {
	SPI_MODE_T             Mode;			 
	uint32_t               ClockMode;		 
	SPI_DATA_ORDER_T       DataOrder;		 
	SPI_SSEL_POL_T         SSELPol;		 
	uint16_t                ClkDiv;			 
} SPI_CONFIG_T;



 
typedef struct {
	uint8_t     PreDelay;				 
	uint8_t     PostDelay;				 
	uint8_t     FrameDelay;				 
	uint8_t     TransferDelay;			 
} SPI_DELAY_CONFIG_T;



 
typedef struct {
	uint16_t  *pTx;	 
	uint32_t  TxCnt; 
	uint16_t  *pRx;	 
	uint32_t  RxCnt; 
	uint32_t  Length;	 
	uint16_t  DataSize;	 
} SPI_DATA_SETUP_T;






 
uint32_t Chip_SPI_CalClkRateDivider(LPC_SPI_T *pSPI, uint32_t bitRate);







 
void Chip_SPI_DelayConfig(LPC_SPI_T *pSPI, SPI_DELAY_CONFIG_T *pConfig);







 
void Chip_SPI_Int_Cmd(LPC_SPI_T *pSPI, uint32_t IntMask, FunctionalState NewState);





 







 
static __inline void Chip_SPI_EnableLoopBack(LPC_SPI_T *pSPI)
{
	pSPI->CFG = (1 << 7) | (pSPI->CFG & ~((1<<1)|(1<<6)|0xfffffe00));
}







 
static __inline void Chip_SPI_DisableLoopBack(LPC_SPI_T *pSPI)
{
	pSPI->CFG &= (~(1 << 7)) & (0x1BD);
}








 
static __inline void Chip_SPI_SetControlInfo(LPC_SPI_T *pSPI, uint8_t Flen, uint32_t Flag)
{
	pSPI->TXCTRL = Flag | (((Flen - 1) & 0x0F) << 24);
}







 
static __inline void Chip_SPI_SendFirstFrame_RxIgnore(LPC_SPI_T *pSPI, uint16_t Data, uint8_t DataSize)
{
	pSPI->TXDATCTL = (0 << 16) | (1 << 21) | (1 << 22) | (((DataSize - 1) & 0x0F) << 24) | ((Data) & 0xFFFF);

}







 
static __inline void Chip_SPI_SendFirstFrame(LPC_SPI_T *pSPI, uint16_t Data, uint8_t DataSize)
{
	pSPI->TXDATCTL = (0 << 16) | (1 << 21) | (((DataSize - 1) & 0x0F) << 24) | ((Data) & 0xFFFF);

}






 
static __inline void Chip_SPI_SendMidFrame(LPC_SPI_T *pSPI, uint16_t Data)
{
	pSPI->TXDAT = ((Data) & 0xFFFF);
}







 
static __inline void Chip_SPI_SendLastFrame_RxIgnore(LPC_SPI_T *pSPI, uint16_t Data, uint8_t DataSize)
{
	pSPI->TXDATCTL = (0 << 16) | (1 << 21) | (1 << 20) | (1 << 22) |
					 (((DataSize - 1) & 0x0F) << 24) | ((Data) & 0xFFFF);
}







 
static __inline void Chip_SPI_SendLastFrame(LPC_SPI_T *pSPI, uint16_t Data, uint8_t DataSize)
{
	pSPI->TXDATCTL = (0 << 16) | (1 << 21) | (1 << 20) |
					 (((DataSize - 1) & 0x0F) << 24) | ((Data) & 0xFFFF);
}





 
static __inline uint16_t Chip_SPI_ReceiveFrame(LPC_SPI_T *pSPI)
{
	return ((pSPI->RXDAT) & 0xFFFF);
}







 
Status Chip_SPI_Int_RWFrames(LPC_SPI_T *pSPI, SPI_DATA_SETUP_T *xf_setup);











 
uint32_t Chip_SPI_RWFrames_Blocking(LPC_SPI_T *pSPI, SPI_DATA_SETUP_T *pXfSetup);










 
uint32_t Chip_SPI_WriteFrames_Blocking(LPC_SPI_T *pSPI, SPI_DATA_SETUP_T *pXfSetup);










 
uint32_t Chip_SPI_ReadFrames_Blocking(LPC_SPI_T *pSPI, SPI_DATA_SETUP_T *pXfSetup);



 





#line 193 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\i2cm_8xx.h"





























 




#line 1 ".\\lpc_chip_82x\\inc\\i2c_common_8xx.h"





























 











 



 
typedef struct {					 
	volatile uint32_t CFG;				 
	volatile uint32_t STAT;				 
	volatile uint32_t INTENSET;			 
	volatile  uint32_t INTENCLR;			 
	volatile uint32_t TIMEOUT;			 
	volatile uint32_t CLKDIV;			 
	volatile const  uint32_t INTSTAT;			 
	volatile const  uint32_t RESERVED0;        
	volatile uint32_t MSTCTL;			 
	volatile uint32_t MSTTIME;			 
	volatile uint32_t MSTDAT;			 
	volatile const  uint32_t RESERVED1[5];     
	volatile uint32_t SLVCTL;			 
	volatile uint32_t SLVDAT;			 
	volatile uint32_t SLVADR[4];		 
	volatile uint32_t SLVQUAL0;			 
	volatile const  uint32_t RESERVED2[9];     
	volatile const  uint32_t MONRXDAT;			 
} LPC_I2C_T;

 
#line 83 ".\\lpc_chip_82x\\inc\\i2c_common_8xx.h"



 
#line 93 ".\\lpc_chip_82x\\inc\\i2c_common_8xx.h"



 
#line 113 ".\\lpc_chip_82x\\inc\\i2c_common_8xx.h"













 
#line 138 ".\\lpc_chip_82x\\inc\\i2c_common_8xx.h"



 
#line 153 ".\\lpc_chip_82x\\inc\\i2c_common_8xx.h"



 




 
#line 173 ".\\lpc_chip_82x\\inc\\i2c_common_8xx.h"



 







 





 




 






 




 






 





 












 
void Chip_I2C_Init(LPC_I2C_T *pI2C);








 
void Chip_I2C_DeInit(LPC_I2C_T *pI2C);









 
static __inline void Chip_I2C_SetClockDiv(LPC_I2C_T *pI2C, uint32_t clkdiv)
{
	if ((clkdiv >= 1) && (clkdiv <= 65536)) {
		pI2C->CLKDIV = clkdiv - 1;
	}
	else {
		pI2C->CLKDIV = 0;
	}
}







 
static __inline uint32_t Chip_I2C_GetClockDiv(LPC_I2C_T *pI2C)
{
	return (pI2C->CLKDIV & 0xFFFF) + 1;
}






 
static __inline void Chip_I2C_EnableInt(LPC_I2C_T *pI2C, uint32_t intEn)
{
	pI2C->INTENSET = intEn;
}






 
static __inline void Chip_I2C_DisableInt(LPC_I2C_T *pI2C, uint32_t intClr)
{
	pI2C->INTENCLR = intClr;
}








 
static __inline void Chip_I2C_ClearInt(LPC_I2C_T *pI2C, uint32_t intClr)
{
	Chip_I2C_DisableInt(pI2C, intClr);
}





 
static __inline uint32_t Chip_I2C_GetPendingInt(LPC_I2C_T *pI2C)
{
	return pI2C->INTSTAT & ~((7<<1)|(1<<5)|(1<<7)|(3<<9)|(7<<12)|(1<<18)|(0xf<<20)|(0x3fu<<26));
}



 





#line 36 ".\\lpc_chip_82x\\inc\\i2cm_8xx.h"






















 



 

#line 71 ".\\lpc_chip_82x\\inc\\i2cm_8xx.h"



 



 
typedef struct {
	const uint8_t *txBuff;	 
	uint8_t *rxBuff;				 
	uint16_t txSz;					
 
	uint16_t rxSz;					
 
	uint16_t status;				 
	uint8_t slaveAddr;			 
} I2CM_XFER_T;












 
void Chip_I2CM_SetBusSpeed(LPC_I2C_T *pI2C, uint32_t busSpeed);






 
static __inline void Chip_I2CM_Enable(LPC_I2C_T *pI2C)
{
	pI2C->CFG = (pI2C->CFG & ((uint32_t) 0x1F)) | (1 << 0);
}






 
static __inline void Chip_I2CM_Disable(LPC_I2C_T *pI2C)
{
	pI2C->CFG = (pI2C->CFG & ((uint32_t) 0x1F)) & ~(1 << 0);
}






 
static __inline uint32_t Chip_I2CM_GetStatus(LPC_I2C_T *pI2C)
{
	return pI2C->STAT & ~((1<<5)|(1<<7)|(0xf<<20)|(0x3fu<<26));
}







 
static __inline void Chip_I2CM_ClearStatus(LPC_I2C_T *pI2C, uint32_t clrStatus)
{
	 
	pI2C->STAT = clrStatus & ((1 << 4) | (1 << 6));
}






 
static __inline _Bool Chip_I2CM_IsMasterPending(LPC_I2C_T *pI2C)
{
	return (pI2C->STAT & (1 << 0)) != 0;
}







 
static __inline uint32_t Chip_I2CM_GetMasterState(LPC_I2C_T *pI2C)
{
	return (pI2C->STAT & (0x7 << 1)) >> 1;
}








 
static __inline void Chip_I2CM_SendStart(LPC_I2C_T *pI2C)
{
	pI2C->MSTCTL = (1 << 1);
}








 
static __inline void Chip_I2CM_SendStop(LPC_I2C_T *pI2C)
{
	pI2C->MSTCTL = (1 << 2);
}








 
static __inline void Chip_I2CM_MasterContinue(LPC_I2C_T *pI2C)
{
	pI2C->MSTCTL = (1 << 0);
}









 
static __inline void Chip_I2CM_WriteByte(LPC_I2C_T *pI2C, uint8_t data)
{
	pI2C->MSTDAT = (uint32_t) data;
}







 
static __inline uint8_t Chip_I2CM_ReadByte(LPC_I2C_T *pI2C)
{
	return (uint8_t) (pI2C->MSTDAT & ((uint32_t) 0x00FF << 0));
}












 
uint32_t Chip_I2CM_XferHandler(LPC_I2C_T *pI2C, I2CM_XFER_T *xfer);






































 
void Chip_I2CM_Xfer(LPC_I2C_T *pI2C, I2CM_XFER_T *xfer);








 
uint32_t Chip_I2CM_XferBlocking(LPC_I2C_T *pI2C, I2CM_XFER_T *xfer);



 





#line 194 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\i2cs_8xx.h"





























 




#line 36 ".\\lpc_chip_82x\\inc\\i2cs_8xx.h"









 





 
typedef void (*I2CSlaveXferStart)(uint8_t addr);





 
typedef uint8_t (*I2CSlaveXferSend)(uint8_t *data);





 
typedef uint8_t (*I2CSlaveXferRecv)(uint8_t data);




 
typedef void (*I2CSlaveXferDone)(void);






















 
typedef struct {
	I2CSlaveXferStart slaveStart;	 
	I2CSlaveXferSend slaveSend;		 
	I2CSlaveXferRecv slaveRecv;		 
	I2CSlaveXferDone slaveDone;		 
} I2CS_XFER_T;






 
static __inline void Chip_I2CS_Enable(LPC_I2C_T *pI2C)
{
	pI2C->CFG = (pI2C->CFG & ((uint32_t) 0x1F)) | (1 << 1);
}





 
static __inline void Chip_I2CS_Disable(LPC_I2C_T *pI2C)
{
	pI2C->CFG = (pI2C->CFG & ((uint32_t) 0x1F)) & ~(1 << 1);
}






 
static __inline uint32_t Chip_I2CS_GetStatus(LPC_I2C_T *pI2C)
{
	return pI2C->STAT & ~((1<<5)|(1<<7)|(0xf<<20)|(0x3fu<<26));
}







 
static __inline void Chip_I2CS_ClearStatus(LPC_I2C_T *pI2C, uint32_t clrStatus)
{
	pI2C->STAT = clrStatus & (1 << 15);
}






 
static __inline _Bool Chip_I2CS_IsSlavePending(LPC_I2C_T *pI2C)
{
	return (pI2C->STAT & (1 << 8)) != 0;
}






 
static __inline _Bool Chip_I2CS_IsSlaveSelected(LPC_I2C_T *pI2C)
{
	return (pI2C->STAT & (1 << 14)) != 0;
}






 
static __inline _Bool Chip_I2CS_IsSlaveDeSelected(LPC_I2C_T *pI2C)
{
	return (pI2C->STAT & (1 << 15)) != 0;
}







 
static __inline uint32_t Chip_I2CS_GetSlaveState(LPC_I2C_T *pI2C)
{
	return (pI2C->STAT & (0x3 << 9)) >> 9;
}





 
static __inline uint32_t Chip_I2CS_GetSlaveMatchIndex(LPC_I2C_T *pI2C)
{
	return (pI2C->STAT & (0x3 << 12)) >> 12;
}








 
static __inline void Chip_I2CS_SlaveContinue(LPC_I2C_T *pI2C)
{
	pI2C->SLVCTL = (1 << 0);
}






 
static __inline void Chip_I2CS_SlaveNACK(LPC_I2C_T *pI2C)
{
	pI2C->SLVCTL = (1 << 1);
}









 
static __inline void Chip_I2CS_WriteByte(LPC_I2C_T *pI2C, uint8_t data)
{
	pI2C->SLVDAT = (uint32_t) data;
}







 
static __inline uint8_t Chip_I2CS_ReadByte(LPC_I2C_T *pI2C)
{
	return (uint8_t) (pI2C->SLVDAT & ((uint32_t) 0x00FF << 0));
}









 
static __inline void Chip_I2CS_SetSlaveAddr(LPC_I2C_T *pI2C, uint8_t slvNum, uint8_t slvAddr)
{
	pI2C->SLVADR[slvNum] = (uint32_t) (slvAddr << 1);
}






 
static __inline uint8_t Chip_I2CS_GetSlaveAddr(LPC_I2C_T *pI2C, uint8_t slvNum)
{
	return (pI2C->SLVADR[slvNum] >> 1) & 0x7F;
}






 
static __inline void Chip_I2CS_EnableSlaveAddr(LPC_I2C_T *pI2C, uint8_t slvNum)
{
	pI2C->SLVADR[slvNum] = (pI2C->SLVADR[slvNum] & ((uint32_t) 0x00FF)) & ~(1 << 0);
}






 
static __inline void Chip_I2CS_DisableSlaveAddr(LPC_I2C_T *pI2C, uint8_t slvNum)
{
	pI2C->SLVADR[slvNum] = (pI2C->SLVADR[slvNum] & ((uint32_t) 0x00FF)) | (1 << 0);
}








 
static __inline void Chip_I2CS_SetSlaveQual0(LPC_I2C_T *pI2C, _Bool extend, uint8_t slvNum)
{
	slvNum = slvNum << 1;
	if (extend) {
		slvNum |= (1 << 0);
	}

	pI2C->SLVQUAL0 = slvNum;
}










 
uint32_t Chip_I2CS_XferHandler(LPC_I2C_T *pI2C, const I2CS_XFER_T *xfers);



 





#line 195 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\spim_8xx.h"





























 




#line 36 ".\\lpc_chip_82x\\inc\\spim_8xx.h"








 





 
uint32_t Chip_SPIM_GetClockRate(LPC_SPI_T *pSPI);








 
uint32_t Chip_SPIM_SetClockRate(LPC_SPI_T *pSPI, uint32_t rate);



 
typedef struct {
	uint8_t PreDelay;					 
	uint8_t PostDelay;					 
	uint8_t FrameDelay;					 
	uint8_t TransferDelay;				 
} SPIM_DELAY_CONFIG_T;






 
void Chip_SPIM_DelayConfig(LPC_SPI_T *pSPI, SPIM_DELAY_CONFIG_T *pConfig);









 
static __inline void Chip_SPIM_ForceEndOfTransfer(LPC_SPI_T *pSPI)
{
	pSPI->STAT = (1 << 7);
}






 
void Chip_SPIM_AssertSSEL(LPC_SPI_T *pSPI, uint8_t sselNum);






 
void Chip_SPIM_DeAssertSSEL(LPC_SPI_T *pSPI, uint8_t sselNum);







 
static __inline void Chip_SPIM_EnableLoopBack(LPC_SPI_T *pSPI)
{
	Chip_SPI_SetCFGRegBits(pSPI, (1 << 7));
}





 
static __inline void Chip_SPIM_DisableLoopBack(LPC_SPI_T *pSPI)
{
	Chip_SPI_ClearCFGRegBits(pSPI, (1 << 7));
}

struct SPIM_XFER;




 
typedef void (*SPIMasterXferCSAssert)(struct SPIM_XFER *pMasterXfer);




 
typedef void (*SPIMasterXferSend)(struct SPIM_XFER *pMasterXfer);




 
typedef void (*SPIMasterXferRecv)(struct SPIM_XFER *pMasterXfer);




 
typedef void (*SPIMMasterXferCSDeAssert)(struct SPIM_XFER *pMasterXfer);




 
typedef void (*SPIMMasterXferDone)(struct SPIM_XFER *pMasterXfer);

 
typedef struct {
	SPIMasterXferCSAssert   masterXferCSAssert;		 
	SPIMasterXferSend       masterXferSend;			 
	SPIMasterXferRecv       masterXferRecv;			 
	SPIMMasterXferCSDeAssert mMasterXferCSDeAssert;	 
	SPIMMasterXferDone      mMasterXferDone;		 
} SPIM_CALLBACKS_T;

 
typedef struct SPIM_XFER {
	const SPIM_CALLBACKS_T *pCB;	 
	union {							 
		uint8_t *pRXData8;			 
		uint16_t *pRXData16;		 
	};

	union {							 
		uint8_t *pTXData8;			 
		uint16_t *pTXData16;		 
	};

	uint32_t options;				 
	uint16_t rxCount;				 
	uint16_t txCount;				 
	uint16_t dataRXferred;			 
	uint16_t dataTXferred;			 
	uint8_t sselNum;				 
	_Bool    terminate;				 
} SPIM_XFER_T;










 
void Chip_SPIM_XferHandler(LPC_SPI_T *pSPI, SPIM_XFER_T *xfer);
































 
void Chip_SPIM_Xfer(LPC_SPI_T *pSPI, SPIM_XFER_T *xfer);











 
void Chip_SPIM_XferBlocking(LPC_SPI_T *pSPI, SPIM_XFER_T *xfer);



 





#line 196 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\spis_8xx.h"





























 




#line 36 ".\\lpc_chip_82x\\inc\\spis_8xx.h"








 



 

 

 

 

 


struct SPIS_XFER;





 
typedef void (*SPISlaveXferCSAssert)(struct SPIS_XFER *pSlaveXfer);




 
typedef void (*SPISlaveXferSend)(struct SPIS_XFER *pSlaveXfer);




 
typedef void (*SPISlaveXferRecv)(struct SPIS_XFER *pSlaveXfer);




 
typedef void (*SPISlaveXferCSDeAssert)(struct SPIS_XFER *pSlaveXfer);

 
typedef struct {
	SPISlaveXferCSAssert    slaveXferCSAssert;		 
	SPISlaveXferSend        slaveXferSend;			 
	SPISlaveXferRecv        slaveXferRecv;			 
	SPISlaveXferCSDeAssert  slaveXferCSDeAssert;	 
} SPIS_CALLBACKS_T;

 
typedef struct SPIS_XFER {
	const SPIS_CALLBACKS_T *pCB;	 
	union {							 
		uint8_t *pRXData8;			 
		uint16_t *pRXData16;		 
	};

	union {							 
		uint8_t *pTXData8;			 
		uint16_t *pTXData16;		 
	};

	uint16_t rxCount;				 
	uint16_t txCount;				 
	uint16_t dataRXferred;			 
	uint16_t dataTXferred;			 
	uint8_t sselNum;				 
} SPIS_XFER_T;















 
uint32_t Chip_SPIS_XferHandler(LPC_SPI_T *pSPI, SPIS_XFER_T *xfer);











 
static __inline void Chip_SPIS_PreBuffSlave(LPC_SPI_T *pSPI, SPIS_XFER_T *xfer)
{
	Chip_SPIS_XferHandler(pSPI, xfer);
}











 
uint32_t Chip_SPIS_XferBlocking(LPC_SPI_T *pSPI, SPIS_XFER_T *xfer);



 





#line 197 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\irc_8xx.h"





























 











 

 








 
_Bool Chip_IRC_SetFreq(uint32_t main, uint32_t sys);

 




 
void Chip_IRC_SetFreq_ROM(uint32_t sys);



 





#line 198 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\adc_8xx.h"





























 











 


 
typedef enum {
	ADC_SEQA_IDX,
	ADC_SEQB_IDX
} ADC_SEQ_IDX_T;



 
typedef struct {								 
	volatile uint32_t CTRL;							 
	volatile const  uint32_t RESERVED0;
	volatile uint32_t SEQ_CTRL[ADC_SEQB_IDX + 1];	 
	volatile uint32_t SEQ_GDAT[ADC_SEQB_IDX + 1];	 
	volatile const  uint32_t RESERVED1[2];
	volatile const  uint32_t DR[12];						 
	volatile uint32_t THR_LOW[2];					 
	volatile uint32_t THR_HIGH[2];					 
	volatile uint32_t CHAN_THRSEL;					 
	volatile uint32_t INTEN;						 
	volatile uint32_t FLAGS;						 
	volatile uint32_t TRM;							 
} LPC_ADC_T;

 




 
 
#line 85 ".\\lpc_chip_82x\\inc\\adc_8xx.h"

 



 
#line 102 ".\\lpc_chip_82x\\inc\\adc_8xx.h"

 
#line 112 ".\\lpc_chip_82x\\inc\\adc_8xx.h"

 
#line 124 ".\\lpc_chip_82x\\inc\\adc_8xx.h"

 
#line 139 ".\\lpc_chip_82x\\inc\\adc_8xx.h"

 



 


 
#line 157 ".\\lpc_chip_82x\\inc\\adc_8xx.h"

 
#line 169 ".\\lpc_chip_82x\\inc\\adc_8xx.h"

 



 












 
void Chip_ADC_Init(LPC_ADC_T *pADC, uint32_t flags);






 
void Chip_ADC_DeInit(LPC_ADC_T *pADC);













 
static __inline void Chip_ADC_SetDivider(LPC_ADC_T *pADC, uint8_t div)
{
	uint32_t temp;

	temp = pADC->CTRL & ~((0xFF << 0));
	pADC->CTRL = temp | (uint32_t) div;
}










 
static __inline void Chip_ADC_SetClockRate(LPC_ADC_T *pADC, uint32_t rate)
{
	Chip_ADC_SetDivider(pADC, (uint8_t) (Chip_Clock_GetSystemClockRate() / rate) - 1);
}









 
static __inline uint8_t Chip_ADC_GetDivider(LPC_ADC_T *pADC)
{
	return pADC->CTRL & (0xFF << 0);
}









 
void Chip_ADC_StartCalibration(LPC_ADC_T *pADC);





 
static __inline _Bool Chip_ADC_IsCalibrationDone(LPC_ADC_T *pADC)
{
	return (_Bool) ((pADC->CTRL & (1 << 30)) == 0);
}









 
static __inline void Chip_ADC_SetSequencerBits(LPC_ADC_T *pADC, ADC_SEQ_IDX_T seqIndex, uint32_t bits)
{
	pADC->SEQ_CTRL[seqIndex] = (pADC->SEQ_CTRL[seqIndex] & ~((7 << 15) | (0x3F << 20))) | bits;
}









 
static __inline void Chip_ADC_ClearSequencerBits(LPC_ADC_T *pADC, ADC_SEQ_IDX_T seqIndex, uint32_t bits)
{
	pADC->SEQ_CTRL[seqIndex] = pADC->SEQ_CTRL[seqIndex] & ~(((7 << 15) | (0x3F << 20)) | bits);
}























 
static __inline void Chip_ADC_SetupSequencer(LPC_ADC_T *pADC, ADC_SEQ_IDX_T seqIndex, uint32_t options)
{
	pADC->SEQ_CTRL[seqIndex] = options;
}






 
static __inline void Chip_ADC_EnableSequencer(LPC_ADC_T *pADC, ADC_SEQ_IDX_T seqIndex)
{
	Chip_ADC_SetSequencerBits(pADC, seqIndex, (1UL << 31));
}






 
static __inline void Chip_ADC_DisableSequencer(LPC_ADC_T *pADC, ADC_SEQ_IDX_T seqIndex)
{
	Chip_ADC_ClearSequencerBits(pADC, seqIndex, (1UL << 31));
}








 
static __inline void Chip_ADC_StartSequencer(LPC_ADC_T *pADC, ADC_SEQ_IDX_T seqIndex)
{
	Chip_ADC_SetSequencerBits(pADC, seqIndex, (1 << 26));
}









 
static __inline void Chip_ADC_StartBurstSequencer(LPC_ADC_T *pADC, ADC_SEQ_IDX_T seqIndex)
{
	Chip_ADC_SetSequencerBits(pADC, seqIndex, (1 << 27));
}






 
static __inline void Chip_ADC_StopBurstSequencer(LPC_ADC_T *pADC, ADC_SEQ_IDX_T seqIndex)
{
	Chip_ADC_ClearSequencerBits(pADC, seqIndex, (1 << 27));
}

 
typedef enum {
	ADC_DR_THCMPRANGE_INRANGE,
	ADC_DR_THCMPRANGE_RESERVED,
	ADC_DR_THCMPRANGE_BELOW,
	ADC_DR_THCMPRANGE_ABOVE
} ADC_DR_THCMPRANGE_T;

 
typedef enum {
	ADC_DR_THCMPCROSS_NOCROSS,
	ADC_DR_THCMPCROSS_RESERVED,
	ADC_DR_THCMPCROSS_DOWNWARD,
	ADC_DR_THCMPCROSS_UPWARD
} ADC_DR_THCMPCROSS_T;
















 
static __inline uint32_t Chip_ADC_GetSequencerDataReg(LPC_ADC_T *pADC, ADC_SEQ_IDX_T seqIndex)
{
	return pADC->SEQ_GDAT[seqIndex];
}
















 
static __inline uint32_t Chip_ADC_GetDataReg(LPC_ADC_T *pADC, uint8_t index)
{
	return pADC->DR[index];
}







 
static __inline void Chip_ADC_SetThrLowValue(LPC_ADC_T *pADC, uint8_t thrnum, uint16_t value)
{
	pADC->THR_LOW[thrnum] = (((uint32_t) value) << (4));
}







 
static __inline void Chip_ADC_SetThrHighValue(LPC_ADC_T *pADC, uint8_t thrnum, uint16_t value)
{
	pADC->THR_HIGH[thrnum] = (((uint32_t) value) << (4));
}









 
static __inline void Chip_ADC_SelectTH0Channels(LPC_ADC_T *pADC, uint32_t channels)
{
	pADC->CHAN_THRSEL = pADC->CHAN_THRSEL & ~(0xFFFFF000 | channels);
}









 
static __inline void Chip_ADC_SelectTH1Channels(LPC_ADC_T *pADC, uint32_t channels)
{
	pADC->CHAN_THRSEL = (pADC->CHAN_THRSEL & ~0xFFFFF000) | channels;
}









 
static __inline void Chip_ADC_EnableInt(LPC_ADC_T *pADC, uint32_t intMask)
{
	pADC->INTEN = (pADC->INTEN & ~0xF8000000) | intMask;
}









 
static __inline void Chip_ADC_DisableInt(LPC_ADC_T *pADC, uint32_t intMask)
{
	pADC->INTEN = pADC->INTEN & ~(0xF8000000 | intMask);
}

 
typedef enum {
	ADC_INTEN_THCMP_DISABLE,
	ADC_INTEN_THCMP_OUTSIDE,
	ADC_INTEN_THCMP_CROSSING,
} ADC_INTEN_THCMP_T;







 
static __inline void Chip_ADC_SetThresholdInt(LPC_ADC_T *pADC, uint8_t ch, ADC_INTEN_THCMP_T thInt)
{
	pADC->INTEN = (pADC->INTEN & ~(0xF8000000 | (3 << (3 + (ch * 2))))) | (thInt << (3 + (ch * 2)));
}









 
static __inline uint32_t Chip_ADC_GetFlags(LPC_ADC_T *pADC)
{
	return pADC->FLAGS;
}






 
static __inline void Chip_ADC_ClearFlags(LPC_ADC_T *pADC, uint32_t flags)
{
	pADC->FLAGS = flags;
}






 
static __inline void Chip_ADC_SetTrim(LPC_ADC_T *pADC, uint32_t trim)
{
	pADC->TRM = trim;
}



 





#line 200 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\dma_8xx.h"





























 











 



 
typedef struct {					 
	volatile uint32_t  ENABLESET;		 
	volatile const  uint32_t  RESERVED0;
	volatile  uint32_t  ENABLECLR;		 
	volatile const  uint32_t  RESERVED1;
	volatile const  uint32_t  ACTIVE;			 
	volatile const  uint32_t  RESERVED2;
	volatile const  uint32_t  BUSY;			 
	volatile const  uint32_t  RESERVED3;
	volatile uint32_t  ERRINT;			 
	volatile const  uint32_t  RESERVED4;
	volatile uint32_t  INTENSET;		 
	volatile const  uint32_t  RESERVED5;
	volatile  uint32_t  INTENCLR;		 
	volatile const  uint32_t  RESERVED6;
	volatile uint32_t  INTA;			 
	volatile const  uint32_t  RESERVED7;
	volatile uint32_t  INTB;			 
	volatile const  uint32_t  RESERVED8;
	volatile  uint32_t  SETVALID;		 
	volatile const  uint32_t  RESERVED9;
	volatile  uint32_t  SETTRIG;			 
	volatile const  uint32_t  RESERVED10;
	volatile  uint32_t  ABORT;			 
} LPC_DMA_COMMON_T;



 
typedef struct {					 
	volatile uint32_t  CFG;				 
	volatile const  uint32_t  CTLSTAT;			 
	volatile uint32_t  XFERCFG;			 
	volatile const  uint32_t  RESERVED;
} LPC_DMA_CHANNEL_T;

 





 
typedef enum {
	DMAREQ_USART0_RX,					 
	DMA_CH0 = DMAREQ_USART0_RX,
	DMAREQ_USART0_TX,					 
	DMA_CH1 = DMAREQ_USART0_TX,
	DMAREQ_USART1_RX,					 
	DMA_CH2 = DMAREQ_USART1_RX,
	DMAREQ_USART1_TX,					 
	DMA_CH3 = DMAREQ_USART1_TX,
	DMAREQ_USART2_RX,					 
	DMA_CH4 = DMAREQ_USART2_RX,
	DMAREQ_USART2_TX,					 
	DMA_CH5 = DMAREQ_USART2_TX,
	DMAREQ_SPI0_RX,
	DMA_CH6 = DMAREQ_SPI0_RX,            
	DMAREQ_SPI0_TX,
	DMA_CH7 = DMAREQ_SPI0_TX,            
	DMAREQ_SPI1_RX,
	DMA_CH8 = DMAREQ_SPI1_RX,            
	DMAREQ_SPI1_TX,
	DMA_CH9 = DMAREQ_SPI1_TX,            
	DMAREQ_I2C0_MST,
	DMA_CH10 = DMAREQ_I2C0_MST,          
	DMAREQ_I2C0_SLV,
	DMA_CH11 = DMAREQ_I2C0_SLV,          
	DMAREQ_I2C1_MST,
	DMA_CH12 = DMAREQ_I2C1_MST,          
	DMAREQ_I2C1_SLV,
	DMA_CH13 = DMAREQ_I2C1_SLV,          
	DMAREQ_I2C2_MST,
	DMA_CH14 = DMAREQ_I2C2_MST,          
	DMAREQ_I2C2_SLV,
	DMA_CH15 = DMAREQ_I2C2_SLV,          
	DMAREQ_I2C3_MST,
	DMA_CH16 = DMAREQ_I2C3_MST,          
	DMAREQ_I2C3_SLV,
	DMA_CH17 = DMAREQ_I2C3_SLV,          
} DMA_CHID_T;

 


 
#line 146 ".\\lpc_chip_82x\\inc\\dma_8xx.h"



 
typedef struct {					 
	volatile uint32_t  CTRL;			 
	volatile const  uint32_t  INTSTAT;			 
	volatile uint32_t  SRAMBASE;		 
	volatile const  uint32_t  RESERVED2[5];
	LPC_DMA_COMMON_T DMACOMMON[1];	 
	volatile const  uint32_t  RESERVED0[225];
	LPC_DMA_CHANNEL_T DMACH[(DMA_CH17 + 1)];	 
} LPC_DMA_T;

 






 





 
static __inline void Chip_DMA_Init(LPC_DMA_T *pDMA)
{
	(void) pDMA;
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_DMA);
}





 
static __inline void Chip_DMA_DeInit(LPC_DMA_T *pDMA)
{
	(void) pDMA;
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_DMA);
}





 
static __inline void Chip_DMA_Enable(LPC_DMA_T *pDMA)
{
	pDMA->CTRL = 1;
}





 
static __inline void Chip_DMA_Disable(LPC_DMA_T *pDMA)
{
	pDMA->CTRL = 0;
}

 










 
static __inline uint32_t Chip_DMA_GetIntStatus(LPC_DMA_T *pDMA)
{
	return (pDMA->INTSTAT & ~(~7));
}

 
typedef struct {
	uint32_t  xfercfg;		 
	uint32_t  source;		 
	uint32_t  dest;			 
	uint32_t  next;			 
} DMA_CHDESC_T;


 
extern DMA_CHDESC_T Chip_DMA_Table[(DMA_CH17 + 1)];












 
static __inline void Chip_DMA_SetSRAMBase(LPC_DMA_T *pDMA, uint32_t base)
{
	pDMA->SRAMBASE = base;
}





 
static __inline uint32_t Chip_DMA_GetSRAMBase(LPC_DMA_T *pDMA)
{
	return (pDMA->SRAMBASE & ~(0xFF));
}



 



 






 
static __inline void Chip_DMA_EnableChannel(LPC_DMA_T *pDMA, DMA_CHID_T ch)
{
	pDMA->DMACOMMON[0].ENABLESET = (1 << ch);
}






 
static __inline void Chip_DMA_DisableChannel(LPC_DMA_T *pDMA, DMA_CHID_T ch)
{
	pDMA->DMACOMMON[0].ENABLECLR = (1 << ch);
}








 
static __inline uint32_t Chip_DMA_GetEnabledChannels(LPC_DMA_T *pDMA)
{
	return (pDMA->DMACOMMON[0].ENABLESET & ~(~(0UL) << (DMA_CH17 + 1)));
}










 
static __inline uint32_t Chip_DMA_GetActiveChannels(LPC_DMA_T *pDMA)
{
	return (pDMA->DMACOMMON[0].ACTIVE & ~(~(0UL) << (DMA_CH17 + 1)));
}











 
static __inline uint32_t Chip_DMA_GetBusyChannels(LPC_DMA_T *pDMA)
{
	return (pDMA->DMACOMMON[0].BUSY & ~(~(0UL) << (DMA_CH17 + 1)));
}









 
static __inline uint32_t Chip_DMA_GetErrorIntChannels(LPC_DMA_T *pDMA)
{
	return (pDMA->DMACOMMON[0].ERRINT & ~(~(0UL) << (DMA_CH17 + 1)));
}






 
static __inline void Chip_DMA_ClearErrorIntChannel(LPC_DMA_T *pDMA, DMA_CHID_T ch)
{
	pDMA->DMACOMMON[0].ERRINT = (1 << ch);
}






 
static __inline void Chip_DMA_EnableIntChannel(LPC_DMA_T *pDMA, DMA_CHID_T ch)
{
	pDMA->DMACOMMON[0].INTENSET = (1 << ch);
}






 
static __inline void Chip_DMA_DisableIntChannel(LPC_DMA_T *pDMA, DMA_CHID_T ch)
{
	pDMA->DMACOMMON[0].INTENCLR = (1 << ch);
}










 
static __inline uint32_t Chip_DMA_GetEnableIntChannels(LPC_DMA_T *pDMA)
{
	return (pDMA->DMACOMMON[0].INTENSET & ~(~(0UL) << (DMA_CH17 + 1)));
}









 
static __inline uint32_t Chip_DMA_GetActiveIntAChannels(LPC_DMA_T *pDMA)
{
	return (pDMA->DMACOMMON[0].INTA & ~(~(0UL) << (DMA_CH17 + 1)));
}






 
static __inline void Chip_DMA_ClearActiveIntAChannel(LPC_DMA_T *pDMA, DMA_CHID_T ch)
{
	pDMA->DMACOMMON[0].INTA = (1 << ch);
}









 
static __inline uint32_t Chip_DMA_GetActiveIntBChannels(LPC_DMA_T *pDMA)
{
	return (pDMA->DMACOMMON[0].INTB & ~(~(0UL) << (DMA_CH17 + 1)));
}






 
static __inline void Chip_DMA_ClearActiveIntBChannel(LPC_DMA_T *pDMA, DMA_CHID_T ch)
{
	pDMA->DMACOMMON[0].INTB = (1 << ch);
}








 
static __inline void Chip_DMA_SetValidChannel(LPC_DMA_T *pDMA, DMA_CHID_T ch)
{
	pDMA->DMACOMMON[0].SETVALID = (1 << ch);
}







 
static __inline void Chip_DMA_SetTrigChannel(LPC_DMA_T *pDMA, DMA_CHID_T ch)
{
	pDMA->DMACOMMON[0].SETTRIG = (1 << ch);
}











 
static __inline void Chip_DMA_AbortChannel(LPC_DMA_T *pDMA, DMA_CHID_T ch)
{
	pDMA->DMACOMMON[0].ABORT = (1 << ch);
}



 




 

 



 
#line 531 ".\\lpc_chip_82x\\inc\\dma_8xx.h"





























 
static __inline void Chip_DMA_SetupChannelConfig(LPC_DMA_T *pDMA, DMA_CHID_T ch, uint32_t cfg)
{
	pDMA->DMACH[ch].CFG = cfg;
}

 








 
static __inline uint32_t Chip_DMA_GetChannelStatus(LPC_DMA_T *pDMA, DMA_CHID_T ch)
{
	return (pDMA->DMACH[ch].XFERCFG & ~((3<<6)|(3<<10)|(0x3fu<<26)));
}

 
#line 600 ".\\lpc_chip_82x\\inc\\dma_8xx.h"

















 
static __inline void Chip_DMA_SetupChannelTransfer(LPC_DMA_T *pDMA, DMA_CHID_T ch, uint32_t cfg)
{
	pDMA->DMACH[ch].XFERCFG = cfg;
}









 
static __inline void Chip_DMA_SetTranBits(LPC_DMA_T *pDMA, DMA_CHID_T ch, uint32_t mask)
{
	
 
	pDMA->DMACH[ch].XFERCFG = (pDMA->DMACH[ch].XFERCFG & ~((3<<6)|(3<<10)|(0x3fu<<26))) | mask;
}









 
static __inline void Chip_DMA_ClearTranBits(LPC_DMA_T *pDMA, DMA_CHID_T ch, uint32_t mask)
{
	
 
	pDMA->DMACH[ch].XFERCFG &= ~(((3<<6)|(3<<10)|(0x3fu<<26)) | mask);
}







 
static __inline void Chip_DMA_SetupChannelTransferSize(LPC_DMA_T *pDMA, DMA_CHID_T ch, uint32_t trans)
{
	pDMA->DMACH[ch].XFERCFG = (pDMA->DMACH[ch].XFERCFG & ~(((3<<6)|(3<<10)|(0x3fu<<26)) | (0x3FF << 16))) | ((trans - 1) << 16);
}






 
static __inline void Chip_DMA_SetChannelValid(LPC_DMA_T *pDMA, DMA_CHID_T ch)
{
	Chip_DMA_SetTranBits(pDMA, ch, (1 << 0));
}






 
static __inline void Chip_DMA_SetChannelInValid(LPC_DMA_T *pDMA, DMA_CHID_T ch)
{
	Chip_DMA_ClearTranBits(pDMA, ch, (1 << 0));
}






 
static __inline void Chip_DMA_SWTriggerChannel(LPC_DMA_T *pDMA, DMA_CHID_T ch)
{
	Chip_DMA_SetTranBits(pDMA, ch, (1 << 2));
}






 
static __inline _Bool Chip_DMA_IsChannelActive(LPC_DMA_T *pDMA, DMA_CHID_T ch)
{
	return (pDMA->DMACOMMON[0].ACTIVE & (1 << ch)) != 0;
}
















 
static __inline _Bool Chip_DMA_SetupTranChannel(LPC_DMA_T *pDMA, DMA_CHID_T ch, const DMA_CHDESC_T *desc)
{
	 
	if (Chip_DMA_IsChannelActive(pDMA, ch))
		return 0;

	 
	((DMA_CHDESC_T *) (pDMA->SRAMBASE & ~(0xFF)))[ch] = *desc;
	return 1;
}



 



 





#line 201 ".\\lpc_chip_82x\\inc\\chip.h"
#line 1 ".\\lpc_chip_82x\\inc\\inmux_8xx.h"





























 











 
typedef struct {
	volatile uint32_t  DMA_INMUX_INMUX[2];     
	volatile  uint32_t  RESERVED[6];            
	volatile uint32_t  SCT0_INMUX[4];          
} LPC_INMUX_T;



 
typedef enum {
	DMA_INMUX_0,   
	DMA_INMUX_1,   
}DMA_INMUX_T;



 
typedef enum {
	SCT_INMUX_0,    
	SCT_INMUX_1,    
	SCT_INMUX_2,    
	SCT_INMUX_3,    
} SCT_INMUX_T;



 
typedef enum {
	SCT_INP_IN0,                   
	SCT_INP_IN1,                 
	SCT_INP_IN2,                 
	SCT_INP_IN3,                 
	SCT_INP_ADC_THCMP_IRQ,       
	SCT_INP_ACMP_O,              
	SCT_INP_ARM_TXEV,            
	SCT_INP_DEBUG_HALTED,        
} SCT_INP_T;







 
static __inline void Chip_INMUX_SetDMAOTrig(LPC_INMUX_T *pINMUX, DMA_INMUX_T imux, DMA_CHID_T ch)
{
	pINMUX->DMA_INMUX_INMUX[imux] = ch;
}







 
static __inline void Chip_INMUX_SetSCTInMux(LPC_INMUX_T *pINMUX, SCT_INMUX_T isct, SCT_INP_T trig)
{
	pINMUX->SCT0_INMUX[isct] = trig;
}



 



 
typedef struct {					 
	volatile uint32_t  DMA_ITRIG_INMUX[(DMA_CH17 + 1)];	 
} LPC_DMATRIGMUX_T;

 
typedef enum {
	DMATRIG_ADC_SEQA_IRQ = 0,			 
	DMATRIG_ADC_SEQB_IRQ,				 
	DMATRIG_SCT0_DMA0,					 
	DMATRIG_SCT0_DMA1,					 
	DMATRIG_ACMP_O,						 
	DMATRIG_PINT0,						 
	DMATRIG_PINT1,						 
	DMATRIG_DMA_INMUX0,					 
	DMATRIG_DMA_INMUX1,					 
} DMA_TRIGSRC_T;










 
static __inline void Chip_DMATRIGMUX_SetInputTrig(LPC_DMATRIGMUX_T *pDMATRIG, DMA_CHID_T ch, DMA_TRIGSRC_T trig)
{
	pDMATRIG->DMA_ITRIG_INMUX[ch] = (uint32_t) trig;
}



 



 





#line 202 ".\\lpc_chip_82x\\inc\\chip.h"





 



 
extern uint32_t SystemCoreClock;





 
void SystemCoreClockUpdate(void);






 
void Chip_SystemInit(void);






 
void Chip_SetupXtalClocking(void);




 
void Chip_SetupIrcClocking(void);



 





#line 33 "lpc_chip_82x\\src\\adc_8xx.c"



 



 



 



 

 
void Chip_ADC_Init(LPC_ADC_T *pADC, uint32_t flags)
{
	 
	Chip_SYSCTL_PowerUp((1 << 4));
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_ADC);

	 
	pADC->INTEN = 0;

	 
	pADC->CTRL = flags;
}

 
void Chip_ADC_DeInit(LPC_ADC_T *pADC)
{
	pADC->INTEN = 0;
	pADC->CTRL = 0;

	 
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_ADC);
	Chip_SYSCTL_PowerDown((1 << 4));
}

 
void Chip_ADC_StartCalibration(LPC_ADC_T *pADC)
{
	 
	pADC->CTRL |= (1 << 30);

	 
	pADC->CTRL &= ~(1 << 8);

	 
	Chip_ADC_SetClockRate(pADC, 500000);

	 
	pADC->CTRL &= ~(1 << 10);

	 
}
