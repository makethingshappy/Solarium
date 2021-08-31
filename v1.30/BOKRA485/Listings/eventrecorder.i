#line 1 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"







 
 


 
#line 1 "C:\\Users\\Peter\\Documents\\Projects\\Workspace-Keil\\LPC824\\BOKRA485\\v1.10\\RTE\\RTE_Components.h"







 







 





#line 14 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"





























































 








 





 

 





 




 

 


typedef enum IRQn {
   
  NotAvail_IRQn                = -128,              

   
  NonMaskableInt_IRQn          = -14,               
  HardFault_IRQn               = -13,               
  SVCall_IRQn                  = -5,                
  PendSV_IRQn                  = -2,                
  SysTick_IRQn                 = -1,                

   
  SPI0_IRQn                    = 0,                 
  SPI1_IRQn                    = 1,                 
  Reserved18_IRQn              = 2,                 
  USART0_IRQn                  = 3,                 
  USART1_IRQn                  = 4,                 
  USART2_IRQn                  = 5,                 
  Reserved22_IRQn              = 6,                 
  I2C1_IRQn                    = 7,                 
  I2C0_IRQn                    = 8,                 
  SCT0_IRQn                    = 9,                 
  MRT0_IRQn                    = 10,                
  CMP_CAPT_IRQn                = 11,                
  WDT_IRQn                     = 12,                
  BOD_IRQn                     = 13,                
  FLASH_IRQn                   = 14,                
  WKT_IRQn                     = 15,                
  ADC0_SEQA_IRQn               = 16,                
  ADC0_SEQB_IRQn               = 17,                
  ADC0_THCMP_IRQn              = 18,                
  ADC0_OVR_IRQn                = 19,                
  DMA0_IRQn                    = 20,                
  I2C2_IRQn                    = 21,                
  I2C3_IRQn                    = 22,                
  Reserved39_IRQn              = 23,                
  PIN_INT0_IRQn                = 24,                
  PIN_INT1_IRQn                = 25,                
  PIN_INT2_IRQn                = 26,                
  PIN_INT3_IRQn                = 27,                
  PIN_INT4_IRQn                = 28,                
  PIN_INT5_IRQn                = 29,                
  PIN_INT6_IRQn                = 30,                
  PIN_INT7_IRQn                = 31                 
} IRQn_Type;



   




 




 







#line 1 ".\\_CMSIS\\v5.20\\Include\\core_cm0plus.h"
 




 
















 










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



 










#line 162 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\system_LPC824.h"































































 










 








#line 85 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\system_LPC824.h"








extern uint32_t g_Sys_Pll_Freq;









 
extern uint32_t SystemCoreClock;







 
void SystemInit (void);







 
void SystemCoreClockUpdate (void);





#line 163 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"



   




 




 

 



 



 







 
typedef enum _dma_request_source
{
    kDmaRequestUSART0_RX_DMA        = 0U,           
    kDmaRequestUSART0_TX_DMA        = 1U,           
    kDmaRequestUSART1_RX_DMA        = 2U,           
    kDmaRequestUSART1_TX_DMA        = 3U,           
    kDmaRequestUSART2_RX_DMA        = 4U,           
    kDmaRequestUSART2_TX_DMA        = 5U,           
    kDmaRequestSPI0_RX_DMA          = 6U,           
    kDmaRequestSPI0_TX_DMA          = 7U,           
    kDmaRequestSPI1_RX_DMA          = 8U,           
    kDmaRequestSPI1_TX_DMA          = 9U,           
    kDmaRequestI2C0_SLV_DMA         = 10U,          
    kDmaRequestI2C0_MST_DMA         = 11U,          
    kDmaRequestI2C1_SLV_DMA         = 12U,          
    kDmaRequestI2C1_MST_DMA         = 13U,          
    kDmaRequestI2C2_SLV_DMA         = 14U,          
    kDmaRequestI2C2_MST_DMA         = 15U,          
    kDmaRequestI2C3_SLV_DMA         = 16U,          
    kDmaRequestI2C3_MST_DMA         = 17U,          
} dma_request_source_t;

 




   




 




 




 





    #pragma push
    #pragma anon_unions
#line 253 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"



 




 

 
typedef struct {
  volatile uint32_t CTRL;                               
  volatile uint32_t LAD;                                
} ACOMP_Type;



 




 

 
 
#line 304 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 317 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 




   


 
 

 

 

 




   




 




 

 
typedef struct {
  volatile uint32_t CTRL;                               
       uint8_t RESERVED_0[4];
  volatile uint32_t SEQ_CTRL[2];                        
  volatile uint32_t SEQ_GDAT[2];                        
       uint8_t RESERVED_1[8];
  volatile const  uint32_t DAT[12];                            
  volatile uint32_t THR0_LOW;                           
  volatile uint32_t THR1_LOW;                           
  volatile uint32_t THR0_HIGH;                          
  volatile uint32_t THR1_HIGH;                          
  volatile uint32_t CHAN_THRSEL;                        
  volatile uint32_t INTEN;                              
  volatile uint32_t FLAGS;                              
  volatile uint32_t TRM;                                
} ADC_Type;



 




 

 
 
#line 387 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 421 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 


 
 
#line 446 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 


 
 
#line 471 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 


 
 



 

 
 



 

 
 



 

 
 



 

 
 
#line 542 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 591 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 685 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 




   


 
 

 

 

 

 





   




 




 

 
typedef struct {
  volatile uint32_t MODE;                               
  volatile uint32_t SEED;                               
  union {                                           
    volatile const  uint32_t SUM;                                
    volatile  uint32_t WR_DATA;                            
  };
} CRC_Type;



 




 

 
 
#line 763 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 



 

 
 



 




   


 
 

 

 

 




   




 




 

 
typedef struct {
  volatile uint32_t CTRL;                               
  volatile const  uint32_t INTSTAT;                            
  volatile uint32_t SRAMBASE;                           
       uint8_t RESERVED_0[20];
  struct {                                          
    volatile uint32_t ENABLESET;                          
         uint8_t RESERVED_0[4];
    volatile uint32_t ENABLECLR;                          
         uint8_t RESERVED_1[4];
    volatile const  uint32_t ACTIVE;                             
         uint8_t RESERVED_2[4];
    volatile const  uint32_t BUSY;                               
         uint8_t RESERVED_3[4];
    volatile uint32_t ERRINT;                             
         uint8_t RESERVED_4[4];
    volatile uint32_t INTENSET;                           
         uint8_t RESERVED_5[4];
    volatile uint32_t INTENCLR;                           
         uint8_t RESERVED_6[4];
    volatile uint32_t INTA;                               
         uint8_t RESERVED_7[4];
    volatile uint32_t INTB;                               
         uint8_t RESERVED_8[4];
    volatile uint32_t SETVALID;                           
         uint8_t RESERVED_9[4];
    volatile uint32_t SETTRIG;                            
         uint8_t RESERVED_10[4];
    volatile uint32_t ABORT;                              
  } COMMON[1];
       uint8_t RESERVED_1[900];
  struct {                                          
    volatile uint32_t CFG;                                
    volatile const  uint32_t CTLSTAT;                            
    volatile uint32_t XFERCFG;                            
         uint8_t RESERVED_0[4];
  } CHANNEL[18];
} DMA_Type;



 




 

 
 



 

 
 
#line 880 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 



 

 


 
 



 

 


 
 



 

 


 
 



 

 


 
 



 

 


 
 



 

 


 
 



 

 


 
 



 

 


 
 



 

 


 
 



 

 


 
 



 

 


 
 



 

 


 
 
#line 1038 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 


 
 
#line 1051 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 


 
 
#line 1088 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 





   


 
 

 

 

 

 




   




 




 

 
typedef struct {
       uint8_t RESERVED_0[16];
  volatile uint32_t FLASHCFG;                           
       uint8_t RESERVED_1[12];
  volatile uint32_t FMSSTART;                           
  volatile uint32_t FMSSTOP;                            
       uint8_t RESERVED_2[4];
  volatile const  uint32_t FMSW0;                              
       uint8_t RESERVED_3[4016];
  volatile const  uint32_t FMSTAT;                             
       uint8_t RESERVED_4[4];
  volatile  uint32_t FMSTATCLR;                          
} FLASH_CTRL_Type;



 




 

 
 



 

 
 



 

 
 
#line 1171 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 



 

 
 



 




   


 
 

 

 

 




   




 




 

 
typedef struct {
  volatile uint8_t B[1][29];                            
       uint8_t RESERVED_0[4067];
  volatile uint32_t W[1][29];                           
       uint8_t RESERVED_1[3980];
  volatile uint32_t DIR[1];                             
       uint8_t RESERVED_2[124];
  volatile uint32_t MASK[1];                            
       uint8_t RESERVED_3[124];
  volatile uint32_t PIN[1];                             
       uint8_t RESERVED_4[124];
  volatile uint32_t MPIN[1];                            
       uint8_t RESERVED_5[124];
  volatile uint32_t SET[1];                             
       uint8_t RESERVED_6[124];
  volatile  uint32_t CLR[1];                             
       uint8_t RESERVED_7[124];
  volatile  uint32_t NOT[1];                             
       uint8_t RESERVED_8[124];
  volatile  uint32_t DIRSET[1];                          
       uint8_t RESERVED_9[124];
  volatile  uint32_t DIRCLR[1];                          
       uint8_t RESERVED_10[124];
  volatile  uint32_t DIRNOT[1];                          
} GPIO_Type;



 




 

 
 



 

 


 


 
 



 

 


 


 
 



 

 


 
 



 

 


 
 



 

 


 
 



 

 


 
 



 

 


 
 



 

 


 
 



 

 


 
 



 

 


 
 



 

 


 
 



 

 





   


 
 

 

 

 




   




 




 

 
typedef struct {
  volatile uint32_t CFG;                                
  volatile uint32_t STAT;                               
  volatile uint32_t INTENSET;                           
  volatile  uint32_t INTENCLR;                           
  volatile uint32_t TIMEOUT;                            
  volatile uint32_t CLKDIV;                             
  volatile const  uint32_t INTSTAT;                            
       uint8_t RESERVED_0[4];
  volatile uint32_t MSTCTL;                             
  volatile uint32_t MSTTIME;                            
  volatile uint32_t MSTDAT;                             
       uint8_t RESERVED_1[20];
  volatile uint32_t SLVCTL;                             
  volatile uint32_t SLVDAT;                             
  volatile uint32_t SLVADR[4];                          
  volatile uint32_t SLVQUAL0;                           
       uint8_t RESERVED_2[36];
  volatile const  uint32_t MONRXDAT;                           
} I2C_Type;



 




 

 
 
#line 1464 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 1516 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 1553 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 1590 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 1600 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 
#line 1644 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 1660 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 1670 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 
#line 1690 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 
#line 1707 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 


 
 
#line 1720 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 1736 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 




   


 
 

 

 

 

 

 

 

 

 

 

 




   




 




 

 
typedef struct {
  volatile uint32_t DMA_ITRIG_INMUX[18];                
       uint8_t RESERVED_0[16312];
  volatile uint32_t DMA_INMUX_INMUX[2];                 
       uint8_t RESERVED_1[24];
  volatile uint32_t SCT0_INMUX[4];                      
} INPUTMUX_Type;



 




 

 
 



 

 


 
 



 

 


 
 



 

 





   


 
 

 

 

 




   




 




 

 
typedef struct {
  volatile uint32_t PIO[30];                            
} IOCON_Type;



 




 

 
 
#line 1897 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 





   


 
 

 

 

 


#line 1947 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"




   




 




 

 
typedef struct {
  struct {                                          
    volatile uint32_t INTVAL;                             
    volatile const  uint32_t TIMER;                              
    volatile uint32_t CTRL;                               
    volatile uint32_t STAT;                               
  } CHANNEL[4];
       uint8_t RESERVED_0[176];
  volatile const  uint32_t MODCFG;                             
  volatile const  uint32_t IDLE_CH;                            
  volatile uint32_t IRQ_FLAG;                           
} MRT_Type;



 




 

 
 
#line 1994 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 


 
 



 

 


 
 
#line 2017 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 


 
 
#line 2030 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 


 
 
#line 2043 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 
#line 2066 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 




   


 
 

 

 

 




   




 




 

 
typedef struct {
  volatile uint32_t POSITION;                           
  volatile uint32_t MASTER;                             
  volatile uint32_t FLOW;                               
  volatile const  uint32_t BASE;                               
} MTB_Type;



 




 

 
 
#line 2123 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 2148 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 2161 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 




   


 
 

 

 

 




   




 




 

 
typedef struct {
  volatile uint32_t ISEL;                               
  volatile uint32_t IENR;                               
  volatile  uint32_t SIENR;                              
  volatile  uint32_t CIENR;                              
  volatile uint32_t IENF;                               
  volatile  uint32_t SIENF;                              
  volatile  uint32_t CIENF;                              
  volatile uint32_t RISE;                               
  volatile uint32_t FALL;                               
  volatile uint32_t IST;                                
  volatile uint32_t PMCTRL;                             
  volatile uint32_t PMSRC;                              
  volatile uint32_t PMCFG;                              
} PINT_Type;



 




 

 
 



 

 
 



 

 
 



 

 
 



 

 
 



 

 
 



 

 
 



 

 
 



 

 
 



 

 
 



 

 
 
#line 2307 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 2335 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 2384 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 




   


 
 

 

 

 

 




   




 




 

 
typedef struct {
  volatile uint32_t PCON;                               
  volatile uint32_t GPREG[4];                           
  volatile uint32_t DPDCTRL;                            
} PMU_Type;



 




 

 
 
#line 2448 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 


 
 
#line 2480 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 




   


 
 

 

 

 




   




 




 

 
typedef struct {
  volatile uint32_t CONFIG;                             
  volatile uint32_t CTRL;                               
  volatile uint32_t LIMIT;                              
  volatile uint32_t HALT;                               
  volatile uint32_t STOP;                               
  volatile uint32_t START;                              
       uint8_t RESERVED_0[40];
  volatile uint32_t COUNT;                              
  volatile uint32_t STATE;                              
  volatile const  uint32_t INPUT;                              
  volatile uint32_t REGMODE;                            
  volatile uint32_t OUTPUT;                             
  volatile uint32_t OUTPUTDIRCTRL;                      
  volatile uint32_t RES;                                
  volatile uint32_t DMA0REQUEST;                        
  volatile uint32_t DMA1REQUEST;                        
       uint8_t RESERVED_1[140];
  volatile uint32_t EVEN;                               
  volatile uint32_t EVFLAG;                             
  volatile uint32_t CONEN;                              
  volatile uint32_t CONFLAG;                            
  union {                                           
    volatile uint32_t SCTCAP[8];                          
    volatile uint32_t SCTMATCH[8];                        
  };
       uint8_t RESERVED_2[224];
  union {                                           
    volatile uint32_t SCTCAPCTRL[8];                      
    volatile uint32_t SCTMATCHREL[8];                     
  };
       uint8_t RESERVED_3[224];
  struct {                                          
    volatile uint32_t STATE;                              
    volatile uint32_t CTRL;                               
  } EVENT[8];
       uint8_t RESERVED_4[448];
  struct {                                          
    volatile uint32_t SET;                                
    volatile uint32_t CLR;                                
  } OUT[6];
} SCT_Type;



 




 

 
 
#line 2591 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 2631 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 2641 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 2651 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 2661 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 2671 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 2681 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 2691 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 2719 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 2729 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 
#line 2758 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 2780 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 2793 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 2806 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 



 

 
 



 

 
 
#line 2840 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 2850 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 


 
 
#line 2863 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 


 
 
#line 2876 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 


 
 
#line 2889 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 


 
 



 

 


 
 
#line 2936 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 


 
 



 

 


 
 



 

 





   


 
 

 

 

 

 




   




 




 

 
typedef struct {
  volatile uint32_t CFG;                                
  volatile uint32_t DLY;                                
  volatile uint32_t STAT;                               
  volatile uint32_t INTENSET;                           
  volatile  uint32_t INTENCLR;                           
  volatile const  uint32_t RXDAT;                              
  volatile uint32_t TXDATCTL;                           
  volatile uint32_t TXDAT;                              
  volatile uint32_t TXCTL;                              
  volatile uint32_t DIV;                                
  volatile const  uint32_t INTSTAT;                            
} SPI_Type;



 




 

 
 
#line 3049 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3065 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3096 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3121 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3146 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3168 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3199 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 
#line 3234 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 
#line 3266 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 




   


 
 

 

 

 

 

 

 




   




 




 

 
typedef struct {
  union {                                           
    struct {                                          
      volatile uint32_t PINASSIGN0;                         
      volatile uint32_t PINASSIGN1;                         
      volatile uint32_t PINASSIGN2;                         
      volatile uint32_t PINASSIGN3;                         
      volatile uint32_t PINASSIGN4;                         
      volatile uint32_t PINASSIGN5;                         
      volatile uint32_t PINASSIGN6;                         
      volatile uint32_t PINASSIGN7;                         
      volatile uint32_t PINASSIGN8;                         
      volatile uint32_t PINASSIGN9;                         
      volatile uint32_t PINASSIGN10;                        
      volatile uint32_t PINASSIGN11;                        
    } ;
    volatile uint32_t PINASSIGN_DATA[12];                 
  };
       uint8_t RESERVED_0[400];
  volatile uint32_t PINENABLE0;                         
} SWM_Type;



 




 

 
 
#line 3350 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3366 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3382 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3398 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3414 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3430 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3446 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3462 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3478 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3494 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3510 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3526 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3542 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 


 
 
#line 3624 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 




   


 
 

 

 

 




   




 




 

 
typedef struct {
  volatile uint32_t SYSMEMREMAP;                        
  volatile uint32_t PRESETCTRL;                         
  volatile uint32_t SYSPLLCTRL;                         
  volatile const  uint32_t SYSPLLSTAT;                         
       uint8_t RESERVED_0[16];
  volatile uint32_t SYSOSCCTRL;                         
  volatile uint32_t WDTOSCCTRL;                         
  volatile uint32_t IRCCTRL;                            
       uint8_t RESERVED_1[4];
  volatile uint32_t SYSRSTSTAT;                         
       uint8_t RESERVED_2[12];
  volatile uint32_t SYSPLLCLKSEL;                       
  volatile uint32_t SYSPLLCLKUEN;                       
       uint8_t RESERVED_3[40];
  volatile uint32_t MAINCLKSEL;                         
  volatile uint32_t MAINCLKUEN;                         
  volatile uint32_t SYSAHBCLKDIV;                       
       uint8_t RESERVED_4[4];
  volatile uint32_t SYSAHBCLKCTRL;                      
       uint8_t RESERVED_5[16];
  volatile uint32_t UARTCLKDIV;                         
       uint8_t RESERVED_6[72];
  volatile uint32_t CLKOUTSEL;                          
  volatile uint32_t CLKOUTUEN;                          
  volatile uint32_t CLKOUTDIV;                          
       uint8_t RESERVED_7[4];
  volatile uint32_t UARTFRGDIV;                         
  volatile uint32_t UARTFRGMULT;                        
       uint8_t RESERVED_8[4];
  volatile uint32_t EXTTRACECMD;                        
  volatile const  uint32_t PIOPORCAP0;                         
       uint8_t RESERVED_9[48];
  volatile uint32_t IOCONCLKDIV6;                       
  volatile uint32_t IOCONCLKDIV5;                       
  volatile uint32_t IOCONCLKDIV4;                       
  volatile uint32_t IOCONCLKDIV3;                       
  volatile uint32_t IOCONCLKDIV2;                       
  volatile uint32_t IOCONCLKDIV1;                       
  volatile uint32_t IOCONCLKDIV0;                       
  volatile uint32_t BODCTRL;                            
  volatile uint32_t SYSTCKCAL;                          
       uint8_t RESERVED_10[24];
  volatile uint32_t IRQLATENCY;                         
  volatile uint32_t NMISRC;                             
  volatile uint32_t PINTSEL[8];                         
       uint8_t RESERVED_11[108];
  volatile uint32_t STARTERP0;                          
       uint8_t RESERVED_12[12];
  volatile uint32_t STARTERP1;                          
       uint8_t RESERVED_13[24];
  volatile uint32_t PDSLEEPCFG;                         
  volatile uint32_t PDAWAKECFG;                         
  volatile uint32_t PDRUNCFG;                           
       uint8_t RESERVED_14[444];
  volatile const  uint32_t DEVICE_ID;                          
} SYSCON_Type;



 




 

 
 



 

 
 
#line 3787 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3797 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 
#line 3814 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 3824 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 
#line 3850 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 



 

 
 



 

 
 



 

 
 



 

 
 
#line 3967 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 



 

 
 



 

 
 



 

 
 



 

 
 



 

 
 
#line 4019 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 



 

 
 



 

 
 



 

 
 



 

 
 



 

 
 



 

 
 



 

 
 
#line 4088 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 



 

 
 
#line 4112 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 


 
 
#line 4150 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 4190 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 4200 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 4231 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 4262 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 




   


 
 

 

 

 




   




 




 

 
typedef struct {
  volatile uint32_t CFG;                                
  volatile uint32_t CTL;                                
  volatile uint32_t STAT;                               
  volatile uint32_t INTENSET;                           
  volatile  uint32_t INTENCLR;                           
  volatile const  uint32_t RXDAT;                              
  volatile const  uint32_t RXDATSTAT;                          
  volatile uint32_t TXDAT;                              
  volatile uint32_t BRG;                                
  volatile const  uint32_t INTSTAT;                            
  volatile uint32_t OSR;                                
  volatile uint32_t ADDR;                               
} USART_Type;



 




 

 
 
#line 4373 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 4395 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 4444 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 4484 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 
#line 4524 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 
#line 4547 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 



 

 
 
#line 4601 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 



 




   


 
 

 

 

 

 

 

 

 

 




   




 




 

 
typedef struct {
  volatile uint32_t CTRL;                               
       uint8_t RESERVED_0[8];
  volatile uint32_t COUNT;                              
} WKT_Type;



 




 

 
 
#line 4687 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 




   


 
 

 

 

 

 




   




 




 

 
typedef struct {
  volatile uint32_t MOD;                                
  volatile uint32_t TC;                                 
  volatile  uint32_t FEED;                               
  volatile const  uint32_t TV;                                 
       uint8_t RESERVED_0[4];
  volatile uint32_t WARNINT;                            
  volatile uint32_t WINDOW;                             
} WWDT_Type;



 




 

 
 
#line 4768 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"
 

 
 



 

 
 



 

 
 



 

 
 



 

 
 



 




   


 
 

 

 

 




   




 





    #pragma pop
#line 4843 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"



   




 




 

#line 4865 "C:\\Keil_v5\\ARM\\PACK\\Keil\\LPC800_DFP\\1.10.1\\Device\\Include\\LPC824.h"






 






 




   




 




 

 



   




#line 15 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"
 
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"


  



    typedef unsigned int size_t;    
#line 54 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"




extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 193 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 209 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 232 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 247 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 270 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 502 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"



 

#line 17 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Include\\EventRecorder.h"







 




#line 14 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Include\\EventRecorder.h"




















#line 40 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Include\\EventRecorder.h"






extern uint32_t EventRecorderTimerSetup (void);



extern uint32_t EventRecorderTimerGetFreq (void);



extern uint32_t EventRecorderTimerGetCount (void);








extern uint32_t EventRecorderInitialize (uint32_t recording, uint32_t start);






extern uint32_t EventRecorderEnable (uint32_t recording, uint32_t comp_start, uint32_t comp_end);






extern uint32_t EventRecorderDisable (uint32_t recording, uint32_t comp_start, uint32_t comp_end);



extern uint32_t EventRecorderStart (void);



extern uint32_t EventRecorderStop (void);



extern uint32_t EventRecorderClockUpdate (void);









extern uint32_t EventRecordData (uint32_t id, const void *data, uint32_t len);






extern uint32_t EventRecord2 (uint32_t id, uint32_t val1, uint32_t val2);








extern uint32_t EventRecord4 (uint32_t id, uint32_t val1, uint32_t val2, uint32_t val3, uint32_t val4);








































































#line 18 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"
#line 1 "C:\\Users\\Peter\\Documents\\Projects\\Workspace-Keil\\LPC824\\BOKRA485\\v1.10\\RTE\\Compiler\\EventRecorderConf.h"







 




































#line 19 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"
 



 
#line 34 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"
 
#line 46 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"
 

#line 57 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"
 
 

 
 




 





 
 

 
 




 
 

 
 





 
 
#line 109 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"
 
 
typedef struct {
  uint32_t ts;                  
  uint32_t val1;                
  uint32_t val2;                
  uint32_t info;                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
} EventRecord_t;
 
 
static EventRecord_t EventBuffer[64U] __attribute__ ((section (".bss.noinit"), zero_init)) __attribute__((aligned(16)));
 
 
static uint8_t EventFilter[128] __attribute__ ((section (".bss.noinit"), zero_init)) __attribute__((aligned(4)));
 
 
typedef struct {
  uint8_t  state;               
  uint8_t  context;             
  uint16_t info_crc;            
  uint32_t record_index;        
  uint32_t records_written;     
  uint32_t records_dumped;      
  uint32_t ts_overflow;         
  uint32_t ts_freq;             
  uint32_t ts_last;             
  uint32_t init_count;          
  uint32_t signature;           
} EventStatus_t;
 
static EventStatus_t EventStatus __attribute__ ((section (".bss.noinit"), zero_init)) __attribute__((aligned(64)));
 
 
typedef struct {
  uint8_t    protocol_type;     
  uint8_t    reserved;          
  uint16_t   protocol_version;  
  
  uint32_t       record_count;  
  EventRecord_t *event_buffer;  
  uint8_t       *event_filter;  
  EventStatus_t *event_status;  
  uint8_t        ts_source;     
  uint8_t        reserved3[3];  
} EventRecorderInfo_t;
 




extern const EventRecorderInfo_t EventRecorderInfo;
__attribute__((used)) const EventRecorderInfo_t EventRecorderInfo =
{
  1U, 0U,
  0x0101U,                      
  64U,
  &EventBuffer[0],
  &EventFilter[0],
  &EventStatus,
  0,
  { 0U, 0U, 0U }
};

 
 
 
 

 
static __inline uint8_t atomic_inc8 (uint8_t *mem) {
  uint32_t primask = __get_PRIMASK();
  uint8_t  ret;
 
  __disable_irq();
  ret = *mem;
  *mem = ret + 1U;
  if (primask == 0U) {
    __enable_irq();
  }
 
  return ret;
}
 
static __inline uint32_t atomic_inc32 (uint32_t *mem) {
  uint32_t primask = __get_PRIMASK();
  uint32_t ret;
 
  __disable_irq();
  ret = *mem;
  *mem = ret + 1U;
  if (primask == 0U) {
    __enable_irq();
  }
 
  return ret;
}
 
static __inline uint32_t atomic_xch32 (uint32_t *mem, uint32_t val) {
  uint32_t primask = __get_PRIMASK();
  uint32_t ret;
 
  __disable_irq();
  ret = *mem;
  *mem = val;
  if (primask == 0U) {
    __enable_irq();
  }
 
  return ret;
}
 
#line 349 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"
 
 
static __inline uint32_t GetContext (void) {
  return ((uint32_t)atomic_inc8(&EventStatus.context));
}
 
static __inline uint32_t GetRecordIndex (void) {
  return (atomic_inc32(&EventStatus.record_index));
}
 
static __inline uint32_t UpdateTS (uint32_t ts) {
  return (atomic_xch32(&EventStatus.ts_last, ts));
}
 
static __inline void IncrementOverflowTS (void) {
  (void)atomic_inc32(&EventStatus.ts_overflow);
}
 
static __inline void IncrementRecordsWritten (void) {
  (void)atomic_inc32(&EventStatus.records_written);
}
 
static __inline void IncrementRecordsDumped (void) {
  (void)atomic_inc32(&EventStatus.records_dumped);
}
 
 

 
static __inline uint32_t LockRecord (uint32_t *mem, uint32_t info) {
  uint32_t primask = __get_PRIMASK();
  uint32_t val;
 
  __disable_irq();
  val = *mem;
  if ((val & 0x04000000U) == 0U) {
     val = (val & 0x80000000U) | info;
    *mem = val;
  } else {
     val = 0U;
  }
  if (primask == 0U) {
    __enable_irq();
  }
 
  return val;
}
 
static __inline uint32_t UnlockRecord (uint32_t *mem, uint32_t info) {
  uint32_t primask = __get_PRIMASK();
  uint32_t val;
  uint32_t ret;
 
  __disable_irq();
  val = *mem;
  if ((val & 0x04000000U) != 0U) {
    *mem = info;
     ret = 1U;
  } else {
     ret = 0U;
  }
  if (primask == 0U) {
    __enable_irq();
  }
 
  return ret;
}
 
#line 538 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"
 
 







 
static uint32_t EventRecordItem (uint32_t id, uint32_t ts, uint32_t val1, uint32_t val2) {
  EventRecord_t *record;
  uint32_t cnt, i;
  uint32_t info;
  uint32_t tbit;
  uint32_t seq;
 
  for (cnt = 7U; cnt != 0U; cnt--) {
    i = GetRecordIndex();
    record = &EventBuffer[i & (64U - 1U)];
    seq  = ((i / 64U) << 20) & 0x00F00000U;
    info = id                                    | 
           seq                                   |
           ((ts   >> 3) & 0x10000000U)   |
           ((val1 >> 2) & 0x20000000U) |
           ((val2 >> 1) & 0x40000000U) |
           0x08000000U                    |
           0x04000000U;
    info = LockRecord(&record->info, info);
    if ((info & 0x04000000U) != 0U) {
      info ^= 0x04000000U;
      info ^= 0x80000000U;
      tbit  = info & 0x80000000U;
      record->ts   = (ts   & ~0x80000000U) | tbit;
      record->val1 = (val1 & ~0x80000000U) | tbit;
      record->val2 = (val2 & ~0x80000000U) | tbit;
      if ((UnlockRecord(&record->info, info)) != 0U) {
        IncrementRecordsWritten();
        
        return 1U;
      } else {
        break;
      }
    }
  }
 
  IncrementRecordsDumped();
  return 0U;
}
 




 
static uint32_t EventGetTS (void) {
  uint32_t ts;
  uint32_t ts_last;
  uint32_t ts_last_prev;
 
  do {
    ts_last_prev = *((volatile uint32_t *)&EventStatus.ts_last);
    ts = EventRecorderTimerGetCount();
    ts_last = UpdateTS(ts);
  } while (ts_last != ts_last_prev);
 
  if (ts < ts_last) {
    IncrementOverflowTS();
  }
 
  return (ts);
}
 




 
static __inline uint32_t EventCheckFilter (uint32_t id) {
  uint32_t ret;
 
  if (EventStatus.state == 0U) {
    ret = 0U;
  } else {
    ret = ((uint32_t)EventFilter[(id >> (8 + 3)) & 0x7FU] >> ((id >> 8) & 0x7U)) & 1U;
  }
  return (ret);
}
 
 





 
static uint16_t crc16_ccitt (const uint8_t *data, uint32_t len) {
  uint16_t crc;
  uint32_t n;
 
  crc = 0xFFFFU;
  while (len != 0U) {
    
    crc ^= ((uint16_t)*data++ << 8);
    for (n = 8U; n != 0U; n--) {
      if ((crc & 0x8000U) != 0U) {
        crc <<= 1;
        crc  ^= 0x1021U;
      } else {
        crc <<= 1;
      }
    }
    len--;
  }
 
  return (crc);
}
 
 





static uint32_t TimeStamp __attribute__ ((section (".bss.noinit"), zero_init));

 
#line 727 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"
 
 



 

__attribute__((weak)) uint32_t EventRecorderTimerSetup (void) {
#line 741 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"
    TimeStamp = 0U;
    return 1U;
#line 750 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"
}

 



 

__attribute__((weak)) uint32_t EventRecorderTimerGetFreq (void) {




    return 0U;
#line 777 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"
}

 



 

__attribute__((weak)) uint32_t EventRecorderTimerGetCount (void) {




    return (TimeStamp++);
#line 804 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"
}

 
 





 
uint32_t EventRecorderInitialize (uint32_t recording, uint32_t start) {
  EventRecord_t *record;
  uint16_t crc;
  uint32_t freq;
  uint32_t ret;
  uint32_t ts;
  uint32_t n;
 
  EventStatus.state = 0U;
  memset(&EventFilter[0], 0, sizeof(EventFilter));
 
  crc = crc16_ccitt((const uint8_t *)&EventRecorderInfo, sizeof(EventRecorderInfo));
 
  if (EventStatus.signature != 0xE1A5276BU) {
    EventStatus.signature  = 0xE1A5276BU;
    EventStatus.info_crc   = crc;
    EventStatus.init_count = 1U;
  } else {
    if (EventStatus.info_crc != crc) {
      EventStatus.info_crc   = crc;
      EventStatus.init_count = 1U;
    } else {
      EventStatus.init_count++;
    }
  }
 
  if (EventStatus.init_count == 1U) {
    EventStatus.context         = 0U;
    EventStatus.record_index    = 0U;
    EventStatus.records_written = 0U;
    EventStatus.records_dumped  = 0U;
    memset(&EventBuffer[0], 0, sizeof(EventBuffer));
  } else {
    for (n = 0U; n < 64U; n++) {
      record = &EventBuffer[n];
      if ((record->info & 0x04000000U) != 0U) {
        record->info &= ~(0x04000000U | 0x08000000U);
      }
    }
  }
 
  if (EventStatus.init_count == 1U) {
    ret = EventRecorderTimerSetup();
    if (ret != 0U) {



        freq = EventRecorderTimerGetFreq();

    } else {
      freq = 0U;
    }
    EventStatus.ts_freq     = freq;
    EventStatus.ts_last     = 0U;
    EventStatus.ts_overflow = 0U;
  } else {




    ret = 1U;
#line 893 "C:\\Keil_v5\\ARM\\PACK\\Keil\\ARM_Compiler\\1.6.2\\Source\\EventRecorder.c"
  }
 
  if (ret != 0U) {
 
    (void)EventRecorderEnable(recording,      0x00U,            0xFEU);
    (void)EventRecorderEnable(0x0FU, 0xEFU, 0xEFU);
    (void)EventRecorderEnable(0x04U,  0xFEU,     0xFEU);
 
    ts = EventGetTS();
 
    (void)EventRecordItem((((uint32_t)0xFFU << 8) | 0x00U | 0x01000000U | 0x02000000U), ts, EventStatus.init_count, 0U);
 
    if (start != 0U) {
      (void)EventRecorderStart();
    }
  }
 
  return (ret);
}
 






 
uint32_t EventRecorderEnable (uint32_t recording, uint32_t comp_start, uint32_t comp_end) {
  uint32_t ofs;
  uint32_t i, j;
 
  if ((comp_start >= 0xFFU) || (comp_end >= 0xFFU)) {
    
    return 0U;
  }
 
  ofs = 0U;
  for (i = 0U; i < 4U; i++) {
    if ((recording & (1UL << i)) != 0U) {
      for (j = comp_start; j <= comp_end; j++) {
        EventFilter[ofs + (j >> 3)] |= (1U << (j & 0x7U));
      }
    }
    ofs += 32U;
  }
 
  return 1U;
}
 






 
uint32_t EventRecorderDisable (uint32_t recording, uint32_t comp_start, uint32_t comp_end) {
  uint32_t ofs;
  uint32_t i, j;
 
  if ((comp_start >= 0xFFU) || (comp_end >= 0xFFU)) {
    
    return 0U;
  }
 
  ofs = 0U;
  for (i = 0U; i < 4U; i++) {
    if ((recording & (1UL << i)) != 0U) {
      for (j = comp_start; j <= comp_end; j++) {
        EventFilter[ofs + (j >> 3)] &= ~(1U << (j & 0x7U));
      }
    }
    ofs += 32U;
  }
 
  return 1U;
}
 



 
uint32_t EventRecorderStart (void) {
  uint32_t ts;
 
  if (EventStatus.state != 0U) {
    
    return 1U;
  }
  EventStatus.state = 1U;
 
  ts = EventGetTS();
 
  (void)EventRecordItem((((uint32_t)0xFFU << 8) | 0x01U | 0x01000000U | 0x02000000U), ts, 0U, 0U);
 
  return 1U;
}
 



 
uint32_t EventRecorderStop (void) {
  uint32_t ts;
 
  if (EventStatus.state == 0U) {
    
    return 1U;
  }
  EventStatus.state = 0U;
 
  ts = EventGetTS();
 
  (void)EventRecordItem((((uint32_t)0xFFU << 8) | 0x02U | 0x01000000U | 0x02000000U), ts, 0U, 0U);
 
  return 1U;
}
 



 
uint32_t EventRecorderClockUpdate (void) {
  uint32_t ts;
 
  EventStatus.ts_freq = EventRecorderTimerGetFreq();
 
  ts = EventGetTS();
 
  (void)EventRecordItem((((uint32_t)0xFFU << 8) | 0x03U | 0x01000000U | 0x02000000U), ts, EventStatus.ts_freq, 0U);
 
  return 1U;
}
 






 
uint32_t EventRecordData (uint32_t id, const void *data, uint32_t len) {
  
  
  const uint8_t *dptr;
  uint32_t ts;
  uint32_t ctx;
  uint32_t val[2];
  uint32_t ret;
 
  if ((data == 0) || (len > ((64U / 4U) * 8U))) {
    
    return 0U;
  }
 
  if (EventCheckFilter(id) == 0U) {
    
    return 1U;
  }
 
  ts = EventGetTS();
 
  id &= 0x0000FFFFU;
  id |= (__get_IPSR() != 0U) ? 0x00080000U : 0U;
  
  dptr = (const uint8_t *)data;
 
  if (len == 0U) {
    ret = EventRecordItem(id, ts, 0U, 0U);
    
    return (ret);
  }
 
  if (len <= 8U) {
    val[0] = 0U;
    val[1] = 0U;
    memcpy(val, dptr, len);
    id |= (len << 16) & 0x00070000U;
    ret = EventRecordItem(id | 0x01000000U | 0x02000000U, ts, val[0], val[1]);
    
    return (ret);
  }
 
  ctx = (GetContext() << 16) & 0x00070000U;
 
  memcpy(val, dptr, 8U);
  dptr += 8U;
  len  -= 8U;
  id |= ctx;
  ret = EventRecordItem(id | 0x01000000U, ts, val[0], val[1]);
  if (ret == 0U) {
    
    return 0U;
  }
 
  
  id = 0xFF01U | ctx;
 
  while (len > 8U) {
    memcpy(val, dptr, 8U);
    dptr += 8U;
    len  -= 8U;
    ret = EventRecordItem(id, ts, val[0], val[1]);
    id++;
    if (ret == 0U) {
      
      return 0U;
    }
  }
 
  val[0] = 0U;
  val[1] = 0U;
  memcpy(val, dptr, len);
  id &= ~0xFF00U;
  id |= len << 8;
  ret = EventRecordItem(id | 0x02000000U, ts, val[0], val[1]);
 
  return (ret);
}
 






 
uint32_t EventRecord2 (uint32_t id, uint32_t val1, uint32_t val2) {
  uint32_t ts;
  uint32_t ret;
 
  if (EventCheckFilter(id) == 0U) {
    
    return 1U;
  }
 
  ts = EventGetTS();
 
  id &= 0x0000FFFFU;
  id |= (__get_IPSR() != 0U) ? 0x00080000U : 0U;
 
  ret = EventRecordItem(id | 0x01000000U | 0x02000000U, ts, val1, val2);
 
  return (ret);
}
 








 
uint32_t EventRecord4 (uint32_t id,
                       uint32_t val1, uint32_t val2, uint32_t val3, uint32_t val4) {
  uint32_t ts;
  uint32_t ctx;
  uint32_t ret;
 
  if (EventCheckFilter(id) == 0U) {
    
    return 1U;
  }
 
  ts = EventGetTS();
 
  id &= 0x0000FFFFU;
  id |= (__get_IPSR() != 0U) ? 0x00080000U : 0U;
  ctx = (GetContext() << 16) & 0x00070000U;
 
  ret = EventRecordItem(id | ctx | 0x01000000U, ts, val1, val2);
  if (ret == 0U) {
    
    return 0U;
  }
  ret = EventRecordItem(1U | ctx | 0x02000000U,  ts, val3, val4);
 
  return (ret);
}
