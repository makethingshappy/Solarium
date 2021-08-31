#line 1 "lpc_chip_82x\\src\\ring_buffer.c"





























 

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



 

#line 33 "lpc_chip_82x\\src\\ring_buffer.c"
#line 1 ".\\lpc_chip_82x\\inc\\ring_buffer.h"





























 




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




 

#line 34 "lpc_chip_82x\\src\\ring_buffer.c"



 






 



 



 

 
int RingBuffer_Init(RINGBUFF_T *RingBuff, void *buffer, int itemSize, int count)
{
	RingBuff->data = buffer;
	RingBuff->count = count;
	RingBuff->itemSz = itemSize;
	RingBuff->head = RingBuff->tail = 0;

	return 1;
}

 
int RingBuffer_Insert(RINGBUFF_T *RingBuff, const void *data)
{
	uint8_t *ptr = RingBuff->data;

	 
	if (RingBuffer_IsFull(RingBuff))
		return 0;

	ptr += ((RingBuff)->head & ((RingBuff)->count - 1)) * RingBuff->itemSz;
	memcpy(ptr, data, RingBuff->itemSz);
	RingBuff->head++;

	return 1;
}

 
int RingBuffer_InsertMult(RINGBUFF_T *RingBuff, const void *data, int num)
{
	uint8_t *ptr = RingBuff->data;
	int cnt1, cnt2;

	 
	if (RingBuffer_IsFull(RingBuff))
		return 0;

	 
	cnt1 = cnt2 = RingBuffer_GetFree(RingBuff);
	if (((RingBuff)->head & ((RingBuff)->count - 1)) + cnt1 >= RingBuff->count)
		cnt1 = RingBuff->count - ((RingBuff)->head & ((RingBuff)->count - 1));
	cnt2 -= cnt1;

	cnt1 = (((cnt1) < (num)) ? (cnt1) : (num));
	num -= cnt1;

	cnt2 = (((cnt2) < (num)) ? (cnt2) : (num));
	num -= cnt2;

	 
	ptr += ((RingBuff)->head & ((RingBuff)->count - 1)) * RingBuff->itemSz;
	memcpy(ptr, data, cnt1 * RingBuff->itemSz);
	RingBuff->head += cnt1;

	 
	ptr = (uint8_t *) RingBuff->data + ((RingBuff)->head & ((RingBuff)->count - 1)) * RingBuff->itemSz;
	data = (const uint8_t *) data + cnt1 * RingBuff->itemSz;
	memcpy(ptr, data, cnt2 * RingBuff->itemSz);
	RingBuff->head += cnt2;

	return cnt1 + cnt2;
}

 
int RingBuffer_Pop(RINGBUFF_T *RingBuff, void *data)
{
	uint8_t *ptr = RingBuff->data;

	 
	if (RingBuffer_IsEmpty(RingBuff))
		return 0;

	ptr += ((RingBuff)->tail & ((RingBuff)->count - 1)) * RingBuff->itemSz;
	memcpy(data, ptr, RingBuff->itemSz);
	RingBuff->tail++;

	return 1;
}

 
int RingBuffer_PopMult(RINGBUFF_T *RingBuff, void *data, int num)
{
	uint8_t *ptr = RingBuff->data;
	int cnt1, cnt2;

	 
	if (RingBuffer_IsEmpty(RingBuff))
		return 0;

	 
	cnt1 = cnt2 = RingBuffer_GetCount(RingBuff);
	if (((RingBuff)->tail & ((RingBuff)->count - 1)) + cnt1 >= RingBuff->count)
		cnt1 = RingBuff->count - ((RingBuff)->tail & ((RingBuff)->count - 1));
	cnt2 -= cnt1;

	cnt1 = (((cnt1) < (num)) ? (cnt1) : (num));
	num -= cnt1;

	cnt2 = (((cnt2) < (num)) ? (cnt2) : (num));
	num -= cnt2;

	 
	ptr += ((RingBuff)->tail & ((RingBuff)->count - 1)) * RingBuff->itemSz;
	memcpy(data, ptr, cnt1 * RingBuff->itemSz);
	RingBuff->tail += cnt1;

	 
	ptr = (uint8_t *) RingBuff->data + ((RingBuff)->tail & ((RingBuff)->count - 1)) * RingBuff->itemSz;
	data = (uint8_t *) data + cnt1 * RingBuff->itemSz;
	memcpy(data, ptr, cnt2 * RingBuff->itemSz);
	RingBuff->tail += cnt2;

	return cnt1 + cnt2;
}
