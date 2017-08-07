#ifndef __UART_APP_H__
#define __UART_APP_H__

#include "rtdef.h"
#include "ringbuffer.h"

#define N540

#define USED_PEIRNT 0
#define DEBUG_UART 0

#define RT_USED_UART0
#define RT_USED_UART1
#define RT_USED_UART2
#define RT_USED_UART3
#define RT_USED_UART4
#define RT_USED_UART5
#define RT_USED_UART6
#define RT_USED_UART7

#define DEFAULT_PACKT_TIME 3 //默认串口打包时间 3ms

enum
{
	UART0_DATA = 0,
	UART1_DATA ,
	UART2_DATA ,
	UART3_DATA ,
	UART4_DATA ,
	UART5_DATA ,
	UART6_DATA ,
	UART7_DATA ,
	UART_MAX_DATA 
};

//计时打包
struct Time_Count_t
{
	rt_uint16_t	UartPackTime;
	rt_uint16_t TimerCount;
	rt_uint8_t  EnableSignal;	
};
	
struct Uart_struct_t
{
	struct RingBuffer_t uart_tx_rb;
	struct RingBuffer_t uart_rx_rb;
	struct Time_Count_t uart_timeout;
	rt_uint8_t uart_n;
};

struct Uart_tx_info_t
{
	rt_uint16_t rx_date_len;
	rt_uint8_t uart_n;
};

struct Uart_rx_info_t
{
	rt_uint16_t rx_date_len;
	rt_uint8_t uart_n;
};

extern struct Uart_struct_t Uart_Struct_Data[UART_MAX_DATA];




void rt_hw_uart_init(void);
void rt_hw_uart_send(rt_uint8_t uart_n, const rt_uint8_t *pui8Buffer, rt_uint32_t ui32Count);
void Uart_recv_timerout_irq_handler(void);

void uart_2_spi_thread(void *parameter);
void spi_2_uart_thread(void *parameter);

#endif
