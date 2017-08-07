#include <rthw.h>
#include <rtthread.h>
#include <components.h>
#include "board.h"
#include "uart_app.h"
#include <stdio.h>

//#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#define INT_UART0_TM4C123       21          // UART0
#define INT_UART1_TM4C123       22          // UART1
#define INT_UART2_TM4C123       49          // UART2
#define INT_UART3_TM4C123       72          // UART3
#define INT_UART4_TM4C123       73          // UART4
#define INT_UART5_TM4C123       74          // UART5
#define INT_UART6_TM4C123       75          // UART6
#define INT_UART7_TM4C123       76          // UART7


#define PRINTF(...)  

#ifdef __GNUC__
// With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar()
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
	#if USED_PEIRNT
	while(!UARTSpaceAvail(UART0_BASE + DEBUG_UART * 0x1000));
	UARTCharPutNonBlocking((UART0_BASE + DEBUG_UART * 0x1000), (uint8_t)ch);
	return ch;
	#else
	return ch;
	#endif
	
}


extern struct rt_messagequeue mq_uart;
extern struct rt_messagequeue mq_spi;

enum
{
	STOP = 0,
	START ,
};


struct Uart_struct_t Uart_Struct_Data[UART_MAX_DATA];


void Uart_ctol(struct Uart_struct_t * pUart_t)	//串口构造函数
{
	rt_memset(pUart_t, 0 , sizeof(struct Uart_struct_t));
	rb_new(&pUart_t->uart_rx_rb);
	rb_new(&pUart_t->uart_tx_rb);
	
	pUart_t->uart_timeout.EnableSignal = STOP;
	pUart_t->uart_timeout.UartPackTime = DEFAULT_PACKT_TIME;	
}


static void Uart_recv_timerout_handler(struct Uart_struct_t * pUart_t)
{
	static rt_err_t ret;
	
	if(pUart_t == NULL )	return;
	if(pUart_t->uart_n >= UART_MAX_DATA) 	return;
	if(pUart_t->uart_timeout.EnableSignal == STOP)return;
	else
	{
		pUart_t->uart_timeout.TimerCount++;
		if(pUart_t->uart_timeout.TimerCount > pUart_t->uart_timeout.UartPackTime)
		{
			struct Uart_tx_info_t uartx_info;
			
			pUart_t->uart_timeout.EnableSignal = STOP;
			pUart_t->uart_timeout.TimerCount = 0;
			
			uartx_info.uart_n = pUart_t->uart_n;
			uartx_info.rx_date_len = rb_can_read(&pUart_t->uart_rx_rb);
	
			ret = rt_mq_send(&mq_uart, (void *)&uartx_info, sizeof(struct Uart_tx_info_t));
			if (ret == -RT_EFULL)
			{
				PRINTF("uart mq full \n");
			}
		}		
	}
}

void Uart_recv_timerout_irq_handler(void)
{
	
	#ifdef RT_USED_UART0
	Uart_recv_timerout_handler(&Uart_Struct_Data[UART0_DATA]);
	#endif
	
	#ifdef RT_USED_UART1
	Uart_recv_timerout_handler(&Uart_Struct_Data[UART1_DATA]);
	#endif
	
	#ifdef RT_USED_UART2
	Uart_recv_timerout_handler(&Uart_Struct_Data[UART2_DATA]);
	#endif
	
	#ifdef RT_USED_UART3
	Uart_recv_timerout_handler(&Uart_Struct_Data[UART3_DATA]);
	#endif
	
	#ifdef RT_USED_UART4
	Uart_recv_timerout_handler(&Uart_Struct_Data[UART4_DATA]);
	#endif
	
	#ifdef RT_USED_UART5
	Uart_recv_timerout_handler(&Uart_Struct_Data[UART5_DATA]);
	#endif
	
	#ifdef RT_USED_UART6
	Uart_recv_timerout_handler(&Uart_Struct_Data[UART6_DATA]);
	#endif
	
	#ifdef RT_USED_UART7
	Uart_recv_timerout_handler(&Uart_Struct_Data[UART7_DATA]);
	#endif

}	


void UARTX_RX_IRQHandler(struct Uart_struct_t * pUart_t, const void *value, rt_uint8_t num)
{		
	if(pUart_t == NULL || num >= UART_MAX_DATA)	return;
	
	if(rb_can_write(&pUart_t->uart_rx_rb) > 0)
	{
		rb_write(&pUart_t->uart_rx_rb, value, 1);
	}
	pUart_t->uart_n = num;
	pUart_t->uart_timeout.EnableSignal = START;
	pUart_t->uart_timeout.TimerCount = 0;
}

void UARTX_TX_IRQHandler(struct Uart_struct_t * pUart_t, rt_int32_t recv )
{

}

void UART0_IRQHandler(void)
{
	  uint32_t ui32Status;
    /* enter interrupt */
    rt_interrupt_enter();

    ui32Status = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, ui32Status);
	
	  if(ui32Status & (UART_INT_RX | UART_INT_RT))
    {
			while(UARTCharsAvail(UART0_BASE))
			{
				rt_int32_t temp = 0;
				
				temp = UARTCharGetNonBlocking(UART0_BASE);	
				if(temp != -1)
					UARTX_RX_IRQHandler(&Uart_Struct_Data[UART0_DATA], &temp, UART0_DATA);
			}
		}
	  if(ui32Status & UART_INT_TX)
    {
			while(rb_can_read(&Uart_Struct_Data[UART0_DATA].uart_tx_rb) && UARTSpaceAvail(UART0_BASE))
			{
				rt_uint8_t value= 0;
				
				rb_read(&Uart_Struct_Data[UART0_DATA].uart_tx_rb, &value, 1);
				UARTCharPut(UART0_BASE, value);
			}		
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

void UART1_IRQHandler(void)
{
	  uint32_t ui32Status;
    /* enter interrupt */
    rt_interrupt_enter();

    ui32Status = UARTIntStatus(UART1_BASE, true);
    UARTIntClear(UART1_BASE, ui32Status);
	
	  if(ui32Status & (UART_INT_RX | UART_INT_RT))
    {
			while(UARTCharsAvail(UART1_BASE))
			{
				rt_int32_t temp = 0;
				
				temp = UARTCharGetNonBlocking(UART1_BASE);	
				if(temp != -1)
					UARTX_RX_IRQHandler(&Uart_Struct_Data[UART1_DATA], &temp, UART1_DATA);
			}
		}
	  if(ui32Status & UART_INT_TX)
    {
			while(rb_can_read(&Uart_Struct_Data[UART1_DATA].uart_tx_rb) && UARTSpaceAvail(UART1_BASE))
			{
				rt_uint8_t value= 0;
				
				rb_read(&Uart_Struct_Data[UART1_DATA].uart_tx_rb, &value, 1);
				UARTCharPut(UART1_BASE, value);
			}		
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

void UART2_IRQHandler(void)
{
	  uint32_t ui32Status;
    /* enter interrupt */
    rt_interrupt_enter();

    ui32Status = UARTIntStatus(UART2_BASE, true);
    UARTIntClear(UART2_BASE, ui32Status);
	
	  if(ui32Status & (UART_INT_RX | UART_INT_RT))
    {
			while(UARTCharsAvail(UART2_BASE))
			{
				rt_int32_t temp = 0;
				
				temp = UARTCharGetNonBlocking(UART2_BASE);	
				if(temp != -1)
					UARTX_RX_IRQHandler(&Uart_Struct_Data[UART2_DATA], &temp, UART2_DATA);
			}
		}
	  if(ui32Status & UART_INT_TX)
    {
			while(rb_can_read(&Uart_Struct_Data[UART2_DATA].uart_tx_rb) && UARTSpaceAvail(UART2_BASE))
			{
				rt_uint8_t value= 0;
				
				rb_read(&Uart_Struct_Data[UART2_DATA].uart_tx_rb, &value, 1);
				UARTCharPut(UART2_BASE, value);
			}	
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

void UART3_IRQHandler(void)
{
	  uint32_t ui32Status;
    /* enter interrupt */
    rt_interrupt_enter();

    ui32Status = UARTIntStatus(UART3_BASE, true);
    UARTIntClear(UART3_BASE, ui32Status);
	
	  if(ui32Status & (UART_INT_RX | UART_INT_RT))
    {
			while(UARTCharsAvail(UART3_BASE))
			{
				rt_int32_t temp = 0;
				
				temp = UARTCharGetNonBlocking(UART3_BASE);	
				if(temp != -1)
					UARTX_RX_IRQHandler(&Uart_Struct_Data[UART3_DATA], &temp, UART3_DATA);	
			}
		}
	  if(ui32Status & UART_INT_TX)
    {
			while(rb_can_read(&Uart_Struct_Data[UART3_DATA].uart_tx_rb) && UARTSpaceAvail(UART3_BASE))
			{
				rt_uint8_t value= 0;
				
				rb_read(&Uart_Struct_Data[UART3_DATA].uart_tx_rb, &value, 1);
				UARTCharPut(UART3_BASE, value);
			}	
    }
    /* leave interrupt */
    rt_interrupt_leave();
}


void UART4_IRQHandler(void)
{
	  uint32_t ui32Status;
    /* enter interrupt */
    rt_interrupt_enter();

    ui32Status = UARTIntStatus(UART4_BASE, true);
    UARTIntClear(UART4_BASE, ui32Status);
	
	  if(ui32Status & (UART_INT_RX | UART_INT_RT))
    {
			while(UARTCharsAvail(UART4_BASE))
			{
				rt_int32_t temp = 0;
				
				temp = UARTCharGetNonBlocking(UART4_BASE);	
				if(temp != -1)
					UARTX_RX_IRQHandler(&Uart_Struct_Data[UART4_DATA], &temp, UART4_DATA);	
			}
		}
	  if(ui32Status & UART_INT_TX)
    {
			while(rb_can_read(&Uart_Struct_Data[UART4_DATA].uart_tx_rb) && UARTSpaceAvail(UART4_BASE))
			{
				rt_uint8_t value= 0;
				
				rb_read(&Uart_Struct_Data[UART4_DATA].uart_tx_rb, &value, 1);
				UARTCharPut(UART4_BASE, value);
			}	

    }
    /* leave interrupt */
    rt_interrupt_leave();
}


void UART5_IRQHandler(void)
{
	  uint32_t ui32Status;
    /* enter interrupt */
    rt_interrupt_enter();

    ui32Status = UARTIntStatus(UART5_BASE, true);
    UARTIntClear(UART5_BASE, ui32Status);
	
	  if(ui32Status & (UART_INT_RX | UART_INT_RT))
    {
			while(UARTCharsAvail(UART5_BASE))
			{
				rt_int32_t temp = 0;
				
				temp = UARTCharGetNonBlocking(UART5_BASE);	
				if(temp != -1)
					UARTX_RX_IRQHandler(&Uart_Struct_Data[UART5_DATA], &temp, UART5_DATA);
			}
		}
	  if(ui32Status & UART_INT_TX)
    {
			while(rb_can_read(&Uart_Struct_Data[UART5_DATA].uart_tx_rb) && UARTSpaceAvail(UART5_BASE))
			{
				rt_uint8_t value= 0;
				
				rb_read(&Uart_Struct_Data[UART5_DATA].uart_tx_rb, &value, 1);
				UARTCharPut(UART5_BASE, value);
			}	
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

void UART6_IRQHandler(void)
{
	  uint32_t ui32Status;
    /* enter interrupt */
    rt_interrupt_enter();

    ui32Status = UARTIntStatus(UART6_BASE, true);
    UARTIntClear(UART6_BASE, ui32Status);
	
	  if(ui32Status & (UART_INT_RX | UART_INT_RT))
    {
			while(UARTCharsAvail(UART6_BASE))
			{
				rt_int32_t temp = 0;
				
				temp = UARTCharGetNonBlocking(UART6_BASE);	
				if(temp != -1)
					UARTX_RX_IRQHandler(&Uart_Struct_Data[UART6_DATA], &temp, UART6_DATA);
			}
		}
	  if(ui32Status & UART_INT_TX)
    {
			while(rb_can_read(&Uart_Struct_Data[UART6_DATA].uart_tx_rb) && UARTSpaceAvail(UART6_BASE))
			{
				rt_uint8_t value= 0;
				
				rb_read(&Uart_Struct_Data[UART6_DATA].uart_tx_rb, &value, 1);
				UARTCharPut(UART6_BASE, value);
			}	
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

void UART7_IRQHandler(void)
{
	  uint32_t ui32Status;
    /* enter interrupt */
    rt_interrupt_enter();

    ui32Status = UARTIntStatus(UART7_BASE, true);
    UARTIntClear(UART7_BASE, ui32Status);
	
	  if(ui32Status & (UART_INT_RX | UART_INT_RT))
    {
			while(UARTCharsAvail(UART7_BASE))
			{
				rt_int32_t temp = 0;
				
				temp = UARTCharGetNonBlocking(UART7_BASE);	
				if(temp != -1)
					UARTX_RX_IRQHandler(&Uart_Struct_Data[UART7_DATA], &temp, UART7_DATA);
			}
		}
	  if(ui32Status & UART_INT_TX)
    {
			while(rb_can_read(&Uart_Struct_Data[UART7_DATA].uart_tx_rb) && UARTSpaceAvail(UART7_BASE))
			{
				rt_uint8_t value= 0;
				
				rb_read(&Uart_Struct_Data[UART7_DATA].uart_tx_rb, &value, 1);
				UARTCharPut(UART7_BASE, value);
			}	
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

static void rt_hw_uart_sendu8(rt_uint8_t uart_n, rt_uint8_t c)
{
    UARTIntDisable(UART0_BASE + uart_n * 0x1000, UART_INT_TX);
    //
    // write single char
		if(UARTSpaceAvail(UART0_BASE + uart_n * 0x1000))
		{
			if(rb_can_read(&Uart_Struct_Data[uart_n].uart_tx_rb) == 0) 
			{
					UARTCharPut(UART0_BASE + uart_n * 0x1000, c);
			}
			else 
			{
				rt_uint8_t value = 0;
				rb_read(&Uart_Struct_Data[uart_n].uart_tx_rb, &value, 1);
				UARTCharPut(UART0_BASE + uart_n * 0x1000, value);
				rb_write(&Uart_Struct_Data[uart_n].uart_tx_rb, &c, 1);
			}
		}
		else
		{
			rb_write(&Uart_Struct_Data[uart_n].uart_tx_rb, &c, 1);
		}
    // enable tx interrupt
    UARTIntEnable(UART0_BASE + uart_n * 0x1000, UART_INT_TX);
}

void rt_hw_uart_send(rt_uint8_t uart_n, const rt_uint8_t *pui8Buffer, rt_uint32_t ui32Count)
{
	if(uart_n >= UART_MAX_DATA ) return;
	
	while(ui32Count--)
	{
		#if USED_PEIRNT //&& DEBUG_UART == uart_n , 宏替换发生在预编译阶段
		if(DEBUG_UART == uart_n)
		{
			while(!UARTSpaceAvail(UART0_BASE + uart_n * 0x1000));
			UARTCharPutNonBlocking((UART0_BASE + uart_n * 0x1000), *pui8Buffer++);
		}
		else
			rt_hw_uart_sendu8(uart_n, *pui8Buffer++);
		#else
		rt_hw_uart_sendu8(uart_n, *pui8Buffer++);
		#endif
	}
}


/*
static const unsigned int RtsPin[UART_MAX_DATA] = 
{
	GPIO_PB5_U0RTS,
	GPIO_PN0_U1RTS,
	GPIO_PD6_U2RTS,
	GPIO_PP4_U3RTS,
	
};

static const unsigned int CtsPin[UART_MAX_DATA] = 
{
	GPIO_PB4_U0CTS,
	GPIO_PN1_U1CTS,
	GPIO_PD7_U2CTS,
	GPIO_PP5_U3CTS,
	
};

static const unsigned int RxPin[UART_MAX_DATA] = 
{

};

static const unsigned int TxPin[UART_MAX_DATA] = 
{

};
*/

//static void uart_config_init(struct *)
//{

//}

static void rt_hw_uartgpio_init(void)
{
	#ifdef RT_USED_UART0
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	// Set GPIO A0 and A1 as UART pins
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	#endif
	
	#ifdef RT_USED_UART1
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	// Set GPIO A0 and A1 as UART pins
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	#endif
	
	#ifdef RT_USED_UART2
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	// Set GPIO A0 and A1 as UART pins
	GPIOPinConfigure(GPIO_PD4_U2RX);
	GPIOPinConfigure(GPIO_PD5_U2TX);
	GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	#endif
	
	#ifdef RT_USED_UART3
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	// Set GPIO A0 and A1 as UART pins
	GPIOPinConfigure(GPIO_PA4_U3RX);
	GPIOPinConfigure(GPIO_PA5_U3TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	#endif
	
	#ifdef RT_USED_UART4
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	// Set GPIO A0 and A1 as UART pins
	GPIOPinConfigure(GPIO_PA2_U4RX);
	GPIOPinConfigure(GPIO_PA3_U4TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3);
	#endif
	
	#ifdef RT_USED_UART5
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	// Set GPIO A0 and A1 as UART pins
	GPIOPinConfigure(GPIO_PC6_U5RX);
	GPIOPinConfigure(GPIO_PC7_U5TX);
	GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	#endif
	
	#ifdef RT_USED_UART6
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
	// Set GPIO A0 and A1 as UART pins
	GPIOPinConfigure(GPIO_PP0_U6RX);
	GPIOPinConfigure(GPIO_PP1_U6TX);
	GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	#endif
	
	#ifdef RT_USED_UART7
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	// Set GPIO A0 and A1 as UART pins
	GPIOPinConfigure(GPIO_PC4_U7RX);
	GPIOPinConfigure(GPIO_PC5_U7TX);
	GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	#endif
	
	#ifdef N540

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);

	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_4, 0);

	GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_5);
	GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_5, 0);

	GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_6);
	GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_6, 0);

	GPIOPinTypeGPIOOutput(GPIO_PORTQ_BASE, GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTQ_BASE, GPIO_PIN_4, 0);
	#endif
}

void rt_hw_uart_init(void)
{
	rt_hw_uartgpio_init();
	#ifdef RT_USED_UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    IntMasterEnable();

    UARTConfigSetExpClk(UART0_BASE, SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
    //
    // Enable the UART interrupt.
    //
    IntEnable(INT_UART0);
		#if (USED_PEIRNT && DEBUG_UART == 0)
		UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
		#else
		UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
		#endif
	
		IntRegister(INT_UART0_TM4C123, UART0_IRQHandler);

		UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX4_8, UART_FIFO_RX1_8);      
		UARTFIFOEnable(UART0_BASE);

	
		Uart_ctol(&Uart_Struct_Data[UART0_DATA]);
    //rt_hw_uart_send( UART0_DATA, (rt_uint8_t *)"UART0_DATA INIT\r\n", rt_strlen("UART0_DATA INIT\r\n"));
		#endif
		
	#ifdef RT_USED_UART1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    IntMasterEnable();
		
    UARTConfigSetExpClk(UART1_BASE, SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    IntEnable(INT_UART1);
		#if (USED_PEIRNT && DEBUG_UART == 1)
		UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
		#else
		UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
		#endif
	
		IntRegister(INT_UART1_TM4C123, UART1_IRQHandler);

		UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX4_8, UART_FIFO_RX1_8);      
		UARTFIFOEnable(UART1_BASE);


		Uart_ctol(&Uart_Struct_Data[UART1_DATA]);
    //rt_hw_uart_send( UART1_DATA, (rt_uint8_t *)"UART1_DATA INIT\r\n", rt_strlen("UART1_DATA INIT\r\n"));
		#endif
		
	#ifdef RT_USED_UART2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);

    IntMasterEnable();
		
    UARTConfigSetExpClk(UART2_BASE, SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    IntEnable(INT_UART2);
		#if (USED_PEIRNT && DEBUG_UART == 2)
		UARTIntEnable(UART2_BASE, UART_INT_RX | UART_INT_RT);
		#else
		UARTIntEnable(UART2_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
		#endif
	
		IntRegister(INT_UART2_TM4C123, UART2_IRQHandler);

		UARTFIFOLevelSet(UART2_BASE, UART_FIFO_TX4_8, UART_FIFO_RX1_8);      
		UARTFIFOEnable(UART2_BASE);


		Uart_ctol(&Uart_Struct_Data[UART2_DATA]);
    //rt_hw_uart_send( UART2_DATA, (rt_uint8_t *)"UART2_DATA INIT\r\n", rt_strlen("UART2_DATA INIT\r\n"));
		#endif
		
	#ifdef RT_USED_UART3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);

    IntMasterEnable();
		
    UARTConfigSetExpClk(UART3_BASE, SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    IntEnable(INT_UART3);
		#if (USED_PEIRNT && DEBUG_UART == 3)
		UARTIntEnable(UART3_BASE, UART_INT_RX | UART_INT_RT);
		#else
		UARTIntEnable(UART3_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
		#endif
	
		IntRegister(INT_UART3_TM4C123, UART3_IRQHandler);

		UARTFIFOLevelSet(UART3_BASE, UART_FIFO_TX4_8, UART_FIFO_RX1_8);      
		UARTFIFOEnable(UART3_BASE);



		Uart_ctol(&Uart_Struct_Data[UART3_DATA]);
    //rt_hw_uart_send( UART3_DATA, (rt_uint8_t *)"UART3_DATA INIT\r\n", rt_strlen("UART3_DATA INIT\r\n"));
		#endif
		
	#ifdef RT_USED_UART4
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);

    IntMasterEnable();
		
    UARTConfigSetExpClk(UART4_BASE, SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    IntEnable(INT_UART4);
		#if (USED_PEIRNT && DEBUG_UART == 4)
		UARTIntEnable(UART4_BASE, UART_INT_RX | UART_INT_RT);
		#else
		UARTIntEnable(UART4_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
		#endif
	
		IntRegister(INT_UART4_TM4C123, UART4_IRQHandler);

		UARTFIFOLevelSet(UART4_BASE, UART_FIFO_TX4_8, UART_FIFO_RX1_8);      
		UARTFIFOEnable(UART4_BASE);


		Uart_ctol(&Uart_Struct_Data[UART4_DATA]);
    //rt_hw_uart_send( UART4_DATA, (rt_uint8_t *)"UART4_DATA INIT\r\n", rt_strlen("UART4_DATA INIT\r\n"));
		#endif
		
	#ifdef RT_USED_UART5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);

    IntMasterEnable();
		
    UARTConfigSetExpClk(UART5_BASE, SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    IntEnable(INT_UART5);
		#if (USED_PEIRNT && DEBUG_UART == 5)
		UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT);
		#else
		UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
		#endif
	
		IntRegister(INT_UART5_TM4C123, UART5_IRQHandler);

		UARTFIFOLevelSet(UART5_BASE, UART_FIFO_TX4_8, UART_FIFO_RX1_8);      
		UARTFIFOEnable(UART5_BASE);


		Uart_ctol(&Uart_Struct_Data[UART5_DATA]);
   // rt_hw_uart_send( UART5_DATA, (rt_uint8_t *)"UART5_DATA INIT\r\n", rt_strlen("UART5_DATA INIT\r\n"));
		#endif
		
	#ifdef RT_USED_UART6
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);

    IntMasterEnable();
		
    UARTConfigSetExpClk(UART6_BASE, SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    IntEnable(INT_UART6);
		#if (USED_PEIRNT && DEBUG_UART == 6)
		UARTIntEnable(UART6_BASE, UART_INT_RX | UART_INT_RT);
		#else
		UARTIntEnable(UART6_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
		#endif
	
		IntRegister(INT_UART6_TM4C123, UART6_IRQHandler);

		UARTFIFOLevelSet(UART6_BASE, UART_FIFO_TX4_8, UART_FIFO_RX1_8);      
		UARTFIFOEnable(UART6_BASE);


		Uart_ctol(&Uart_Struct_Data[UART6_DATA]);
    //rt_hw_uart_send( UART6_DATA, (rt_uint8_t *)"UART6_DATA INIT\r\n", rt_strlen("UART6_DATA INIT\r\n"));
		#endif
		
	#ifdef RT_USED_UART7
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);

    IntMasterEnable();
		
    UARTConfigSetExpClk(UART7_BASE, SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    IntEnable(INT_UART7);
		#if (USED_PEIRNT && DEBUG_UART == 7)
		UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT);
		#else
		UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
		#endif
	
		IntRegister(INT_UART7_TM4C123, UART7_IRQHandler);

		UARTFIFOLevelSet(UART7_BASE, UART_FIFO_TX4_8, UART_FIFO_RX1_8);      
		UARTFIFOEnable(UART7_BASE);


		Uart_ctol(&Uart_Struct_Data[UART7_DATA]);
    //rt_hw_uart_send( UART7_DATA, (rt_uint8_t *)"UART7_DATA INIT\r\n", rt_strlen("UART7_DATA INIT\r\n"));
		#endif
}


void uart_2_spi_thread(void *parameter)
{
	rt_err_t err;
	static struct Uart_tx_info_t Obtain_uart_mq;
	static rt_uint8_t uart_buf[1500];
	rt_uint8_t ret;
	
	(void)parameter;
	

	while(1)
	{
		rt_memset(&Obtain_uart_mq, 0, sizeof(struct Uart_tx_info_t));
		err = rt_mq_recv(&mq_uart, &Obtain_uart_mq, sizeof(struct Uart_tx_info_t), RT_WAITING_FOREVER);
		if(err == RT_EOK) 
		{
			rt_uint16_t i, len, len_ret;
		
			
			PRINTF("uart[%d] , send date len = %d \n", Obtain_uart_mq.uart_n, Obtain_uart_mq.rx_date_len);
			
			len = (sizeof(uart_buf) > Obtain_uart_mq.rx_date_len)? Obtain_uart_mq.rx_date_len : sizeof(uart_buf);
			
			len_ret = rb_read(&Uart_Struct_Data[Obtain_uart_mq.uart_n].uart_rx_rb, uart_buf, len);
			if(len_ret != len) PRINTF("ringbuf read err\n");
			
			rt_hw_uart_send( Obtain_uart_mq.uart_n, uart_buf, len);
			
			for(i = 0; i < len; i++)
			{				
				PRINTF("%02x ",(uart_buf[ len ]));
			}
			PRINTF("\r\n");
    }
		else		
		{
			PRINTF("uart to spi thread err \r\n");
		}
	}
}

void spi_2_uart_thread(void *parameter)
{
	rt_err_t err;
	static struct Uart_rx_info_t Obtain_spi_mq;
	static rt_uint8_t uart_buf[1500];
	rt_uint8_t ret;
	
	(void)parameter;

	while(1)
	{
		rt_memset(&Obtain_spi_mq, 0, sizeof(struct Uart_rx_info_t));
		err = rt_mq_recv(&mq_spi, &Obtain_spi_mq, sizeof(struct Uart_rx_info_t), RT_WAITING_FOREVER);
		if(err == RT_EOK) 
		{
			rt_uint16_t i, len, len_ret;
		
			
			PRINTF("uart[%d] , recv date len = %d \n", Obtain_spi_mq.uart_n, Obtain_spi_mq.rx_date_len);
			
			len = (sizeof(uart_buf) > Obtain_spi_mq.rx_date_len)? Obtain_spi_mq.rx_date_len : sizeof(uart_buf);
			
			//先写串口FIFO,再写ringbuf
			len_ret = rb_write(&Uart_Struct_Data[Obtain_spi_mq.uart_n].uart_tx_rb, uart_buf, len);
			
			if(len_ret != len) PRINTF("ringbuf write err\n");
			
			for(i = 0; i < len; i++)
			{				
				PRINTF("%02x ",(uart_buf[ len ]));
			}
			PRINTF("\r\n");
    }
		else		
		{
		  PRINTF("spi to uart thread err \r\n");
		}
	}
}
