#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

//#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/tiva_timer.h"
#include "board.h"
#include "uart_app.h"

#define INT_TIMER0A_TM4C123     35          // 16/32-Bit Timer 0A
#define INT_TIMER0B_TM4C123     36          // 16/32-Bit Timer 0B
#define INT_TIMER1A_TM4C123     37          // 16/32-Bit Timer 1A
#define INT_TIMER1B_TM4C123     38          // 16/32-Bit Timer 1B
#define INT_TIMER2A_TM4C123     39          // 16/32-Bit Timer 2A
#define INT_TIMER2B_TM4C123     40          // 16/32-Bit Timer 2B


static uint32_t g_ui32Flags;


void Timer0_IRQHandler(void)
{
	char cOne, cTwo;
	
	/* enter interrupt */
	rt_interrupt_enter();
	
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	//
	// Toggle the flag for the first timer.
	//
	HWREGBITW(&g_ui32Flags, 0) ^= 1;

	IntMasterDisable();
	cOne = HWREGBITW(&g_ui32Flags, 0) ? '1' : '0';
	cTwo = HWREGBITW(&g_ui32Flags, 1) ? '1' : '0';
	//printf("\rT1: %c  T2: %c \n", cOne, cTwo);
	Uart_recv_timerout_irq_handler();
	
	IntMasterEnable();

	/* leave interrupt */
	rt_interrupt_leave();
}

//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************
void Timer1_IRQHandler(void)
{
	char cOne, cTwo;

	/* enter interrupt */
	rt_interrupt_enter();

	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	HWREGBITW(&g_ui32Flags, 1) ^= 1;


	IntMasterDisable();
	cOne = HWREGBITW(&g_ui32Flags, 0) ? '1' : '0';
	cTwo = HWREGBITW(&g_ui32Flags, 1) ? '1' : '0';
	printf("\rT1: %c  T2: %c \n", cOne, cTwo);
	IntMasterEnable();

	/* leave interrupt */
	rt_interrupt_leave();
}



//*****************************************************************************
//
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//
//*****************************************************************************
void rt_hw_time_init(void)
{
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Configure the two 32-bit periodic timers.
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    //TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysClock/1000);
    //TimerLoadSet(TIMER1_BASE, TIMER_A, SysClock / 2);

    //
    // Setup the interrupts for the timer timeouts.
    //
    IntEnable(INT_TIMER0A);
    //IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    //TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

		IntRegister(INT_TIMER0A_TM4C123, Timer0_IRQHandler);
		//IntRegister(INT_TIMER1A_TM4C123, Timer1_IRQHandler);
    //
    // Enable the timers.
    //
    TimerEnable(TIMER0_BASE, TIMER_A);
    //TimerEnable(TIMER1_BASE, TIMER_A);
}
