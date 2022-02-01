/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-07-24     Tanek        the first version
 * 2018-11-12     Ernest Chen  modify copyright
 */
#include <rthw.h>
#include <gd32vf103.h>
#include "riscv_encoding.h"

#include "n200_timer.h"

// #define TMR_MSIP 0xFFC
// #define TMR_MSIP_size   0x4
// #define TMR_MTIMECMP 0x8
// #define TMR_MTIMECMP_size 0x8
// #define TMR_MTIME 0x0
// #define TMR_MTIME_size 0x8

// #define TMR_CTRL_ADDR           0xd1000000
// #define TMR_REG(offset)         _REG32(TMR_CTRL_ADDR, offset)
// #define TMR_FREQ                ((uint32_t)SystemCoreClock/4)  //units HZ

void usart0_init(void);

void riscv_clock_init(void)
{
    SystemInit();

    /* ECLIC init */
    eclic_init(ECLIC_NUM_INTERRUPTS);
    eclic_mode_enable();
    set_csr(mstatus, MSTATUS_MIE);
}

static void ostick_config(rt_uint32_t ticks)
{
    /* set value */
    *(rt_uint64_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP) = ticks;
    /* enable interrupt */
    eclic_irq_enable(CLIC_INT_TMR, 0, 0);
    /* clear value */
    *(rt_uint64_t *)(TIMER_CTRL_ADDR + TIMER_MTIME) = 0;
}

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
#define RT_HEAP_SIZE 1024
static uint32_t rt_heap[RT_HEAP_SIZE];  // heap default size: 4K(1024 * 4)
RT_WEAK void *rt_heap_begin_get(void)
{
    return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
    return rt_heap + RT_HEAP_SIZE;
}
#endif

/**
 * This function will initial your board.
 */
void rt_hw_board_init()
{
    /* system clock Configuration */
    riscv_clock_init();

    /* start usart0 */
    usart0_init();

    /* OS Tick Configuration */
    ostick_config(TIMER_FREQ / RT_TICK_PER_SECOND);

    /* Call components board initial (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
}

/* This is the timer interrupt service routine. */
void eclic_mtip_handler(void)
{
    /* clear value */
    *(rt_uint64_t *)(TIMER_CTRL_ADDR + TIMER_MTIME) = 0;

    /* enter interrupt */
    rt_interrupt_enter();
    /* tick increase */
    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

void rt_hw_console_output(const char *str)
{
    char ch = *str++;
    while(ch != '\0')
    {
        usart_data_transmit(USART0, (uint8_t) ch);
        while (usart_flag_get(USART0, USART_FLAG_TBE) == RESET)
        {
        }

        if('\n' == ch)
        {
            ch = '\r';
        }
        else
        {
            ch = *str++;
        }
    }

}

void usart0_init(void)
{
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);
    
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
        
    /* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);
}
