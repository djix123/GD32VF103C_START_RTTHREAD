/*!
    \file  main.c
    \brief running led
    
    \version 2019-6-5, V1.0.0, firmware for GD32VF103
*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32vf103.h"
#include "rtconfig.h"
#include "rtthread.h"

//#define __GD32VF103C_START__
//#define __SIPEED_LONGAN_NANO__

/* Built-in LED of GD32VF103C_START Boards is PA7 */
#ifdef __GD32VF103C_START__
#define LED_PIN GPIO_PIN_7
#define LED_GPIO_PORT GPIOA
#define LED_GPIO_CLK RCU_GPIOA

#define WAKEUP_KEY_PIN GPIO_PIN_0
#define WAKEUP_KEY_GPIO_PORT GPIOA
#define WAKEUP_KEY_GPIO_CLK RCU_GPIOA
#endif

#ifdef __SIPEED_LONGAN_NANO__
#define LED_PIN GPIO_PIN_13
#define LED_GPIO_PORT GPIOC
#define LED_GPIO_CLK RCU_GPIOC

#define __LED_INVERT__
#endif

// void delay_1ms(uint32_t count);
void led_init(void);
void led_on(void);
void led_off(void);

void button_init(void);

void thread1_entry(void *parameter);
void thread2_entry(void *parameter);

static volatile bool button_pressed = FALSE;

void led_init()
{
    /* enable the led clock */
    rcu_periph_clock_enable(LED_GPIO_CLK);
    /* configure led GPIO port */ 
    gpio_init(LED_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_PIN);
    /* Turn led off */
    led_off();
}

void led_off()
{
#ifndef __LED_INVERT__
    GPIO_BC(LED_GPIO_PORT) = LED_PIN;
#else
    GPIO_BOP(LED_GPIO_PORT) = LED_PIN;
#endif
}

void led_on()
{
#ifndef __LED_INVERT__
    GPIO_BOP(LED_GPIO_PORT) = LED_PIN;
#else
    GPIO_BC(LED_GPIO_PORT) = LED_PIN;
#endif
}

void button_init()
{
    /* enable the Wakeup clock */
    rcu_periph_clock_enable(WAKEUP_KEY_GPIO_CLK);
    rcu_periph_clock_enable(RCU_AF);

    /* configure button pin as input */
    gpio_init(WAKEUP_KEY_GPIO_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, WAKEUP_KEY_PIN);
 }

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    rt_thread_t tid1, tid2;

    led_init();
    button_init();

    // when setting thread1 priority set less than main thread priority *or*
    // add rt_thread_mdelay to yield to thread1
    // main thread priority = RT_THREAD_PRIORITY_MAX / 3 (see components.c)
    tid1 = rt_thread_create("thread1", thread1_entry, RT_NULL,
                           RT_MAIN_THREAD_STACK_SIZE, RT_THREAD_PRIORITY_MAX / 4, 20);
    RT_ASSERT(tid1 != RT_NULL);

    rt_thread_startup(tid1);

    tid2 = rt_thread_create("thread2", thread2_entry, RT_NULL,
                           RT_MAIN_THREAD_STACK_SIZE, RT_THREAD_PRIORITY_MAX / 4 - 1, 20);
    RT_ASSERT(tid2 != RT_NULL);

    rt_thread_startup(tid2);

    while(1)
    {
        // if main thread priority < thread1 priority
        // need to add delay so thread1 gets scheduled
        //rt_thread_mdelay(1);
    }
}


void thread1_entry(void *parameter)
{
    while(1)
    {
        /* turn on builtin led */
        led_on();
        rt_thread_mdelay( (button_pressed == FALSE ? 10 : 500) );
        /* turn off uiltin led */
        led_off();
        rt_thread_mdelay(500);
    }
}

void thread2_entry(void *parameter)
{
    while(1)
    {
        FlagStatus button = gpio_input_bit_get(WAKEUP_KEY_GPIO_PORT, WAKEUP_KEY_PIN);
        if(SET == button)
        {
            if(FALSE == button_pressed)
            {
                rt_kprintf("Pressed\n");
            }
            button_pressed = TRUE;
        }
        else
        {
            if(TRUE == button_pressed)
            {
                rt_kprintf("Released\n");
            }
            button_pressed = FALSE;
        }

        rt_thread_mdelay(50);
    }
}