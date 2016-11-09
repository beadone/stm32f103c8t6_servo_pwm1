//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "Trace.h"

#include "Timer.h"
#include "BlinkLed.h"
#include "ExceptionHandlers.h"
#include "stm32f10x_conf.h"
#include "misc.h"

#define SYSCLK 72000000
#define PRESCALER 72

#define NUM 10
 int i,j;
  char name[NUM+1] = {'\0'};

  GPIO_InitTypeDef port;



  ErrorStatus HSEStartUpStatus;



  void servo_init(void);



// ----------------------------------------------------------------------------
//simple pwm
//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 3 / 4)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int
main(int argc, char* argv[])
{


//  const char msg1[] = " forward\r\n";
 // const char msg2[] = " backwards\r\n";
//  const char msg3[] = " stop\r\n";
 // int TIM_Pulse;


 // int i;

  //Init buttons
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_StructInit(&port);
  port.GPIO_Mode = GPIO_Mode_IPU;
  port.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
  port.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &port);

  servo_init();

 // TIM_Pulse = timerPWM.TIM_Pulse;
// TIM_Pulse = timerPWM.TIM_Pulse;
//  char buffer [33];

  timer_start();

  blink_led_init();
//  Usart1Init();

  uint32_t seconds = 0;
// the old way to send info





  // Infinite loop
  while (1)
    {
      if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0) {

          TIM4->CCR1 = 1000;
          blink_led_on();
          timer_sleep(seconds == 0 ? TIMER_FREQUENCY_HZ : BLINK_ON_TICKS);




      }
      if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 0) {

          TIM4->CCR1 = 100;
          blink_led_off();
          timer_sleep(BLINK_OFF_TICKS);

      }
      if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == 0) {

          TIM4->CCR1 = 500;
          blink_led_on();
          timer_sleep(seconds == 0 ? TIMER_FREQUENCY_HZ : BLINK_ON_TICKS);
          blink_led_off();
          timer_sleep(BLINK_OFF_TICKS);

      }





      ++seconds;

      // Count seconds on the trace device.
      //trace_printf("Second %u\n", seconds);
    }
  // Infinite loop, never return.
}



void blink_led_init()
{
  // Enable GPIO Peripheral clock
  RCC_APB2PeriphClockCmd(BLINK_RCC_MASKx(BLINK_PORT_NUMBER), ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure pin in output push/pull mode
  GPIO_InitStructure.GPIO_Pin = BLINK_PIN_MASK(BLINK_PIN_NUMBER);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(BLINK_GPIOx(BLINK_PORT_NUMBER), &GPIO_InitStructure);

  // Start with led turned off
  blink_led_off();
}

#if defined(USE_HAL_DRIVER)
void HAL_IncTick(void);
#endif

// Forward declarations.

void
timer_tick (void);

// ----------------------------------------------------------------------------

volatile timer_ticks_t timer_delayCount;

// ----------------------------------------------------------------------------

void
timer_start (void)
{
  // Use SysTick as reference for the delay loops.
  SysTick_Config (SystemCoreClock / TIMER_FREQUENCY_HZ);
}

void
timer_sleep (timer_ticks_t ticks)
{
  timer_delayCount = ticks;

  // Busy wait until the SysTick decrements the counter to zero.
  while (timer_delayCount != 0u)
    ;
}

void
timer_tick (void)
{
  // Decrement to zero the counter used by the delay routine.
  if (timer_delayCount != 0u)
    {
      --timer_delayCount;
    }
}

// ----- SysTick_Handler() ----------------------------------------------------

void
SysTick_Handler (void)
{
#if defined(USE_HAL_DRIVER)
  HAL_IncTick();
#endif
  timer_tick ();
}








void servo_init(void) {

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    GPIO_StructInit(&port);
    port.GPIO_Mode = GPIO_Mode_AF_PP;
    port.GPIO_Pin = GPIO_Pin_6;   //TIM4 CHANNEL1
    port.GPIO_Speed = GPIO_Speed_2MHz;
    //port.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &port);

    TIM_TimeBaseInitTypeDef timer;
    timer.TIM_Prescaler = 71;
    timer.TIM_ClockDivision = 0;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_RepetitionCounter = 0;
    timer.TIM_Period = 20000;
    TIM_TimeBaseInit(TIM4, &timer);


    TIM_OCInitTypeDef timerPWM;

    timerPWM.TIM_Pulse = 500;

    timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
    timerPWM.TIM_OutputState = TIM_OutputState_Enable;
    timerPWM.TIM_OCPolarity = TIM_OCPolarity_High;


    TIM_OC1Init(TIM4, &timerPWM);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_Cmd(TIM4, ENABLE);



}









#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
