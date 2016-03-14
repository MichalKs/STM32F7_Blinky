
#include <stm32f7xx_hal.h>
#include <stm32746g_discovery_lcd.h>
#include "led.h"
#include "timers.h"
#include "system.h"
#include "comm.h"

#define DEBUG

#ifdef DEBUG
#define print(str, args...) printf(""str"%s",##args,"")
#define println(str, args...) printf("MAIN--> "str"%s",##args,"\r\n")
#else
#define print(str, args...) (void)0
#define println(str, args...) (void)0
#endif

/**
  * @brief  LCD FB_StartAddress
  * LCD Frame buffer start address : starts at beginning of SDRAM
  */
#define LCD_FRAME_BUFFER          SDRAM_DEVICE_ADDR

void softTimerCallback(void) {
  LED_Toggle(_LED0);
  println("Test string sent from STM32F7!!!"); // Print test string
}

/**
 * @brief Main function
 * @return
 */
int main(void) {

  SYSTEM_Init(); // Initialize STM32F7 and HAL (SYSTICK)
  COMM_Init(115200);
  println("Starting program"); // Print a string to terminal

  // Add a soft timer with callback running every 1000ms
  int8_t timerID = TIMER_AddSoftTimer(1000, softTimerCallback);
  TIMER_StartSoftTimer(timerID); // start the timer

  LED_Init(_LED0); // Add an LED

  //*******************LCD test
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER, LCD_FRAME_BUFFER);

  /* Set LCD Foreground Layer  */
  BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);

  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

  /* Clear the LCD */
  BSP_LCD_SetBackColor(LCD_COLOR_DARKMAGENTA);
  BSP_LCD_Clear(LCD_COLOR_DARKMAGENTA);

  /* Set the LCD Text Color */
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

  /* Display LCD messages */
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"STM32F7 Discovery", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, 35, (uint8_t *)"Hello World!!!", CENTER_MODE);
  //*******************LCD test END

  while (1) {
    TIMER_SoftTimersUpdate(); // run timers
  }
}
