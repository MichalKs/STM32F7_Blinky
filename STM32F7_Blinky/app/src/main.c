
#include <string.h>
#include <stm32f7xx_hal.h>
#include <stm32746g_discovery_lcd.h>
#include <stm32746g_discovery_ts.h>
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
//  LED_Toggle(_LED0);
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

  // Set LCD Foreground Layer
  BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);

  // Clear the LCD
//  BSP_LCD_SetBackColor(LCD_COLOR_DARKMAGENTA);
  BSP_LCD_Clear(LCD_COLOR_DARKMAGENTA);

  /* Set the LCD Text Color */
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

  /* Display LCD messages */
  BSP_LCD_DisplayStringAtLine(0, (uint8_t *)"STM32F7 Discovery");
  BSP_LCD_DisplayStringAtLine(1, (uint8_t *)"Hello World!!!");

  BSP_LCD_DrawLine(50, 50, 150, 150);
  BSP_LCD_FillRect(100, 100, 100, 100);
  BSP_LCD_FillCircle(300, 50, 40);

  BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

  TS_StateTypeDef tsState;

  //*******************LCD test END

  uint8_t buf[255]; // buffer for receiving commands from PC
  uint8_t len;      // length of command

  while (1) {

    // check for new frames from PC
    if (!COMM_GetFrame(buf, &len)) {
      println("Got frame of length %d: %s", (int)len, (char*)buf);

      // control LED0 from terminal
      if (!strcmp((char*)buf, ":LED0 ON")) {
        LED_ChangeState(_LED0, LED_ON);
      }
      if (!strcmp((char*)buf, ":LED0 OFF")) {
        LED_ChangeState(_LED0, LED_OFF);
      }
    }

    // check for touch screen events
    BSP_TS_GetState(&tsState);
    if (tsState.touchDetected == 1) {
      LED_Toggle(_LED0);
    }

    TIMER_SoftTimersUpdate(); // run timers
  }
}
