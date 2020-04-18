

/*
    https://manualzz.com/doc/11587480/using-atsamd21-sercom-for-more-spi--i2c-and-serial-ports
    Using ATSAMD21 SERCOM for more SPI, I2C and Serial ports

    View detail for Atmel AT11628: SAM D21 SERCOM I2C Configuration
*/

#include "board/mpins.h"
#include "wake/wake.h"
#include "commands.h"
#include "power/power_reg.h"
#include "adc/adc.h"

#include <Arduino.h>            // N. порядок не нарушать!
#include "wiring_private.h"     // N=1.




void setup() 
{
  SerialUSB.begin(115200);
  delay(1);

  // инициализация UART порта ( D0:PA11/UART-RX, D1:PA10/UART-TX )
  Serial1.begin(115200);

  wakeInit( 0x00 );           // Без адреса
  initAdc();


}

void loop() 
{
  if( 1 )
  {   // Измерение готово
      doMeasure();   // считать, преобразовать, задать следующее и запустить
      //delay(1000);
      doPid();      // исполнять, если задано
  }
  else
  {
    if (Serial1.available()) 
    {
      wakeRead();
      
      // #ifdef DEBUG_COMMANDS
      //   SerialUSB.print(" -> 0x"); SerialUSB.print(Serial1.read(), HEX);
      // #endif
    }
    delay(100);
    doCommand();       // обмен с ESP
  }
}