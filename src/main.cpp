

/*
    https://manualzz.com/doc/11587480/using-atsamd21-sercom-for-more-spi--i2c-and-serial-ports
    Using ATSAMD21 SERCOM for more SPI, I2C and Serial ports

    View detail for Atmel AT11628: SAM D21 SERCOM I2C Configuration
*/

#include "board/mpins.h"
#include "wake/wake.h"
#include "commands/commands.h"
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

  wakeInit( 0x00, 500 );           // Без адреса, время ожидания, ms
  initAdc();


}

void loop() 
{
  if( 1)      // пока так
  {   // Измерение готово
    doMeasure();   // считать, преобразовать, задать следующее и запустить
    delay(10);
    doPid();      // исполнять, если задано
  }
  
  if( Serial1.available() )     // В буфере приема есть принятые байты, не факт, что пакет полный
  {
    wakeRead();                 // Пока не принят весь пакет, время ожидания ограничено (пока 1с)
    #ifdef DEBUG_COMMANDS
      //SerialUSB.print(" -> 0x"); SerialUSB.print(Serial1.read(), HEX); // !! очистит буфер !!
    #endif
  }
  doCommand();                  // Если ненулевая, будет исполнена
  //delay(10);
}