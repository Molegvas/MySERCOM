/*


*/

#include <Arduino.h>
#include "adc/adc.h"
#include "board/mpins.h"

float voltage = 1.1111;
float current = 2.2222;
float celsius = 3.3333;


void initAdc()
{
  analogReadResolution( 10 ); // Set analog input resolution to max, 10-bits ??? 12
}


void doVoltage()
{
  voltage = analogReadDiffRaw( MPins::bat_plus_mux, MPins::bat_minus_mux ) * 3.3 / 512.0;    // 4, 5
}

void doCurrent()
{
  current = analogReadDiffRaw( MPins::shunt_plus_mux, MPins::shunt_minus_mux ) * 3.3 / 512.0;    // 6, 7
}

void doCelsius()
{
  celsius = analogRead( MPins::rtu_pin ) * 3.3 / 1024.0 ;
}



void doMeasure()
{
  static int cnt = 0;

  cnt++;
  if( cnt == 4 ) { cnt = 0; } 

  switch (cnt)
  {
  case 1:
    doVoltage();
    #ifdef DEBUG_ADC
      SerialUSB.print("V= "); SerialUSB.println(voltage, 2);
    #endif
    break;

  case 2:
    doCurrent();
    #ifdef DEBUG_ADC
      SerialUSB.print("I= "); SerialUSB.println(current, 2);
    #endif
    break;

   case 3:
    doCelsius();
    #ifdef DEBUG_ADC
      SerialUSB.print("T= "); SerialUSB.println(celsius, 2);
    #endif
    break;
 
  default:
    break;
  }



}
