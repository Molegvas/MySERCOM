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


// void doVoltage()
// {
//   voltage = analogReadDiffRaw( MPins::bat_plus_mux, MPins::bat_minus_mux ) * 3.3 / 512.0;    // 4, 5
// }

// void doCurrent()
// {
//   current = analogReadDiffRaw( MPins::shunt_plus_mux, MPins::shunt_minus_mux ) * 3.3 / 512.0;    // 6, 7
// }

// void doCelsius()
// {
//   celsius = analogRead( MPins::rtu_pin ) * 3.3 / 1024.0 ;
// }



void doMeasure()
{
  static int cnt = 0;

  cnt++;
  if( cnt == 4 ) { cnt = 0; } 

  switch (cnt)
  {
  case 1:
    //doVoltage();
  analogReadResolution( 12 ); // Set analog input resolution to max, 10-bits ??? 12
  analogReference( AR_DEFAULT );
  voltage = analogReadDiffRaw( MPins::bat_plus_mux, MPins::bat_minus_mux ) * 3.3 / 2048.0;    // 4, 5


    #ifdef DEBUG_ADC
      SerialUSB.print("V= "); SerialUSB.println(voltage, 2);
    #endif
    break;

  case 2:
    //doCurrent();
  analogReadResolution( 12 ); // Set analog input resolution to max, 10-bits ??? 12
  analogReference( AR_DEFAULT );
  current = analogReadDiffRaw( MPins::shunt_plus_mux, MPins::shunt_minus_mux ) * 3.3 / 2048.0;    // 6, 7


    #ifdef DEBUG_ADC
      SerialUSB.print("I= "); SerialUSB.println(current, 2);
    #endif
    break;

   case 3:
    //doCelsius();
    analogReadResolution( 10 ); // Set analog input resolution to max, 10-bits ??? 12
    analogReference( AR_INTERNAL ); // 1/1.48 VDDANA = 1/1.48* 3V3 = 2.2297
    celsius = analogRead( MPins::rtu_pin ) * 2.2297 / 1024.0 ;

    #ifdef DEBUG_ADC
      SerialUSB.print("T= "); SerialUSB.println(celsius, 2);
    #endif
    break;
 
  default:
    break;
  }
}


