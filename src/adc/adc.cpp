/*


*/

#include <Arduino.h>
#include "adc/adc.h"
#include "board/mpins.h"

/*
enum gains
{
  GAIN_1X = 0x00,
  GAIN_2X,
  GAIN_4X,
  GAIN_8X,
  GAIN_16X,
  GAIN_DIV2
};
*/


enum references
{
  INTREF = 0x00,    // Internal Bandgap Reference
  INTVCC0,          // 1/1.6 VDDANA
  INTVCC1,          // 1/2 VDDANA
  AREFA,            // External Reference
  AREFB,            // External Reference
};

// Данные АЦП
uint16_t adcVoltage = 0x0000;
uint16_t adcCurrent = 0x0000;
uint16_t adcReserve = 0x0000;
uint16_t adcCelsius = 0x0000;

// Пересчитанные в физические величины - резерв
float voltage = 0.0f;
float current = 0.0f;
float reserve = 0.0f;
float celsius = 0.0f;

// Настройки режимов АЦП mode: wiring_analog.c
uint16_t          probeResolution[] = { 12, 12, 10, 10 };
eAnalogReference  probeMode[]  = { AR_DEFAULT, AR_DEFAULT, AR_DEFAULT, AR_INTERNAL };

// comm 52
uint8_t prbResolution[] = { 12, 12, 10, 10 };
uint8_t prbGain[] = { 0x00 };
uint8_t prbReference[] = { 0x00 };
void analogGain52( uint8_t prb );
void analogRef52( uint8_t prb );


void initAdc()
{
  #ifdef COMM52
    analogReadResolution( 12 ); // Set analog input resolution to max
    analogGain52( 0 );
    analogRef52( 0 );
  #else
    analogReadResolution( 12 ); // Set analog input resolution to max
    analogReference( AR_DEFAULT );        // 1/2 VDDANA = 0.5* 3V3 = 1.65V
  #endif
}

void doMeasure()
{
  static int cnt = 0;

  cnt++;
  if( cnt >= 4 ) { cnt = 0; } 

  switch (cnt)
  {
  case 0:
    analogReadResolution( (int)probeResolution[cnt] );
    analogReference( probeMode[cnt] );
    adcVoltage = analogReadDiffRaw( MPins::bat_plus_mux, MPins::bat_minus_mux );    // 4, 5
    voltage = adcVoltage * 3.3 / 2048.0;

    #ifdef DEBUG_ADC
      SerialUSB.print("V= "); SerialUSB.println(voltage, 2);
    #endif
    break;

  case 1:
    analogReadResolution( (int)probeResolution[cnt] );
    analogReference( probeMode[cnt] );
    adcCurrent = analogReadDiffRaw( MPins::shunt_plus_mux, MPins::shunt_minus_mux );    // 6, 7
    current = adcCurrent * 3.3 / 2048.0;

    #ifdef DEBUG_ADC
      SerialUSB.print("I= "); SerialUSB.println(current, 2);
    #endif
    break;

   case 2:
    break;

   case 3:
    analogReadResolution( (int)probeResolution[cnt] );
    analogReference( probeMode[cnt] );
    adcCelsius = analogRead( MPins::rtu_pin );
    celsius = adcCelsius * 2.2297 / 1024.0 ;

    #ifdef DEBUG_ADC
      SerialUSB.print("T= "); SerialUSB.println(celsius, 2);
    #endif
    break;
 
  default:
    break;
  }
}

// Замена ардуиновскому mode
void analogGain52( uint8_t prb )
{
  analogGain( prbGain[prb] );
}

void analogRef52( uint8_t prb )
{
  analogRef( prbReference[prb] );
}