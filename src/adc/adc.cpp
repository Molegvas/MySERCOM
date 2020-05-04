/*


*/

#include <Arduino.h>
#include "atsamd21_adc.h"
#include "adc/adc.h"
#include "board/mpins.h"

/*
enum gains      { GAIN_1X = 0x00, GAIN_2X, GAIN_4X, GAIN_8X, GAIN_16X, GAIN_DIV2 };
enum references { INTREF  = 0x00, INTVCC0,INTVCC1, AREFA, AREFB };    
// Internal Bandgap Reference, 1/1.48 VDDANA, 1/2 VDDANA, External Reference A, External Reference B
*/

// Данные АЦП
uint16_t adcVoltage = 0x0000;
uint16_t adcCurrent = 0x0000;
uint16_t adcReserve = 0x0000;
uint16_t adcCelsius = 0x0000;

// Пересчитанные в физические величины - mV, mA, mC
int16_t voltage = 0x0064;     //  0.10V
int16_t current = 0xfc17;     // -1.00A
int16_t reserve = 0x0000;
int16_t celsius = 0x61a8;     // +25.00

// Настройки режимов АЦП mode: wiring_analog.c
uint16_t          probeResolution[] = { 12, 12, 10, 10 };
eAnalogReference  probeMode[]  = { AR_DEFAULT, AR_DEFAULT, AR_DEFAULT, AR_INTERNAL };

// comm 52
uint8_t  prbResolution[] = {   14,   14,   10,   10 };
uint8_t  prbGain[]       = { 0x00, 0x02, 0x00, 0x00 }; // 05 - DIV2  00 - X1 
uint8_t  prbReference[]  = { 0x00, 0x00, 0x01, 0x01 }; // 02 - VCC1  01 - VCC0
uint16_t prbOffset[]     = { 0x0000, 0x0000, 0x0000, 0x0000 };  // Приборное смещение
                         // 39k/2k2
uint16_t prbFactor[]     = { 0x12ba, 0x0100, 0x0000, 0x0100 };  // Коэффициент преобразования

// comm 53
uint8_t adcBits   [] = { 0x01, 0x01, 0x01, 0x01 };  // 0x00(12), 0x01(16), 0x02(10), 0x03(8)
uint8_t adcSamples[] = { 0x03, 0x03, 0x04, 0x04 };  // 0x00 ... 0x0a (1, 2, 4, 8 ... 1024)
uint8_t adcDivider[] = { 0x04, 0x04, 0x04, 0x04 };  // 0x00 ... 0x07 (2^0, 2^1, 2^2 ... 2^7)

void analogGain52( uint8_t prb );
void analogRef52( uint8_t prb );

void initAdc52(uint8_t n)
{
  analogGain( prbGain[n] );
  analogRef( prbReference[n] ); 
  //analogReadExtended( prbResolution[n] ); // Расширенный функционал analogReadResolution()

  analogReadConfig( adcBits[n], adcSamples[n], adcDivider[n] ); 


  // SerialUSB.print(n); SerialUSB.print("->");
  // SerialUSB.print("n->"); SerialUSB.print(prbResolution[n]);
  // SerialUSB.print("->"); SerialUSB.print(prbGain[n]);
  // SerialUSB.print("->"); SerialUSB.print(prbReference[n]);
} 

// Преобразование данных АЦП в милливольты


int16_t averaging(uint16_t adc, uint8_t cnt)
{
    switch (prbResolution[cnt])
  {
  case 0x0c: return adc >> 2; break;
  case 0x0d: return adc >> 3; break;
  case 0x0e: return adc >> 4; break;
  case 0x0f: return adc >> 5; break;
  default:   return adc;      break;
  }
}


int16_t convertToValue(uint16_t adc, bool diff)
{
  uint16_t maxVal = 4096;
  uint16_t ref = 3300;

  if(diff) maxVal /= 2;               // Не поддерживается ардуиной дифф. режим

  //SerialUSB.print("->"); SerialUSB.println(adc, HEX);


  return adc * ref / maxVal;
}


// Усреднения автоматически для 13...16 бит.
void doMeasure()
{
  static int cnt = 0;

  cnt++;
  if( cnt >= 4 ) { cnt = 0; } 

  switch (cnt)
  {
  case 0:
    initAdc52(cnt);               // настройка усиления и опоры
      analogReadConfig( adcBits[cnt], adcSamples[cnt], adcDivider[cnt] ); //настройка АЦП

    adcVoltage = analogDifferentialRaw( MPins::bat_plus_mux, MPins::bat_minus_mux );    // 4, 5
    //voltage = adcVoltage * 3.3 / 2048.0;
    //voltage = (int16_t)( adcVoltage * 1000 / 4096 ) / 2;
    //voltage = convertToValue(adcVoltage, true);
  voltage = adcVoltage;   //averaging(adcVoltage, cnt);

    #ifdef DEBUG_ADC
      SerialUSB.print("V= "); SerialUSB.println(voltage, 2);
    #endif
    break;

  case 1:
    initAdc52(cnt);
      analogReadConfig( adcBits[cnt], adcSamples[cnt], adcDivider[cnt] ); //настройка АЦП

    adcCurrent = analogDifferentialRaw( MPins::shunt_plus_mux, MPins::shunt_minus_mux );    // 6, 7
    //current = adcCurrent * 3.3 / 2048.0;
    current = convertToValue(adcCurrent, true);
    
    #ifdef DEBUG_ADC
      SerialUSB.print("I= "); SerialUSB.println(current, 2);
    #endif
    break;

  case 2:
    break;

  case 3:
    initAdc52(cnt);
      analogReadConfig( adcBits[cnt], adcSamples[cnt], adcDivider[cnt] ); //настройка АЦП

    adcCelsius = analogRead( MPins::rtu_pin );
    //celsius = adcCelsius * 2.2297 / 1024.0 ;
    celsius = convertToValue(adcCelsius, false);
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