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

// Данные аппаратной поддержки
constexpr uint16_t u_divider = (uint16_t)((39000 + 2200) * 0x100 / 2200); 

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

// comm 51 параметры измерения
uint8_t  prbReference[] = { 0x00, 0x00, 0x01, 0x01 };             // 02 - VCC1  01 - VCC0;
uint8_t  prbGain[]      = { 0x00, 0x02, 0x00, 0x00 };             // 05 - DIV2  00 - X1 ;
int16_t  prbOffset[]    = { 0x0000, 0x0000, 0x0000, 0x0000 };     // Приборные смещения
uint16_t prbDivider[]   = { u_divider, 0x0100, 0x0000, 0x0100 };  // Коэффициенты преобразования

// comm 52
uint8_t adcBits   [] = { 0x01, 0x01, 0x00, 0x01 };  // 0x00(12), 0x01(16), 0x02(10), 0x03(8)
uint8_t adcSamples[] = { 0x03, 0x03, 0x00, 0x04 };  // 0x00 ... 0x0a (1, 2, 4, 8 ... 1024)
uint8_t adcDivider[] = { 0x04, 0x04, 0x00, 0x04 };  // 0x00 ... 0x07 (2^0, 2^1, 2^2 ... 2^7)
uint8_t adcRefComp[] = { 0x01, 0x01, 0x00, 0x01 };  // 0x00, 0x01  - Резервная позиция
// adcRefComp = 0/1 отключить/включить коррекцию смещения и усиления,
// результат АЦП будет автоматически откорректирован по формуле
// Result = ( Conversion value - OFFSETCORR ) * GAINCORR
uint8_t refComp = 0x01; 



// comm 53
// offsetCorr = 

// gainCorr   = 



void initAdc(uint8_t n)
{
  // параметры АЦП
  analogReadConfig( adcBits[n], adcSamples[n], adcDivider[n] ); 
  //analogReferenceCompensation( adcRefComp[n] );   // автокомпенсация начального смещения (выкл или вкл)
  analogReferenceCompensation( refComp );   // автокомпенсация начального смещения (выкл или вкл)

  // параметры датчика (формально это тоже параметры АЦП)
  //analogGain( prbGain[n] );
  //analogRef( prbReference[n] ); 
} 

// Преобразование данных АЦП в милливольты


// int16_t averaging(uint16_t adc, uint8_t prb)
// {
//     switch (prbResolution[prb])
//   {
//   case 0x0c: return adc >> 2; break;
//   case 0x0d: return adc >> 3; break;
//   case 0x0e: return adc >> 4; break;
//   case 0x0f: return adc >> 5; break;
//   default:   return adc;      break;
//   }
// }


int16_t convertToValue(uint16_t adc, bool diff)
{
  uint16_t maxVal = 4096;
  uint16_t ref = 3300;

  if(diff) maxVal /= 2;               // Не поддерживается ардуиной дифф. режим

  //SerialUSB.print("->"); SerialUSB.println(adc, HEX);


  return adc * ref / maxVal;
}


// автомат последовательного опроса датчиков
void doMeasure()
{
  static int prb = 0;

  prb++;
  if( prb >= 4 ) { prb = 0; } 

  switch (prb)
  {
  case 0:
    initAdc(prb);               // настройка усиления и опоры
      analogReadConfig( adcBits[prb], adcSamples[prb], adcDivider[prb] ); //настройка АЦП

    adcVoltage = analogDifferentialRaw( MPins::bat_plus_mux, MPins::bat_minus_mux );    // 4, 5
    //voltage = adcVoltage * 3.3 / 2048.0;
    //voltage = (int16_t)( adcVoltage * 1000 / 4096 ) / 2;
    //voltage = convertToValue(adcVoltage, true);
  voltage = adcVoltage;   //averaging(adcVoltage, prb);

    #ifdef DEBUG_ADC
      SerialUSB.print("V= "); SerialUSB.println(voltage, 2);
    #endif
    break;

  case 1:
    initAdc(prb);
      analogReadConfig( adcBits[prb], adcSamples[prb], adcDivider[prb] ); //настройка АЦП

    adcCurrent = analogDifferentialRaw( MPins::shunt_plus_mux, MPins::shunt_minus_mux );    // 6, 7
    //current = adcCurrent * 3.3 / 2048.0;
    current = convertToValue(adcCurrent, true);
    
    #ifdef DEBUG_ADC
      SerialUSB.print("I= "); SerialUSB.println(current, 2);
    #endif
    break;

  case 2:
  //SerialUSB.println(u_divider, HEX);

    break;

  case 3:
    initAdc(prb);
      analogReadConfig( adcBits[prb], adcSamples[prb], adcDivider[prb] ); //настройка АЦП

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

