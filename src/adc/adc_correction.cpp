/*
  1) Снимите любой соединительный кабель, экран или перемычку с Вашего Arduino ZERO
  2) Соедините вывод A1 с ближайшим выводом GND, используя самую короткую перемычку
  3) подсоедините штифт А2 к штифту 3,3 В с помощью максимально короткой перемычки
  4) Подключите Arduino ZERO к компьютеру с помощью USB-кабеля, подключенного к программному порту USB платы
  5) загрузите этот эскиз и оставьте плату включенной по крайней мере на одну минуту
  6) откройте последовательный монитор и нажмите кнопку сброса на Arduino ZERO
  7) в конце процедуры вы можете найти зарегистрированные данные
       - значения смещения и усиления для платы, на которой только что был выполнен эскиз
       - строка инструкции для копирования / вставки в окончательный эскиз
*/

#include <Arduino.h>
#include "atsamd21_adc.h"
#include "adc/adc.h"
#include "adc/adc_correction.h"
#include "board/mpins.h"

#include "SAMD_AnalogCorrection.h"


#define ADC_GND_PIN          A1
#define ADC_3V3_PIN          A2

constexpr uint8_t adc_read_shift        = 8;
constexpr uint8_t adc_reads_count       = 1 << adc_read_shift;

constexpr uint16_t adc_min_gain         = 0x0400;
constexpr uint16_t adc_unity_gain       = 0x0800;
constexpr uint16_t adc_max_gain         = 0x1000 - 1;
constexpr uint8_t  adc_resolution_bits  = 12;
constexpr uint16_t adc_range            = 1 << adc_resolution_bits;
constexpr uint16_t adc_top_value        = adc_range - 1;

constexpr uint8_t  max_top_value_reads  = 10;

int      offsetCorrectionValue = 0;
uint16_t gainCorrectionValue = adc_unity_gain;




//uint16_t read3V3Level();
//uint16_t readGndLevel();


uint16_t readGndLevel()
{
  uint32_t readAccumulator = 0;

  for (int i = 0; i < adc_reads_count; ++i)
  {
    readAccumulator += analogRead(ADC_GND_PIN);
  }

  uint16_t readValue = readAccumulator >> adc_read_shift;
  
  #ifdef DEBUG_COMPENSATION
    SerialUSB.print("ADC(GND) = ");
    SerialUSB.println(readValue);
  #endif
  return readValue;
}

uint16_t read3V3Level()
{
  uint32_t readAccumulator = 0;

  for (int i = 0; i < adc_reads_count; ++i)
  {
    readAccumulator += analogRead(ADC_3V3_PIN);
  }

  uint16_t readValue = readAccumulator >> adc_read_shift;
  
  if (readValue < (adc_range >> 1))
  {
    readValue += adc_range;
  }

  #ifdef DEBUG_COMPENSATION
    SerialUSB.print("ADC(3.3V) = ");
    SerialUSB.println(readValue);
  #endif
  return readValue;
}

void setCorrection()
{
//  Serial.begin(9600);

//  Serial.println("\r\nCalibrating ADC with factory values");

  analogReadResolution( adc_resolution_bits );

//  Serial.println("\r\nReading GND and 3.3V ADC levels");
//  Serial.print("   ");
  readGndLevel();
//  Serial.print("   ");
  read3V3Level();




  //- int offsetCorrectionValue = 0;
  //- uint16_t gainCorrectionValue = adc_unity_gain;




//  Serial.print("\r\nOffset correction (@gain = ");
//  Serial.print(gainCorrectionValue);
//  Serial.println(" (unity gain))");

  // Set default correction values and enable correction
  analogReadCorrection(offsetCorrectionValue, gainCorrectionValue);

  for (int offset = 0; offset < (int)(ADC_OFFSETCORR_MASK >> 1); ++offset)
  {
    analogReadCorrection(offset, gainCorrectionValue);
    
//    Serial.print("   Offset = ");
////    Serial.print(offset);
    Serial.print(", ");

    if (readGndLevel() == 0)
    {
      offsetCorrectionValue = offset;
      break;
    }
  }


// Gain correction
  Serial.println("\r\nGain correction");

  uint8_t topValueReadsCount = 0U;
  
  uint16_t minGain = 0U,
           maxGain = 0U;

  analogReadCorrection(offsetCorrectionValue, gainCorrectionValue);
  Serial.print("   Gain = ");
  Serial.print(gainCorrectionValue);
  Serial.print(", ");
  uint16_t highLevelRead = read3V3Level();
  
  if (highLevelRead < adc_top_value)
  {
    for (uint16_t gain = adc_unity_gain + 1; gain <= adc_max_gain; ++gain)
    {
      analogReadCorrection(offsetCorrectionValue, gain);

      Serial.print("   Gain = ");
      Serial.print(gain);
      Serial.print(", ");
      highLevelRead = read3V3Level();
      
      if (highLevelRead == adc_top_value)
      {
        if (minGain == 0U)
          minGain = gain;

        if (++topValueReadsCount >= max_top_value_reads)
        {
          maxGain = minGain;
          break;
        }
        
        maxGain = gain;
      }

      if (highLevelRead > adc_top_value)
        break;
    }
  }
  else if (highLevelRead >= adc_top_value)
  {
    if (highLevelRead == adc_top_value)
      maxGain = adc_unity_gain;

    for (uint16_t gain = adc_unity_gain - 1; gain >= adc_min_gain; --gain)
    {
      analogReadCorrection(offsetCorrectionValue, gain);

      Serial.print("   Gain = ");
      Serial.print(gain);
      Serial.print(", ");
      highLevelRead = read3V3Level();
      
      if (highLevelRead == adc_top_value)
      {
        if (maxGain == 0U)
          maxGain = gain;
        
        minGain = gain;
      }

      if (highLevelRead < adc_top_value)
        break;
    }
  }

  gainCorrectionValue = (minGain + maxGain) >> 1;

  analogReadCorrection(offsetCorrectionValue, gainCorrectionValue);

  // Serial.println("\r\nReadings after corrections");
  // Serial.print("   ");
  // readGndLevel();
  // Serial.print("   ");
  // read3V3Level();

  // Serial.println("\r\n==================");
  // Serial.println("\r\nCorrection values:");
  // Serial.print("   Offset = ");
  // Serial.println(offsetCorrectionValue);
  // Serial.print("   Gain = ");
  // Serial.println(gainCorrectionValue);
  // Serial.println("\r\nAdd the next line to your sketch:");
  // Serial.print("   analogReadCorrection(");
  // Serial.print(offsetCorrectionValue);
  // Serial.print(", ");
  // Serial.print(gainCorrectionValue);
  // Serial.println(");");
  // Serial.println("\r\n==================");
}
