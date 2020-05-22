/*
  Реализован циклический автомат опроса датчиков. Период опроса задается параметром hz 
  в настройках пид-регулятора. По умолчанию hz = 10.0 Hz, что соответствует 0,1с.
  Датчиков в системе 4:
  * измеритель напряжения - дифференциальный;
  * измеритель тока - дифференциальный;
  * пид-регулятор (притворился датчиком);
  * датчик температуры.
  Параметры датчиков - разрядность, опорное напряжение, усиление, накопление и делитель для
  автоматического вычисления среднего, задаются соответствующими командами, как и приборные 
  смещение и фактор пересчета в физические величины.
*/

#include <Arduino.h>
#include "atsamd21_adc.h"
#include "adc/adc.h"
#include "power/power_reg.h"
#include "board/mpins.h"
#include "stdint.h"

/*
enum gains      { GAIN_1X = 0x00, GAIN_2X, GAIN_4X, GAIN_8X, GAIN_16X, GAIN_DIV2 };
enum references { INTREF  = 0x00, INTVCC0,INTVCC1, AREFA, AREFB };    
// Internal Bandgap Reference, 1/1.48 VDDANA, 1/2 VDDANA, External Reference A, External Reference B
*/

constexpr uint8_t numDat = 4;   // Количество аналоговых датчиков в системе 

  // Тайминг опроса датчиков
//extern float hz;                // 10Hz по умолчанию
//uint32_t period = 25;           // 25ms for 10Hz Период опроса датчика 25*4=100мс (10Гц)
uint16_t constexpr period = 25;   // 25ms for 10Hz Период опроса датчика 25*4=100мс (10Гц)
uint32_t ts;                    // таймер

  // Данные аппаратной поддержки
constexpr uint16_t  u_divide = (uint16_t)((39000 + 2200) * 0x100 / 2200);   // Заводской (R+R)/R << 16
uint16_t u_factor = u_divide;     // Корректированный, по умолчанию - заводской

  // Данные АЦП
uint16_t adcVoltage = 0x0000;
uint16_t adcCurrent = 0x0000;
uint16_t adcReserve = 0x0000;
uint16_t adcCelsius = 0x0000;

  // Пересчитанные в физические величины - mV, mA, mC
int16_t voltage     = 0x0064;     //  0.10V
int16_t current     = 0xfc17;     // -1.00A
int16_t reserve     = 0x0000;
float   celsius     = 25.0f;      // +25.00
int16_t celsiusHex  = 0x09c4;     // +25.00


// comm 51 параметры измерения
uint8_t  prbReference[] = { 0x00, 0x00, 0x01, 0x00 };             // 02 - VCC1  01 - VCC0;
uint8_t  prbGain[]      = { 0x00, 0x02, 0x00, 0x00 };             // 05 - DIV2  00 - X1 ;
int16_t  prbOffset[]    = { 0x0000, 0x0000, 0x0000, 0x0000 };     // Приборные смещения
uint16_t prbFactor[]    = { u_factor, 0x03ad, 0x0000, 0x0000 };   // Коэффициенты преобразования

// comm 52
uint8_t adcBits   [] = { 0x01, 0x01, 0x00, 0x01 };  // 0x00(12), 0x01(16), 0x02(10), 0x03(8)
uint8_t adcSamples[] = { 0x03, 0x03, 0x00, 0x03 };  // 0x00 ... 0x0a (1, 2, 4, 8 ... 1024)
uint8_t adcDivider[] = { 0x04, 0x00, 0x00, 0x04 };  // 0x00 ... 0x07 (2^0, 2^1, 2^2 ... 2^7)
uint8_t adcRefComp[] = { 0x01, 0x01, 0x00, 0x00 };  // 0x00, 0x01  - Резервная позиция
// adcRefComp = 0/1 отключить/включить коррекцию смещения и усиления,
// результат АЦП будет автоматически откорректирован по формуле
// Result = ( Conversion value - OFFSETCORR ) * GAINCORR
uint8_t refComp = 0x01; 

// Опорные напряжения (в милливольтах), выбор по prbReference[]
uint16_t reference[] = { 1100u, 2230u, 1650u, 2540u, 2540u };

// Усиление как коэффициент, выбор по выбор по prbGain[]
uint16_t gain[] = { 1000u, 2000u, 4000u, 8000u, 16000u, 500u };

// Максимальное значение ADC после автоматической обработки
constexpr uint16_t maxVal = 4096;

// comm 53
// offsetCorr = 

// gainCorr   = 

// Параметры термистора MF52B 3950 NTC 10 kOm
constexpr float reference_resistance = 10000.0f;    // 10kOm
constexpr float nominal_resistance   = 10000.0f;    // 10kOm
constexpr float nominal_temperature  =    25.0f;
constexpr float b_value              =  3950.0f;

// Они же, по факту откорректированные. По умолчанию - заводские
uint16_t refRes = reference_resistance;   // 10k  последовательный
uint16_t nomRes = nominal_resistance;     // 10k  номинал


float readSteinhart( const int adc );

  // Установка параметров преобразования датчика:
  // разрядность, усиление, опора, накопление, делитель, автокомпенсация
void initAdc(uint8_t n)
{
  analogGain( prbGain[n] );
  analogRef( prbReference[n] ); 
  analogReadConfig( adcBits[n], adcSamples[n], adcDivider[n] );
  analogReferenceCompensation( refComp );   // автокомпенсация начального смещения (выкл или вкл)
} 

// Преобразование данных АЦП в милливольты


int16_t convMv(uint16_t adc, uint8_t prb)
{
  //uint16_t maxVal = 4096;
  uint16_t ref = reference[ prb ];
  //return (uint16_t)(adc * ref / maxVal);  // * 16;
  return (adc / maxVal) * ref;  // * 16;
}


// int16_t convertToValue(uint16_t adc, bool diff)
// {
//   uint16_t maxVal = 4096;
//   uint16_t ref = 3300;

//   if(diff) maxVal /= 2;               // Не поддерживается ардуиной дифф. режим

//   //SerialUSB.print("->"); SerialUSB.println(adc, HEX);


//   return adc * ref / maxVal;
// }


// Циклический автомат последовательного опроса датчиков
void doMeasure()
{
  //period = (uint32_t)( (1000.0 / hz) / numDat );    // Может быть изменен в процессе?

  if( millis() - ts >= period )
  { 
    ts += period; 
    static int prb = 0;

    prb++;
    if( prb >= numDat ) { prb = 0; } 

    switch (prb)
    {
    case 0:
      initAdc(prb);               // настройка усиления и опоры
      adcVoltage = analogDifferentialRaw( MPins::bat_plus_mux, MPins::bat_minus_mux );    // 4, 5
      //voltage = adcVoltage * 3.3 / 2048.0;
      //voltage = (int16_t)( adcVoltage * 1000 / 4096 ) / 2;
      //voltage = convertToValue(adcVoltage, true);
    //voltage = adcVoltage;   //averaging(adcVoltage, prb);
    //voltage = (convMv( adcVoltage, prb ) * prbFactor  [prb]) / 0x100 - prbOffset[prb];
    voltage = (( adcVoltage * reference[prb] / maxVal ) * prbFactor[prb]) / 0x100 - prbOffset[prb];

      #ifdef DEBUG_ADC
        //SerialUSB.print("V= "); SerialUSB.println(voltage, 2);
        SerialUSB.print("V= "); SerialUSB.println(adcVoltage, HEX);
      #endif
      break;

    case 1:
      initAdc(prb);
      adcCurrent = analogDifferentialRaw( MPins::shunt_plus_mux, MPins::shunt_minus_mux );    // 6, 7
      //current = adcCurrent * 3.3 / 2048.0;
      //current = convertToValue(adcCurrent, true);
      current = (convMv( adcCurrent, prb ) * prbFactor  [prb] ) / 0x100 ;

      
      #ifdef DEBUG_ADC
        SerialUSB.print("I= "); SerialUSB.println(current, 2);
      #endif
      break;

    case 2:
      doPid();      // исполнять, если задано

        //SerialUSB.println(millis()); 
      break;

    case 3:
      initAdc(prb);
      //analogReadConfig( adcBits[prb], adcSamples[prb], adcDivider[prb] ); //настройка АЦП

      adcCelsius = analogRead( MPins::rtu_pin );  // Код АЦП (Может быть сдвинут при накоплении)
      celsius = readSteinhart( adcCelsius );      // float 
      celsiusHex = (int16_t)( celsius * 100.0 );  // Преобразование для передачи в int16_t
      #ifdef DEBUG_ADC
        SerialUSB.print("T= "); SerialUSB.println(celsius, 2);
      #endif
      break;
  
    default:
      break;
    }
  }
}

float readSteinhart( const int adc )
{
// https://neyasyt.ru/uploads/files/termistor-NTC-10-K-MF52.pdf
  float steinhart;
  float tr = 4095.0 / adc - 1.0;
  
  tr = (float)refRes / tr;
  steinhart = tr / (float)nomRes;                       // (R/Ro)
  steinhart = log(steinhart);                           // ln(R/Ro)
  steinhart /= b_value;                                 // 1/B * ln(R/Ro)
  steinhart += 1.0f / (nominal_temperature + 273.15f);  // + (1/To)
  steinhart = 1.0f / steinhart;                         // Invert
  steinhart -= 273.15f;
  if ( steinhart == -273.15f ) steinhart = 120.0f;      // При отсутствии датчика
  return ( steinhart > 120.0f ) ? 120.0f : steinhart;
}

