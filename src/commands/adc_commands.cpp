/*


*/

#include "board/mpins.h"
#include "wake/wake.h"
#include "adc_commands.h"
#include "power/power_reg.h"
#include "stdint.h"
#include <Arduino.h>

    // Переменные - уточнить типы  
extern char     rxNbt;          //+ принятое количество байт в пакете
extern char     rxDat[frame];   //+ массив принятых данных
extern uint8_t  command;        // код команды на выполнение

//extern char     TxCmd;          // команда, передаваемая в пакете
extern char     txNbt;          // количество байт данных в пакете
extern char     txDat[frame];   //+ массив данных для передачи

extern uint16_t adcVoltage;  // Данные АЦП
extern uint16_t adcCurrent;  //
extern uint16_t adcReserve;  //
extern uint16_t adcCelsius;  //

// Пересчитанные в физические величины - mV, mA, mC

extern int16_t voltage;
extern int16_t current;
extern int16_t reserve;
extern int16_t celsius;


extern uint16_t         probeResolution[];
extern eAnalogReference probeMode[];
extern uint16_t         probeReference[];
// comm 52
extern uint8_t prbResolution[];
extern uint8_t prbGain[];
extern uint8_t prbReference[];
extern uint16_t prbOffset[];  // Приборное смещение
extern uint16_t prbFactor[];  // Коэффициент преобразования
// comm 53
extern uint8_t adcBits   [];         // 0x00(12), 0x01(16), 0x02(10), 0x03(8)
extern uint8_t adcSamples[];         // 0x00 ... 0x0a (1, 2, 4, 8 ... 1024)
extern uint8_t adcDivider[];         // 0x00 ... 0x07 (2^0, 2^1, 2^2 ... 2^7)

// отправить данные измерений
void doReadProbes()
{
  if( rxNbt == 0 )
  {
    txDat[0] = ( adcVoltage >> 8) & 0xFF; // Hi
    txDat[1] =   adcVoltage & 0xFF;       // Lo
    txDat[2] = ( adcCurrent >> 8) & 0xFF; // Hi
    txDat[3] =   adcCurrent & 0xFF;       // Lo
    txDat[4] = ( adcReserve >> 8) & 0xFF; // Hi
    txDat[5] =   adcReserve & 0xFF;       // Lo
    txDat[6] = ( adcCelsius >> 8) & 0xFF; // Hi
    txDat[7] =   adcCelsius & 0xFF;       // Lo
    txDat[8] =   0x77;      // mcr1
    txDat[9] =   0x88;      // mcr2

    txNbt = 10;
    txReplay( txNbt, txDat[0] );
    #ifdef DEBUG_COMMANDS
      Serial.println("измерения");
    #endif
  }
  else
  {
    txReplay(1, err_tx);
  }
}

// Выбор для заданного аналогового датчика разрешения и опоры.
// Ошибки ввода исправляются автоматически без сообщения
void doAdcConfig()
{
  if( rxNbt == 5 )
  {
    uint8_t  _probe         = rxDat[0] & 0x03;              // 0-1-2-3 - U, I, D, C
    probeResolution[_probe] = get16(1);
    probeMode [_probe] = (eAnalogReference)get16(3);   // mode

//    testReply( 5 );
    txReplay( 1, 0 );                   // Об ошибках не сообщается - исправляются автоматически
  }
  else
  {
    txReplay(1, err_tx);                // ошибка протокола
  }   
}

void doAdcConfig52()
{
  if( rxNbt == 8 )
  {
    uint8_t  _probe       = rxDat[0] & 0x03;  // 0-1-2-3 - U, I, D, C
    prbResolution[_probe] = rxDat[1];         // bits   (8...12, 13, 14, 15, 16)
    prbGain      [_probe] = rxDat[2];         // gain   (0...5: X1, X2, X4, X8, X16, DIV2)
    prbReference [_probe] = rxDat[3];         // reference (0...4: 1V, 1/1.48VDDA, 1/2VDDA, ExtA, ExtB)
    prbOffset    [_probe] = get16(4);         // offset (+/- mV)
    prbFactor    [_probe] = get16(6);         // factor (R/R << 8)

    //testReply( 8 );
    txReplay( 1, 0 );         // Об ошибках не сообщается - исправляются автоматически при конфигурировании
  }
  else
  {
    txReplay(1, err_tx);      // ошибка протокола
  }   
}

void doAdcConfig53()
{
  if( rxNbt == 4 )
  {
    uint8_t  _probe       = rxDat[0] & 0x03;  // 0-1-2-3 - U, I, D, C
    adcBits   [_probe] = rxDat[1];         // 0x00(12), 0x01(16), 0x02(10), 0x03(8)
    adcSamples[_probe] = rxDat[2];         // 0x00 ... 0x0a (1, 2, 4, 8 ... 1024)
    adcDivider[_probe] = rxDat[3];         // 0x00 ... 0x07 (2^0, 2^1, 2^2 ... 2^7)

    //testReply( 4 );
    txReplay( 1, 0 );         // Об ошибках не сообщается - исправляются автоматически при конфигурировании
  }
  else
  {
    txReplay(1, err_tx);      // ошибка протокола
  }   
}


// Чтение данных в физических величинах ( милливольты, миллиамперы, миллиградусы )
void doReadValues()
{
  if( rxNbt == 0 )
  {
    txDat[0] = ( voltage >> 8) & 0xFF; // Hi
    txDat[1] =   voltage & 0xFF;       // Lo
    txDat[2] = ( current >> 8) & 0xFF; // Hi
    txDat[3] =   current & 0xFF;       // Lo
    txDat[4] = ( reserve >> 8) & 0xFF; // Hi
    txDat[5] =   reserve & 0xFF;       // Lo
    txDat[6] = ( celsius >> 8) & 0xFF; // Hi
    txDat[7] =   celsius & 0xFF;       // Lo
    txDat[8] =   0x77;      // mcr1
    txDat[9] =   0x88;      // mcr2

    txNbt = 10;
    txReplay( txNbt, txDat[0] );
    #ifdef DEBUG_COMMANDS
      Serial.println("измерения");
    #endif
  }
  else
  {
    txReplay(1, err_tx);
  }
}

