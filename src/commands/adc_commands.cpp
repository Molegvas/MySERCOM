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

extern uint16_t         probeResolution[];
extern eAnalogReference probeMode[];
extern uint16_t probeReference[];
// comm 52
extern uint8_t prbResolution[];
extern uint8_t prbGain[];
extern uint8_t prbReference[];

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
  if( rxNbt == 4 )
  {
    uint8_t  _probe       = rxDat[0] & 0x03;      // 0-1-2-3 - U, I, D, C
    prbResolution[_probe] = rxDat[1];
    prbGain      [_probe] = rxDat[2];
    prbReference [_probe] = rxDat[3];

    //testReply( 4 );
    txReplay( 1, 0 );         // Об ошибках не сообщается - исправляются автоматически при конфигурировании
  }
  else
  {
    txReplay(1, err_tx);      // ошибка протокола
  }   
}
