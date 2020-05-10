/*


*/

#include "board/mpins.h"
#include "wake/wake.h"
#include "adc_commands.h"
#include "SAMD_AnalogCorrection.h"
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

// comm 51 параметры измерения
extern uint8_t  prbReference[];     // опорное напряжение
extern uint8_t  prbGain[];          // усиление
extern int16_t  prbOffset[];        // приборное смещение
extern uint16_t prbDivider[];       // коэффициент преобразования в физическую величину

// comm 52 параметры преобразования
extern uint8_t adcBits   [];        // 0x00(12), 0x01(16), 0x02(10), 0x03(8)
extern uint8_t adcSamples[];        // 0x00 ... 0x0a (1, 2, 4, 8 ... 1024)
extern uint8_t adcDivider[];        // 0x00 ... 0x07 (2^0, 2^1, 2^2 ... 2^7)
extern uint8_t adcRefComp[];        // 0x00, 0x01  Это резервная позиция

// ?? comm 53 analogReferenceCompensation on/off
extern uint8_t refComp;

//
extern int      offsetCorrectionValue;
extern uint16_t gainCorrectionValue;

// отправить данные измерений
void doReadProbes()
{
  if( rxNbt == 0 )
  {
    txDat[1]  = ( adcVoltage >> 8) & 0xFF; // Hi
    txDat[2]  =   adcVoltage & 0xFF;       // Lo
    txDat[3]  = ( adcCurrent >> 8) & 0xFF; // Hi
    txDat[4]  =   adcCurrent & 0xFF;       // Lo
    txDat[5]  = ( adcReserve >> 8) & 0xFF; // Hi
    txDat[6]  =   adcReserve & 0xFF;       // Lo
    txDat[7]  = ( adcCelsius >> 8) & 0xFF; // Hi
    txDat[8]  =   adcCelsius & 0xFF;       // Lo
    txDat[9]  =   0x77;      // mcr1 - информация о состоянии 
    txDat[10] =   0x88;      // mcr2 - ... или управление 

    txNbt = 11;
    txReplay( txNbt, 0 );   //txDat[0] ); // в нулевом - сообщение об ошибках, если не иное
  }
  else
  {
    txReplay(1, err_tx);    // ошибка протокола (пакет не полный)
  }
}

// Выбор параметров для заданного аналогового датчика
void doAdcConfig()
{
  if( rxNbt == 7 )
  {
    uint8_t prb = rxDat[0] & 0x03;  // индекс датчика (0...3: U, I, D, C
    prbReference[prb]   = rxDat[1]; // выбор опорного напряжения(0...4: 1V, 1/1.48VDDA, 1/2VDDA, ExtA, ExtB)
    prbGain[prb]        = rxDat[2]; // выбор усиления (0...5: X1, X2, X4, X8, X16, DIV2)
    prbOffset[prb]      = get16(3); // приборное смещение
    prbDivider[prb]     = get16(5); // коэффициент преобразования в физическую величину

//    testReply( 7 );
    txReplay( 1, 0 );                   // Об ошибках параметров не сообщается
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
    uint8_t    _probe  = rxDat[0] & 0x03;  // 0-1-2-3 - U, I, D, C
    adcBits   [_probe] = rxDat[1];         // 0x00(12), 0x01(16), 0x02(10), 0x03(8)
    adcSamples[_probe] = rxDat[2];         // 0x00 ... 0x0a (1, 2, 4, 8 ... 1024)
    adcDivider[_probe] = rxDat[3];         // 0x00 ... 0x07 (2^0, 2^1, 2^2 ... 2^7)
//    adcRefComp[_probe] = rxDat[4];         // 0x00, 0x01 
    //testReply( 5 );
    txReplay( 1, 0 );         // Об ошибках не сообщается - исправляются автоматически при конфигурировании
  }
  else
  {
    txReplay(1, err_tx);      // ошибка протокола
  }   
}

// Arduino has it disabled by default.
// Устанавливает параметры коррекции преобразований по смещению,
// задаются и автоматически включается коррекция (comm 53)
void doAdcRefCompensation()
{
  if( rxNbt == 4 )
  {
    int offsetCorrectionValue = 0x00000000;
    offsetCorrectionValue  = ( rxDat[0] & 0xff ) << 24;
    offsetCorrectionValue |= ( rxDat[1] & 0xff ) << 16;
    offsetCorrectionValue |= ( rxDat[2] & 0xff ) <<  8;
    offsetCorrectionValue |= ( rxDat[3] & 0xff );
    gainCorrectionValue    = 0x800;    // adc_unity_gain = 0x0800;    
    // txDat[1] = rxDat[0];
    // txDat[2] = rxDat[1];
    // txDat[3] = rxDat[2];
    // txDat[4] = rxDat[3];
    
    //SerialUSB.print("->0x"); SerialUSB.println(offsetCorrectionValue, HEX);

    txReplay( 1, 0 );         // err txDat[0]
  }
  else
  {
    txReplay(1, err_tx);      // ошибка протокола
  }     
}

// comm 0x58
void doOffsetCompensation()
{
  if( rxNbt == 5 )
  {
    // уточнить
    refComp = rxDat[0] & 0x01;       // 0/1 disable/enable

    int _offsetCorrectionValue = 0x00000000;
    _offsetCorrectionValue  = ( rxDat[0] & 0xff ) << 24;
    _offsetCorrectionValue |= ( rxDat[1] & 0xff ) << 16;
    _offsetCorrectionValue |= ( rxDat[2] & 0xff ) <<  8;
    _offsetCorrectionValue |= ( rxDat[3] & 0xff );  
    uint16_t _gainCorrectionValue    = 0x800;    // adc_unity_gain = 0x0800;
       // Set default correction values and enable correction
  analogReadCorrection( _offsetCorrectionValue, _gainCorrectionValue);

    txDat[1] = ( adcVoltage >> 8) & 0xFF; // Hi
    txDat[2] =   adcVoltage & 0xFF;       // Lo
    txDat[3] = ( adcCurrent >> 8) & 0xFF; // Hi
    txDat[4] =   adcCurrent & 0xFF;       // Lo
    txDat[5] = ( adcReserve >> 8) & 0xFF; // Hi
    txDat[6] =   adcReserve & 0xFF;       // Lo
    txDat[7] = ( adcCelsius >> 8) & 0xFF; // Hi
    txDat[8] =   adcCelsius & 0xFF;       // Lo

    txNbt = 9;
    txReplay( txNbt, 0 );         // 
  }
  else
  {
    txReplay(1, err_tx);      // ошибка протокола
  }     
}

// comm 0x59 - зарезервирована
void doOffsetGainCompensation()
{
  if( rxNbt == 1 )
  {
    // уточнить
    refComp  = rxDat[0] & 0x01;       // 0/1 disable/enable

    txDat[1] = ( offsetCorrectionValue >> 8) & 0xFF; // Hi
    txDat[2] =   offsetCorrectionValue & 0xFF;       // Lo
    txDat[3] = ( gainCorrectionValue   >> 8) & 0xFF; // Hi
    txDat[4] =   gainCorrectionValue   & 0xFF;       // Lo    

    txReplay( 5, 0 );         // 
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

// Команды ... резервные
void doAdcBat()
{
    if( rxNbt == 4 )
  {
    uint8_t err   = 0x00;
    // ...
    txReplay( 1, err );
  }
  else
  {
    txReplay(1, err_tx);
  }
}

void doAdcShunt()
{
    if( rxNbt == 4 )
  {
    uint8_t err   = 0x00;
    // ...
    txReplay( 1, err );
  }
  else
  {
    txReplay(1, err_tx);
  }
}

void doAdcRtu()
{
    if( rxNbt == 4 )
  {
    uint8_t err   = 0x00;
    // ...
    txReplay( 1, err );
  }
  else
  {
    txReplay(1, err_tx);
  }
}
