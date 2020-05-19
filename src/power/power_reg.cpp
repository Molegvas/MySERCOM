/*
  https://github.com/mike-matera/FastPID/tree/master/examples/VoltageRegulator
  FastPID: A fast 32-bit fixed-point PID controller for Arduino

*/

#include <Arduino.h>
#include "board/mpins.h"
#include "wake/wake.h"
#include "power/power_reg.h"
#include "commands/commands.h"
#include <FastPID.h>
#include "stdint.h"

    // Переменные - уточнить типы  
extern char     rxNbt;          //+ принятое количество байт в пакете
extern char     rxDat[frame];   //+ массив принятых данных
extern uint8_t  command;        // код команды на выполнение

//extern char     TxCmd;          // команда, передаваемая в пакете
extern char     txNbt;          // количество байт данных в пакете
extern char     txDat[frame];   //+ массив данных для передачи

  // state1
extern bool _switchStatus;          // коммутатор ( foff_pin 21 D21 PA23 )
extern bool _converterStatus;       // преобразователь
// extern bool _currentControlStatus;  // регулирование по току
// extern bool _voltageControlStatus;  // регулирование по напряжению
extern bool _chargeStatus;          // заряд
// extern bool _dischargeStatus;       // разряд
// extern bool _pauseStatus;           // пауза
extern bool _pidStatus;             // управление регулятором

  // state2
// extern bool _overHeatingStatus;     // перегрев
// extern bool _overloadStatus;        // перегрузка
// extern bool _powerLimitationStatus; // ограничение мощности
// extern bool _reversePolarityStatus; // обратная полярность
// extern bool _shortCircuitStatus;    // короткое замыкание
// extern bool _calibrationStatus;     // калибровка
// extern bool _upgradeStatus;         // обновление
// extern bool _reserve2Status;        // резерв 2

float kp =  0.1;
float ki =  0.5;
float kd =  0.0;
float hz = 10.0; 
int output_bits = 10; // Set analog out resolution to max, 10-bits
bool output_signed = false; 

uint16_t output     = 0x0000;
//bool     pidStatus  = false;    // false - PID-регулятор отключен 
uint8_t  pidMode    = 0;        // 0-1-2 - тестирование: задать напряжение, ток заряда или ток разряда
// pidReference

FastPID myPID( kp, ki, kd, hz, output_bits, output_signed );

uint16_t setpoint = 512;
uint16_t feedback = 511;

void initPid()
{
  analogWriteResolution( output_bits );
}

void doPid()
{
  if( _pidStatus )
  {
    uint32_t before;
    uint32_t after; 
    before = micros(); 
    //uint16_t output = myPID.step(setpoint, feedback); 
    output = myPID.step(setpoint, feedback); 
    after = micros();
 
    //analogWrite( MPins::dac_pin, output );

    #ifdef DEBUG_PID
    // SerialUSB.print("runtime: "); 
    // SerialUSB.print(after - before); 
    // SerialUSB.print(" sp: "); 
    // SerialUSB.print(setpoint); 
    // SerialUSB.print(" fb: "); 
    // SerialUSB.print(feedback); 
    // SerialUSB.print(" out: "); 
    // SerialUSB.println(output); 
    #endif
  }
  else
  {
//     // output in millivolts
// output = 250;         // Test
//     //analogReference(AR_DEFAULT);
//     //DAC->CTRLB.reg = 0x40;  // ???use AVCC as the reference - DAC Off
//     //analogWrite( MPins::dac_pin, ( output * 0x3ff /4  ) / 3300 ); //1200 = AREF
//                 // найти откуда /4 ?
//     #ifdef DEBUG_PID
//       SerialUSB.print(" out: "); 
//       SerialUSB.println(output); 
//     #endif
  }
  


}

bool setCoefficients( float kp, float ki, float kd, float hz )
{
  return myPID.setCoefficients( kp, ki, kd, hz);
} 

bool setOutputConfig( int bits, bool sign )
{
  return myPID.setOutputConfig( bits, sign );
}

bool setOutputRange( int16_t min, int16_t max )
{
  return myPID.setOutputRange( min, max );
}

bool configure( float kp, float ki, float kd, float hz, int bits, bool sign )
{
  return myPID.configure( kp, ki, kd, hz, bits, sign );
}

void clear()
{
  return myPID.clear();
}

void portsInit()
{
  pinMode( MPins::off_pin,  OUTPUT);  // off_pin  =  2   D4   PA14
  pinMode( MPins::ch_pin,   OUTPUT);  // ch_pin   =  5   D5   PA15
  pinMode( MPins::foff_pin, OUTPUT);  // foff_pin = 21   D21  PA23
  #ifdef WEMOS    // using pcb SAMD21 MINI
    pinMode( MPins::led_rx, OUTPUT);  // led_rx   = 25   no   PB03/LED1 (LED_BUILTIN, LED_RX)
    pinMode( MPins::led_tx, OUTPUT);  // led_tx   = 26   no   PA27/LED2 (LED_TX)
  #endif
  dacInit();                          // Set reference
}

  // ===== Управление дискретными выходами =====

  // Включение/отключение коммутатора (foff_pin = 21  D21 PA23)
void switchFoff(bool on)
{
  digitalWrite( MPins::foff_pin, on );
}

  // Включение/отключение преобразователя (off_pin = 2  D4 PA14)
void converterOff(bool on)
{
  digitalWrite( MPins::off_pin, !on );
}

  // 0x62 Задать напряжение в МВ и включить
void doSetVoltage()
{
  if( rxNbt == 4 )
  {
    txDat[0] = 0x00;                        // Очистить сообщение об ошибках
    uint16_t voltage62 = get16(0);          // Заданное напряжение в милливольтах
    uint16_t factor62  = get16(2);          // Коэффициент преобразования в код ADC
    uint16_t value = voltage62 / factor62;
    if(value >= 0x0400)                     // Если за пределом
    {
      value = 0x3ff;                        // Задать максимум
      txDat[0] = 0x01;                      // и сообщить об ошибке
    } 
    _pidStatus = false;                     // Регулятор отключить чтобы не мешал
    dacWrite10bit( value );                 // Задать код
    _switchStatus     = true;               // коммутатор включить     ( foff_pin = 21 D21 PA23 ) 
    _converterStatus  = true;               // преобразователь включить ( off_pin =  2 D4  PA14 )

    // Подготовить 3 байта ответа: 0 - нет ошибок и код, который ушел в ADC
    txDat[1]  = ( value >> 8) & 0xFF; // Hi
    txDat[2]  =   value & 0xFF;       // Lo
    txNbt = 3;
    txReplay( txNbt, txDat[0] ); 
  }
  else
  {
    txReplay(1, err_tx);                // ошибка протокола
  }
}


  // 0x63 задать ток в мА и включить
void doSetCurrent()
{
  if( rxNbt == 4 )
  {
    uint16_t current63 = get16(0); // Ток в миллиамперах
    uint16_t factor63  = get16(2); // Коэффициент преобразования для ADC
    uint16_t value = current63 / factor63; 
    // dac analogWrite
    //dacWrite( MPins::dac_pin, value ); //( output * 0x3ff /4  ) / 3300 ); //1200 = AREF
    dacWrite10bit( value ); //( output * 0x3ff /4  ) / 3300 ); //1200 = AREF
    _switchStatus     = true;  // коммутатор      ( foff_pin = 21 D21 PA23 ) включить
    _converterStatus  = true;  // преобразователь ( off_pin  = 2  D4  PA14 ) включить

    // reply
    txDat[1]  = ( value >> 8) & 0xFF; // Hi
    txDat[2]  =   value & 0xFF;       // Lo
    txNbt = 3;
    txReplay( txNbt, 0 ); 
  }
  else
  {
    txReplay(1, err_tx);                // ошибка протокола
  }
}

  //  Включение/отключение заряда (ch_pin = 5;  D5   PA15)
void chargerCh(bool on)
{
  digitalWrite( MPins::ch_pin, !on );

}   


void doSwitchFoff()
{
  if( rxNbt == 1 )
  {
    _switchStatus = rxDat[0] & 0x01;  // 

    txReplay( 1, 0 );                   // Об ошибках параметров не сообщается
  }
  else
  {
    txReplay(1, err_tx);                // ошибка протокола
  }   
}

void doConverterOff()
{
  if( rxNbt == 1 )
  {
    _converterStatus = rxDat[0] & 0x01;  // 

    txReplay( 1, 0 );                   // Об ошибках параметров не сообщается
  }
  else
  {
    txReplay(1, err_tx);                // ошибка протокола
  }   
}

void doChargerCh()           // 0x64 ch_pin = 5  D5  PA15  on/off
{
  if( rxNbt == 1 )
  {
    _chargeStatus = rxDat[0] & 0x01;  // 

    txReplay( 1, 0 );                   // Об ошибках параметров не сообщается
  }
  else
  {
    txReplay(1, err_tx);                // ошибка протокола
  }   
}

void doSetDiscurrent()       // 0x65 задать ток и включить
{

}

// =====    =====
