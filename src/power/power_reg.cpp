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
// extern bool _chargeStatus;          // заряд
// extern bool _dischargeStatus;       // разряд
// extern bool _pauseStatus;           // пауза
// extern bool _reserve1Status;        // резерв 1

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
bool     pidStatus  = false;    // false - PID-регулятор отключен 
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
  if( pidStatus )
  {
    uint32_t before;
    uint32_t after; 
    before = micros(); 
    //uint16_t output = myPID.step(setpoint, feedback); 
    output = myPID.step(setpoint, feedback); 
    after = micros();
 
    analogWrite( MPins::dac_pin, output );

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
    // output in millivolts
output = 250;         // Test
    //analogReference(AR_DEFAULT);
    //DAC->CTRLB.reg = 0x40;  // ???use AVCC as the reference - DAC Off
    analogWrite( MPins::dac_pin, ( output * 0x3ff /4  ) / 3300 ); //1200 = AREF
                // найти откуда /4 ?
    #ifdef DEBUG_PID
      SerialUSB.print(" out: "); 
      SerialUSB.println(output); 
    #endif
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
  pinMode( MPins::foff_pin, OUTPUT);
  pinMode( MPins::off_pin,  OUTPUT);
}


  // Включение/отключение коммутатора (foff_pin 21  D21 PA23)
void switchFoff(bool on)
{
  digitalWrite( MPins::foff_pin, on );
  // if(_switchStatus)
  // {
  //   digitalWrite( MPins::foff_pin, LOW );
  // }
  // else
  // {
  //   digitalWrite( MPins::foff_pin, HIGH );
  // }
}

  // Включение/отключение преобразователя (off_pin 2  D4 PA14)
void converterOff(bool on)
{
  digitalWrite( MPins::off_pin, !on );
  // if(!on)
  // {
  //   digitalWrite( MPins::off_pin, LOW );
  // }
  // else
  // {
  //   digitalWrite( MPins::off_pin, HIGH );
  // }
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