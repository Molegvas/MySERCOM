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
extern bool switchStatus;          // коммутатор ( foff_pin 21 D21 PA23 )
extern bool converterStatus;       // преобразователь
// extern bool currentControlStatus;  // регулирование по току
// extern bool voltageControlStatus;  // регулирование по напряжению
extern bool chargeStatus;          // заряд
// extern bool dischargeStatus;       // разряд
// extern bool pauseStatus;           // пауза
extern bool pidStatus;             // управление регулятором

  // state2
// extern bool overHeatingStatus;     // перегрев
// extern bool overloadStatus;        // перегрузка
// extern bool powerLimitationStatus; // ограничение мощности
// extern bool reversePolarityStatus; // обратная полярность
// extern bool shortCircuitStatus;    // короткое замыкание
// extern bool calibrationStatus;     // калибровка
// extern bool upgradeStatus;         // обновление
// extern bool reserve2Status;        // резерв 2

  // Данные АЦП
extern uint16_t adcVoltage;
extern uint16_t adcCurrent;
  // Пересчитанные в физические величины - mV, mA
extern int16_t voltage;
extern int16_t current;

  // Опорное напряжение ADC в милливольтах
constexpr uint16_t avcc = 3300;

  // Приборные диапазоны задания напряжения и токов
constexpr uint16_t volt_min       =  2000;   //  2.0 в
constexpr uint16_t volt_max       = 20000;   // 20.0 в
constexpr uint16_t curr_ch_min    =    50;   //  0.05 А
constexpr uint16_t curr_ch_max    = 12000;   // 12.0 А
constexpr uint16_t curr_disch_min =    50;   //  0.05 А
constexpr uint16_t curr_disch_max =  3000;   //  3.0 А

// Дефолтные параметры регулирования для всех режимов
constexpr float _kp       =  0.1f;  
constexpr float _ki       =  0.5f;
constexpr float _kd       =  0.0f;
constexpr float _hz       = 10.0f;
constexpr uint16_t _bits  = 10;
constexpr uint16_t _min   = 0x0000;
constexpr uint16_t _max   = 0x03ff;


// Варианты настройки для разных режимов (modes) регулирования 
// напряжения, тока заряда, тока разряда и резервая позиция (дефолтные значения)
// Для разряда (режим D) может использоваться другой экземпляр регулятора
// Разрядность (10бит) и опорное (AVCC) DAC заданы жестко, как и частота (hz=10Hz) 
enum mode { U = 0, I, D, R };

// Для тестирования при непосредственном соединении входа с выходом DAC  kp = 0.02
float kP[]          = {  0.02,  0.02,  0.02,   _kp };  
float kI[]          = {   _ki,   _ki,   _ki,   _ki };
float kD[]          = {   _kd,   _kd,   _kd,   _kd };
bool signOut[]      = { false, false, false, false };
uint16_t minOut[]   = {  _min,  _min,  _min,  _min };
uint16_t maxOut[]   = {  _max,  _max,  _max,  _max };
uint16_t setpoint[] = { 0x0000 };

uint16_t output     = 0x0000;
uint8_t  pidMode    = 0;        // 0-1-2 - тестирование: задать напряжение, ток заряда или ток разряда
// pidReference - без выбора, всегда AVDD

FastPID myPID( kP[U], kI[U], kD[U], _hz, _bits, signOut[U] );  // Voltage control mode

uint16_t feedback = 0x0100;

void initPid()
{
  analogWriteResolution( _bits );
}


void doPid()
{
  uint16_t _setpoint = 0x0000;

  if( pidStatus )
  {
    #ifdef DEBUG_PID
      //uint32_t before = micros();
    #endif

    // ОС по напряжению
    //feedback = voltage;
    //_setpoint = setpoint[U];

    // ОС по току
    feedback = current;
    _setpoint = setpoint[I];


    output = myPID.step(_setpoint, feedback);
    dacWrite10bit( output & 0x3ff );                 // Задать код

    #ifdef DEBUG_PID
      //uint32_t after = micros();
      //SerialUSB.print("runtime: "); SerialUSB.print(after - before); 
      SerialUSB.print(" sp: ");     SerialUSB.print(_setpoint);     
      SerialUSB.print(" fb: ");     SerialUSB.print(feedback);
      SerialUSB.print(" out: ");    SerialUSB.println(output); 
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

bool replaceConfigure( float kp, float ki, float kd, float hz, int bits, bool sign )
{
  return myPID.replaceConfig( kp, ki, kd, hz, bits, sign );
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

  // 0x62 Задать напряжение в мВ и включить
void doSetVoltage()
{
  if( rxNbt == 5 )
  {
    txDat[0] = 0x00;                      // Очистить сообщение об ошибках

    pidStatus       = rxDat[0] & 0x01;   // Регулятор отключить или включить
    uint16_t voltage = get16(1);          // Заданное напряжение в милливольтах
    uint16_t factor  = get16(3);          // Коэффициент преобразования в код ADC
    
    if(voltage < volt_min)                // Если за пределом
    {
      voltage = volt_min;                 // Задать минимум
      txDat[0] = 0x01;                    // и сообщить об ошибке
    }

    if(voltage > volt_max)                // Если за пределом
    {
      voltage = volt_max;                 // Задать максимум
      txDat[0] = 0x01;                    // и сообщить об ошибке
    } 

    uint16_t value = voltage / factor;

    // Задать условия, установить напряжение 
    chargeStatus    = true;              // заряд, иное невозможно
    switchStatus    = true;              // коммутатор включить     ( foff_pin = 21 D21 PA23 ) 
    converterStatus = true;              // преобразователь включить ( off_pin =  2 D4  PA14 )

    if(pidStatus)
    {
      setpoint[U] = value / 4;   // vSetpoint
      // запустить
    }
    else
    {
      dacWrite10bit( value );              // Задать код
    }

    // Подготовить 3 байта ответа: 0 - нет ошибок и код, который ушел в ADC или setpoint
    txDat[1] = ( value >> 8) & 0xFF;       // Hi
    txDat[2] =   value & 0xFF;             // Lo
    txNbt = 3;
    txReplay( txNbt, txDat[0] ); 
  }
  else
  {
    txReplay(1, err_tx);                    // ошибка протокола
  }
} // !doSetVoltage()

  // 0x63 задать ток в мА и включить
void doSetCurrent()
{
  if( rxNbt == 5 )
  {
    txDat[0] = 0x00;                      // Очистить сообщение об ошибках

    pidStatus       = rxDat[0] & 0x01;   // Регулятор отключить или включить
    uint16_t _setpoint = get16(1);          // Заданный ток в миллиамперах
    uint16_t factor  = get16(3);          // Коэффициент преобразования в код ADC
    
    if(_setpoint < curr_ch_min)             // Если за пределом
    {
      _setpoint = curr_ch_min;              // Задать минимум
      txDat[0] = 0x01;                    // и сообщить об ошибке
    }

    if(_setpoint > curr_ch_max)             // Если за пределом
    {
      _setpoint = curr_ch_max;              // Задать максимум
      txDat[0] = 0x01;                    // и сообщить об ошибке
    } 

    uint16_t value = current / factor;

    // Задать условия, установить напряжение 
    chargeStatus    = true;              // заряд, иное невозможно
    switchStatus    = true;              // коммутатор включить     ( foff_pin = 21 D21 PA23 ) 
    converterStatus = true;              // преобразователь включить ( off_pin =  2 D4  PA14 )

    if(pidStatus)
    {
      setpoint[I] = value / 4;
      // запустить
    }
    else
    {
      dacWrite10bit( value );              // Задать код
    }

    // Подготовить 3 байта ответа: 0 - нет ошибок и код, который ушел в ADC или setpoint
    txDat[1] = ( value >> 8) & 0xFF;       // Hi
    txDat[2] =   value & 0xFF;             // Lo
    txNbt = 3;
    txReplay( txNbt, txDat[0] ); 
  }
  else
  {
    txReplay(1, err_tx);                    // ошибка протокола
  }
}



// {
//   if( rxNbt == 4 )
//   {
//     txDat[0] = 0x00;                        // Очистить сообщение об ошибках
//     uint16_t current63 = get16(0);          // Заданный ток в миллиамперах
//     uint16_t factor63  = get16(2);          // Коэффициент преобразования в код ADC
//     uint16_t value = current63 / factor63;

//     if(value >= 0x0400)                     // Если за пределом
//     {
//       value = 0x3ff;                        // Задать максимум
//       txDat[0] = 0x01;                      // и сообщить об ошибке
//     }

//     // Задать условия, установить ток (нагрузка должна быть на клеммах) 
//     chargeStatus    = true;                // заряд, иное невозможно
//     pidStatus       = false;               // Регулятор отключить чтобы не мешал
//     switchStatus    = true;                // коммутатор включить     ( foff_pin = 21 D21 PA23 ) 
//     converterStatus = true;                // преобразователь включить ( off_pin =  2 D4  PA14 )
//     dacWrite10bit( value );                 // Задать код

//     // Подготовить 3 байта ответа: 0 - нет ошибок и код, который ушел в ADC
//     txDat[1]  = ( value >> 8) & 0xFF;       // Hi
//     txDat[2]  =   value & 0xFF;             // Lo
//     txNbt = 3;
//     txReplay( txNbt, txDat[0] ); 
//   }
//   else
//   {
//     txReplay(1, err_tx);                    // ошибка протокола
//   }
// }

  // 0x65 задать ток разряда в мА и включить
void doSetDiscurrent()
{
  if( rxNbt == 4 )
  {
    txDat[0] = 0x00;                        // Очистить сообщение об ошибках
    uint16_t current65 = get16(0);          // Заданный ток в миллиамперах
    uint16_t factor65  = get16(2);          // Коэффициент преобразования в код ADC
    uint16_t value = current65 / factor65;

    if(value >= 0x0400)                     // Если за пределом
    {
      value = 0x3ff;                        // Задать максимум
      txDat[0] = 0x01;                      // и сообщить об ошибке
    }

    // Задать условия и установить ток ( на клеммах должна быть заряженная батарея ) 
    chargeStatus    = false;               // это разряд, иное невозможно
    pidStatus       = false;               // Регулятор отключить чтобы не мешал
    switchStatus    = true;                // коммутатор включить      ( foff_pin = 21 D21 PA23 ) 
    converterStatus = false;               // преобразователь выключить ( off_pin =  2 D4  PA14 )
    dacWrite10bit( value );                 // Задать код

    // Подготовить 3 байта ответа: 0 - нет ошибок и код, который ушел в ADC
    txDat[1]  = ( value >> 8) & 0xFF;       // Hi
    txDat[2]  =   value & 0xFF;             // Lo
    txNbt = 3;
    txReplay( txNbt, txDat[0] ); 
  }
  else
  {
    txReplay(1, err_tx);                    // ошибка протокола
  }
}

  // 0x66 задать поддержание напряжения пид-регулятором




  //  Включение/отключение заряда (ch_pin = 5;  D5   PA15)
void chargerCh(bool on)
{
  digitalWrite( MPins::ch_pin, !on );
}   


void doSwitchFoff()
{
  if( rxNbt == 1 )
  {
    switchStatus = rxDat[0] & 0x01;  // 

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
    converterStatus = rxDat[0] & 0x01;  // 

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
    chargeStatus = rxDat[0] & 0x01;  // 

    txReplay( 1, 0 );                   // Об ошибках параметров не сообщается
  }
  else
  {
    txReplay(1, err_tx);                // ошибка протокола
  }   
}



// =====    =====
