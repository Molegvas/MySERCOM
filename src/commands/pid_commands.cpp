
#include "board/mpins.h"
#include "wake/wake.h"
#include "pid_commands.h"
//#include "SAMD_AnalogCorrection.h"
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

extern float kp;
extern float ki;
extern float kd;
extern float hz;
extern int   output_bits; // Set analog out resolution to max
extern bool  output_signed; 


//uint16_t adcVoltage = 0x0123;  // extern
//uint16_t adcCurrent = 0x8081;  // extern

extern bool     _pidStatus;
extern uint8_t  pidMode;
extern uint16_t setpoint;


void doPidTest()
{
  if( rxNbt == 8 )
  {
    uint8_t err = 0;

    uint8_t   _reserve  = rxDat[0];         // зарезервирован
    uint8_t   _pidMode  = rxDat[1] & 0x03;  // 0-1-2-3 - выкл или задать напряжение, ток заряда или ток разряда
    uint16_t  _setpoint = get16(2);  
    int16_t   _min      = get16(4);
    int16_t   _max      = get16(6);
    if( !setOutputRange( _min, _max ) ) err = 0x40; // с учетом PARAM_MULT !!

    #ifdef DEBUG_COMMANDS
      SerialUSB.print("  0: 0x"); SerialUSB.println( _reserve, HEX );
      SerialUSB.print("  1: 0x"); SerialUSB.println( _pidMode, HEX );
      SerialUSB.print("2,3: 0x"); SerialUSB.println( _setpoint, HEX );
      SerialUSB.print("4,5: 0x"); SerialUSB.println( _min, HEX );
      SerialUSB.print("6,7: 0x"); SerialUSB.println( _max, HEX );
      if( err )                   SerialUSB.println( "error" );  
      
      //long mmm = PARAM_MULT;
      //SerialUSB.println( mmm, HEX );

    #endif
    
    if( err )
    {
      _pidStatus = false;            // отключен
      pidMode   = 0;                // не выбран
      setpoint  = 0x00;             // не задано
    }
    else
    {
      _pidStatus = false;            // PID-регулятор отключен
      pidMode   = _pidMode;         // выбор канала регулирования
      setpoint  = _setpoint;        // установка выхода
    }

    analogWrite( MPins::dac_pin, setpoint << 2 );

    txReplay( 1, err );       // 0x40 - ошибка любого параметра myPID.setOutputRange()
  }
  else
  {
    txReplay(1, err_tx);      // Ошибка протокола
  }      
}


// 0x41
void doPidCoefficients()
{
  if( rxNbt == 8 )
  {
    float _kp = (float)getF16(0);
    float _ki = (float)getF16(2);
    float _kd = (float)getF16(4);
    float _hz = (float)getF16(6);
    bool  err = ( setCoefficients( _kp, _ki, _kd, _hz ) );
    #ifdef DEBUG_COMMANDS
      SerialUSB.print("  0: "); SerialUSB.println( _kp, 2 );
      SerialUSB.print("  1: "); SerialUSB.println( _ki, 2 );
      SerialUSB.print("2,3: "); SerialUSB.println( _kd, 2 );
      SerialUSB.print("4,5: "); SerialUSB.println( _hz, 2 );
      if( err ) {               SerialUSB.println( "error" );}
    #endif
    
    if( !err )
    {
      kp = _kp;
      ki = _ki;
      kd = _kd;
      hz = _hz; 
    }

    txReplay( 1, err );       // Ошибка параметра
  }
  else
  {
    txReplay(1, err_tx);      // Ошибка протокола
  }
}

void doPidOutputConfig()
{
  if( rxNbt == 3 )
  {
    uint16_t bits = get16(0);
    bool sign     = rxDat[2];
    bool err = setOutputConfig( (int)bits, sign );
    txReplay( 1, err );
  }
  else
  {
    txReplay(1, err_tx);
  }

}

void doPidOutputRange()
{
  if( rxNbt == 4 )
  {
    int16_t min = get16(0);
    int16_t max = get16(2);
    bool err = setOutputRange( min, max );   //(int16_t min, int16_t max);
    txReplay( 1, err );
  }
  else
  {
    txReplay(1, err_tx);
  }

}

  // исполняется долго
void doPidConfigure()
{
  uint8_t err = 0x00;

  if( rxNbt == 11 )
  {
    float _kp = (float)getF16(0);
    float _ki = (float)getF16(2);
    float _kd = (float)getF16(4);
    float _hz = (float)getF16(6);
    int16_t _bits = get16(8);
    bool _sign = rxDat[10];
    if( !configure( _kp, _ki, _kd, _hz, _bits, _sign ) ) err = 0x40;
    //if( !configure( 0.1, 0.5, 0.0, 10.0, 10, false ) ) err = 0x40;

    #ifdef DEBUG_COMMANDS
      SerialUSB.print("0,1: "); SerialUSB.println( _kp, 2 );
      SerialUSB.print("2,3: "); SerialUSB.println( _ki, 2 );
      SerialUSB.print("4,5: "); SerialUSB.println( _kd, 2 );
      SerialUSB.print("6,7: "); SerialUSB.println( _hz, 2 );
      SerialUSB.print("8,9: 0x"); SerialUSB.println( _bits, HEX );
      SerialUSB.print(" 10: 0x"); SerialUSB.println( _sign, HEX );
      if( err ) {               SerialUSB.println( "error" );}
    #endif
    // if( err )
    // {
    //   kp = _kp;
    //   ki = _ki;
    //   kd = _kd;
    //   hz = _hz;
    //   output_bits   = (int)_bits;
    //   output_signed = _sign; 
    // }
    txReplay( 1, err );  
  }
  else
  {
    txReplay(1, err_tx);
  }
}


void doPidClear()
{
  if( rxNbt == 0 )
  {
    clear();
    txReplay(1, 0);
  }
  else
  {
    txReplay(1, err_tx);
  }
}
