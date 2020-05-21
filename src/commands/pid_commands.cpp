
#include "board/mpins.h"
#include "wake/wake.h"
#include "pid_commands.h"
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

extern float kP[];
extern float kI[];
extern float kD[];
extern bool  signOut[];
extern float hz;              // const?
extern int   output_bits; // Set analog out resolution to max const?

//extern bool  output_signed; 


// предварительно ...
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



// void doPidOutputConfig()
// {
//   if( rxNbt == 3 )
//   {
//     uint16_t bits = get16(0);
//     bool sign     = rxDat[2];
//     bool err = setOutputConfig( (int)bits, sign );
//     txReplay( 1, err );
//   }
//   else
//   {
//     txReplay(1, err_tx);
//   }

// }

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

  // 0x40    Конфигурирование пид-регулятора
void doPidConfigure()
{
  uint8_t err = 0x00;

  if( rxNbt == 9 )
  {
    uint8_t mode = rxDat[0] & 0x03;   // Выбор режима ( U, I, D, defolts)
    float _kp = (float)getF16(1);
    float _ki = (float)getF16(3);
    float _kd = (float)getF16(5);
    //int16_t _bits = get16(8); // всегда 10
    bool _sign = rxDat[7] & 0x01;
    bool _replace = rxDat[8] & 0x01;
    if(_replace)
    {
      // Это та же процедура, только без очистки регистров регулятора:
      // _last_sp = 0; _last_out = 0; _sum = 0; _last_err = 0;
      if( !replaceConfigure( _kp, _ki, _kd, 10.0, 10, _sign ) ) err = 0x41;  // такой код ошибки
    }
    else
    {
      if( !configure( _kp, _ki, _kd, 10.0, 10, _sign ) ) err = 0x40;  // такой код ошибки
    }
    #ifdef DEBUG_PID
      SerialUSB.print(" kp: ");     SerialUSB.println( _kp, 2 );
      SerialUSB.print(" ki: ");     SerialUSB.println( _ki, 2 );
      SerialUSB.print(" kd: ");     SerialUSB.println( _kd, 2 );
      SerialUSB.print(" sn: 0x");   SerialUSB.println( _sign, HEX );
      if( err )                    {SerialUSB.println("error");}
    #endif

    // Если ошибок нет, иначе configure() будет со всеми нулями
    if( !err )
    {
      kP[mode]      = _kp;
      kI[mode]      = _ki;
      kD[mode]      = _kd;
      signOut[mode] = _sign; 
    }
    txReplay( 1, err );  
  }
  else
  {
    txReplay(1, err_tx);
  }
}

// 0x41 Нужна команда конфигурирования без сброса регистров для 
// перехода из режима в режим (CCCV)
// // 0x41 Тестовая: ввод коэффициентов kp, ki, kd с проверкой корректности 
// void doPidCoefficients()
// {
//   if( rxNbt == 7 )
//   {
//     uint8_t mode = rxDat[0] & 0x03;   // Выбор режима ( U, I, D, defolts)
//     float _kp = (float)getF16(1);
//     float _ki = (float)getF16(3);
//     float _kd = (float)getF16(5);
//     //float _hz = (float)getF16(6);     // отменено, hz всегда 10.0
//     // Загрузка с проверкой корректности данных
//     bool  err = ( setCoefficients( _kp, _ki, _kd, 10.0f ) );
//     //#ifdef DEBUG_COMMANDS
//       SerialUSB.print(" kp: "); SerialUSB.println( _kp, 2 );
//       SerialUSB.print(" ki: "); SerialUSB.println( _ki, 2 );
//       SerialUSB.print(" kd: "); SerialUSB.println( _kd, 2 );
//       //SerialUSB.print("4,5: "); SerialUSB.println( _hz, 2 );
//       if( err ) {               SerialUSB.println( "error" );}
//     //#endif
    
//     if( !err )
//     {                         // Если данные корректны, запомнить в настройках
//       kP[mode] = _kp;
//       kI[mode] = _ki;
//       kD[mode] = _kd;
//     }

//     txReplay( 1, err );       // Ошибка параметра
//   }
//   else
//   {
//     txReplay(1, err_tx);      // Ошибка протокола
//   }
// }






// 0x44 Очистка регистров регулятора, в том числе и накопленных ошибок?? преобразования параметров 
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
