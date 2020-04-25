/*


*/

#include "commands.h"
#include "board/mpins.h"
#include "wake/wake.h"
        #include <FastPID.h>
#include "power/power_reg.h"
#include "stdint.h"
#include <Arduino.h>

// Имя устройства
static constexpr char Info[] = {"Q920dn Rev0.0\n\0"};   //

//bool _powerStatus = false;

    // Коды целевых команд:
static constexpr uint8_t cmd_power_on = 0x20; // 
static constexpr uint8_t cmd_probe    = 0x30; // 

    // ПИД-регулятор
static constexpr uint8_t cmd_set_stop_go        = 0x40; // стоп-пауза-пуск 
static constexpr uint8_t cmd_set_coefficients   = 0x41; // kp, ki, kd, hz 
static constexpr uint8_t cmd_set_output_config  = 0x42; // bits, sign
static constexpr uint8_t cmd_set_output_range   = 0x43; // min, max
static constexpr uint8_t cmd_configure          = 0x44; // kp, ki, kd, hz, bits, sign 
static constexpr uint8_t cmd_pid_test           = 0x45; // mode, setpoint, min, max

    // АЦП - настройки
static constexpr uint8_t cmd_set_adc_bat        = 0x52;
static constexpr uint8_t cmd_set_adc_shunt      = 0x53;
static constexpr uint8_t cmd_set_adc_rtu        = 0x54;
uint8_t res[]  = { 12, 12, 12, 12};
uint8_t mode[] = { 0, 0, 0, 0 }; // conv to eAnalogReference, 0 = AR_DEFAULT

    // ЦАП - настройки
    // https://stackoverflow.com/questions/53542591/using-external-vref-for-samd21-dac
enum dac_reference {
    /** 1V from the internal band-gap reference*/
    DAC_REFERENCE_INT1V = DAC_CTRLB_REFSEL(0),
    /** Analog V<SUB>CC</SUB> as reference */
    DAC_REFERENCE_AVCC  = DAC_CTRLB_REFSEL(1),
    /** External reference on AREF */
    DAC_REFERENCE_AREF  = DAC_CTRLB_REFSEL(2),
};

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


uint16_t adcVoltage = 0x0123;  // extern
uint16_t adcCurrent = 0x8081;  // extern

extern bool     pidStatus;
extern uint8_t  pidMode;
extern uint16_t setpoint;

uint8_t cmd = cmd_nop;

void doPidTest();
void doSetCoefficients();
void doSetOutputConfig();
void doSetOutputRange();
void doConfigure();

void doProbe();
void doInfo();
void doEcho();
void doErr();

void doAdcBat();
void doAdcShunt();
void doAdcRtu();

uint16_t get16(int i)
{
  uint16_t par  = rxDat[i] << 8;
  return(  par |= rxDat[i+1]); 
}

float getF16(int i)
{
  uint16_t par  = (rxDat[i] << 8) & 0xff00;
  par |= rxDat[i+1];
  return (float)par / 100; 
}

void doCommand()
{
  cmd = command;

  if( cmd != cmd_nop)
  {
    #ifdef DEBUG_COMMANDS
      SerialUSB.print(" command -> 0x"); SerialUSB.println(cmd, HEX);
    #endif

    switch( cmd )
    {
      
      case cmd_probe :
        doProbe();
        #ifdef DEBUG_COMMANDS
          SerialUSB.println("Probe done");
        #endif
      break;

      //case cmd_ ...


      case cmd_pid_test :
        doPidTest();
        #ifdef DEBUG_COMMANDS
          SerialUSB.println("PidStopGo done");
        #endif
      break;

      case cmd_set_coefficients :
        doSetCoefficients();
        #ifdef DEBUG_COMMANDS
          SerialUSB.println("Coefficients done");
        #endif
      break;

      case cmd_set_output_config :
        doSetOutputConfig();
        #ifdef DEBUG_COMMANDS
          SerialUSB.println("Output Config done");
        #endif
      break;

      case cmd_set_output_range :
        doSetOutputRange();
        #ifdef DEBUG_COMMANDS
          SerialUSB.println("Output Range done");
        #endif
      break;

      case cmd_configure :
        doConfigure();
        #ifdef DEBUG_COMMANDS
          SerialUSB.println("Configure done");
        #endif
      break;

      case cmd_err :
        doErr();
        #ifdef DEBUG_COMMANDS
          SerialUSB.println("Err done");
        #endif
      break;

      case cmd_echo :
        doEcho();
        #ifdef DEBUG_COMMANDS
          SerialUSB.println("Echo done");
        #endif
      break;

      case cmd_info :
        doInfo();
        #ifdef DEBUG_COMMANDS
          SerialUSB.print("Info done N= ");
        #endif
      break;

      case cmd_set_adc_bat :
        doAdcBat();
        #ifdef DEBUG_COMMANDS
          SerialUSB.print("ADC Bat done ");
        #endif
      break;

      case cmd_set_adc_shunt :
        doAdcShunt();
        #ifdef DEBUG_COMMANDS
          SerialUSB.print("ADC Shunt done ");
        #endif
      break;

      case cmd_set_adc_rtu :
        doAdcRtu();
        #ifdef DEBUG_COMMANDS
          SerialUSB.print("ADC Rtu done ");
        #endif
      break;


      default : 
      ;
    }
    cmd = cmd_nop;
  }
}

// передать информацию об устройстве
void doInfo()
{
  char ch = 1;
  int i = 0;

  for( i = 0; i < frame && ch; i++ )
  {
  ch = txDat[i] = Info[i];

  #ifdef DEBUG_WAKE
    Serial.print( ch );
  #endif
  }
  
  txReplay( i, txDat[0] );
}

// передать эхо
void doEcho()
{
  for( int i = 0; i < rxNbt && i < frame; i++ )
  txDat[i] = rxDat[i];
  txReplay( rxNbt, txDat[0] );
  #ifdef DEBUG_WAKE
    Serial.print("команда эхо = "); Serial.print( rxNbt );
  #endif
}

// ошибка приема пакета
void doErr()
{
  txReplay(1, err_tx);
  #ifdef DEBUG_WAKE
    Serial.println("обработка ошибки");
  #endif
}

// отправить данные измерений
void doProbe()
{
  txDat[0] = ( adcVoltage >> 8) & 0xFF; // Hi
  txDat[1] =   adcVoltage & 0xFF;       // Lo
  txDat[2] = ( adcCurrent >> 8) & 0xFF; // Hi
  txDat[3] =   adcCurrent & 0xFF;       // Lo
  txDat[4] =   0x77;      // mcr1
  txDat[5] =   0x88;      // mcr2

  txNbt = 6;
  txReplay( txNbt, txDat[0] );
  #ifdef DEBUG_WAKE
    Serial.println("измерения");
  #endif
}


void doPidTest()
{
  if( rxNbt == 8 )
  {
    bool err = false;

    uint8_t _reserve     = rxDat[0];    // зарезервирован
    uint8_t _pidMode = rxDat[1];    // 0-1-2 - задать напряжение, ток заряда или ток разряда
    if( _pidMode > 3 ) err = true;  // проверить на корректность <3

    uint16_t _setpoint  = get16(2);  
    int16_t _min        = get16(4);
    int16_t _max        = get16(6);
    err = err || setOutputRange( _min, _max ); // с учетом PARAM_MULT !!

    #ifdef DEBUG_COMMANDS
      SerialUSB.print("  0: 0x"); SerialUSB.println( _reserve, HEX );
      SerialUSB.print("  1: 0x"); SerialUSB.println( _pidMode, HEX );
      SerialUSB.print("2,3: 0x"); SerialUSB.println( _setpoint, HEX );
      SerialUSB.print("4,5: 0x"); SerialUSB.println( _min, HEX );
      SerialUSB.print("6,7: 0x"); SerialUSB.println( _max, HEX );
      if( err )                   SerialUSB.println( "error" );
    #endif
    
    if(!err)
    {
      pidStatus = false;                      // PID-регулятор отключен
      pidMode   = _pidMode;                   // выбор канала регулирования
      setpoint  = _setpoint;                  // установка выхода
      analogWrite( MPins::dac_pin, setpoint << 2 );
    }

    txReplay( 1, err );       // true - ошибка любого параметра
  }
  else
  {
    txReplay(1, err_tx);      // Ошибка протокола
  }      
}



void doSetCoefficients()
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

void doSetOutputConfig()
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

void doSetOutputRange()
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

void doConfigure()
{
  if( rxNbt == 8 )
  {
    float kp = (float)get16(0) / 100;
    float ki = (float)get16(2) / 100;
    float kd = (float)get16(4) / 100;
    float hz = (float)get16(6) / 100;
    int bits = 16;
    bool sign = false;
    bool err = configure( kp, ki, kd, hz, bits, sign );
    txReplay( 1, err );  
  }
  else
  {
    txReplay(1, err_tx);
  }
}

// Команды настроек АЦП
void doAdcBat()
{
    if( rxNbt == 4 )
  {
    uint8_t err   = 0x00;
    uint8_t i     = rxDat[0];    
    uint8_t _res   = rxDat[1];                              // в инициализацию каждого измерителя
    uint8_t _mode  = rxDat[2];
    if( i     > 3 ) err |= 0x51;   // Ошибка выбора канала измерения
    if( _res  > 5 ) err |= 0x52;   // Ошибка выбора Resolution  (16?)
    if( _mode > 6 ) err |= 0x54;   // Ошибка выбора Ref или Gain
    if( !err )
    {
      res[i]  = _res;
      mode[i] = _mode;
    }
    txReplay( 1, err );
  }
  else
  {
    txReplay(1, err_tx);
  }
}

void doAdcShunt(){}
void doAdcRtu(){}