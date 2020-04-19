/*


*/

#include "commands.h"
#include "wake/wake.h"
        #include <FastPID.h>
#include "power/power_reg.h"
#include "stdint.h"
#include <Arduino.h>

// Имя устройства
static constexpr char Info[] = {"Q920dn Rev0.0\n\0"};   //



    // Коды целевых команд:
static constexpr uint8_t cmd_power_on = 0x20; // 
static constexpr uint8_t cmd_probe    = 0x30; // 

    // ПИД-регулятор
static constexpr uint8_t cmd_set_coefficients   = 0x40; // 
static constexpr uint8_t cmd_set_output_config  = 0x41; // 
static constexpr uint8_t cmd_set_output_range   = 0x42; // 
static constexpr uint8_t cmd_configure          = 0x43; // 

    // АЦП - настройки
static constexpr uint8_t cmd_set_adc_bat        = 0x52;
static constexpr uint8_t cmd_set_adc_shunt      = 0x53;
static constexpr uint8_t cmd_set_adc_rtu        = 0x54;
uint8_t res[]  = { 12, 12, 12, 12};
uint8_t mode[] = { 0, 0, 0, 0 }; // conv to eAnalogReference, 0 = AR_DEFAULT



    // Переменные - уточнить типы  
extern char     rxNbt;          //+ принятое количество байт в пакете
extern char     rxDat[frame];   //+ массив принятых данных
extern uint8_t  command;        // код команды на выполнение

//extern char     TxCmd;          // команда, передаваемая в пакете
extern char     txNbt;          // количество байт данных в пакете
extern char     txDat[frame];   //+ массив данных для передачи


uint16_t adcVoltage = 0x0123;  // extern
uint16_t adcCurrent = 0x8081;  // extern



uint8_t cmd = cmd_nop;

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
  uint16_t par = rxDat[0] << 8;
  return(par |= rxDat[1]); 
}

void doCommand()
{

//command = cmd_probe;
//cmd = cmd_probe;           // Test 

  if( cmd != cmd_nop)
  {
    switch( cmd )
    {
      
      case cmd_probe :
        doProbe();
        #ifdef DEBUG_COMMANDS
          SerialUSB.println("Probe done");
        #endif
      break;

      //case cmd_ ...

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
  //ch = Info[i]; 

  #ifdef DEBUG_WAKE
    Serial.print( ch );
  #endif
  }
  //txReplay( i, Tx_Dat[0] );
  txReplay( i, txDat[0] );
}

// передать эхо
void doEcho()
{
  //for( i = 0; i < Rx_Nbt && i < MWake::frame; i++ )
  for( int i = 0; i < rxNbt && i < frame; i++ )
  //Tx_Dat[i] = Rx_Dat[i];
  txDat[i] = rxDat[i];
  //txReplay( Rx_Nbt, Tx_Dat[0] );
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

void doSetCoefficients()
{
  if( rxNbt == 8 )
  {
    float kp = (float)get16(0) / 100;
    float ki = (float)get16(2) / 100;
    float kd = (float)get16(4) / 100;
    float hz = (float)get16(6) / 100;
    //bool err = myPID.setCoefficients( kp, ki, kd, hz);
    bool err = setCoefficients( kp, ki, kd, hz );
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