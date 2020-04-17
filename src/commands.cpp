/*


*/

#include "commands.h"
#include "wake/wake.h"
#include "stdint.h"
#include <Arduino.h>

// Имя устройства
static constexpr char Info[] = {"Q920dn Rev0.0\n\0"};   //



    // Коды целевых команд:
static constexpr uint8_t cmd_power_on = 0x20; // 
static constexpr uint8_t cmd_probe    = 0x30; // 


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

void doProbe();
void doInfo();
void doEcho();
void doErr();



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
