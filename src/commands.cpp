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
extern char     RxNbt;          //+ принятое количество байт в пакете
extern char     RxDat[frame];   //+ массив принятых данных
//extern uint8_t  Command;        // код команды на выполнение

//extern char     TxCmd;          // команда, передаваемая в пакете
//extern char     TxNbt;          // количество байт данных в пакете
extern char     TxDat[frame];   //+ массив данных для передачи


extern uint8_t Command;    // код команды на выполнение   // Test

uint8_t cmd = cmd_nop;

void doInfo();
void doEcho();
void doErr();



void doCommand()
{

Command = cmd_err;
cmd = cmd_err;           // Test 

  if( cmd != cmd_nop)
  {
    switch( cmd )
    {
      
      case cmd_probe :
        // doProbe();
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
  ch = TxDat[i] = Info[i];
  //ch = Info[i]; 

  #ifdef DEBUG_WAKE
      Serial.print( ch );
  #endif
  }
  //txReplay( i, Tx_Dat[0] );
  txReplay( i, TxDat[0] );
}

// передать эхо
void doEcho()
{
  //for( i = 0; i < Rx_Nbt && i < MWake::frame; i++ )
  for( int i = 0; i < RxNbt && i < frame; i++ )
  //Tx_Dat[i] = Rx_Dat[i];
  TxDat[i] = RxDat[i];
  //txReplay( Rx_Nbt, Tx_Dat[0] );
  txReplay( RxNbt, TxDat[0] );
  #ifdef DEBUG_WAKE
      Serial.print("команда эхо = "); Serial.print( RxNbt );
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