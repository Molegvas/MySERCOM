/*


*/

#include <Arduino.h>
#include "wake/wake.h"


char          Rx_Sta;        // состояние процесса приема пакета
char          Rx_Pre;        // предыдущий принятый байт
char          Rx_Add;        // адрес, с которым сравнивается принятый
char          Rx_Cmd;        // принятая команда
char          Rx_Crc;        // контрольная сумма принимаемого пакета
unsigned char Rx_Ptr;        // указатель на массив принимаемых данных

char          txSta;        // состояние процесса передачи пакета
char          txPre;        // предыдущий переданный байт
char          txAdd;        // адрес, передававемый в пакете
char          txCrc;        // контрольная сумма передаваемого пакета
unsigned char txPtr;        // указатель на массив передаваемых данных

//RX process states:
enum { WAIT_FEND,    //ожидание приема FEND == 0
      WAIT_ADDR,     //ожидание приема адреса
      WAIT_CMD,      //ожидание приема команды
      WAIT_NBT,      //ожидание приема количества байт в пакете
      WAIT_DATA,     //прием данных
      WAIT_CRC,      //ожидание окончания приема CRC
      WAIT_CARR };   //ожидание несущей

//TX process states:
enum { send_idle,     //состояние бездействия == 0
        send_addr,     //передача адреса
        send_cmd,      //передача команды
        send_nbt,      //передача количества байт в пакете
        send_data,     //передача данных
        send_crc,      //передача CRC
        send_end };    //окончание передачи пакета nu?

// Переменные - уточнить типы  
char    RxNbt;         // принятое количество байт в пакете
char    RxDat[frame];  // массив принятых данных
uint8_t Command;    // код команды на выполнение

char    TxCmd;         // команда, передаваемая в пакете
uint8_t TxNbt;         // количество байт данных в пакете
char    TxDat[frame];  // массив данных для передачи


void wakeInit( uint8_t addr )
{
  Rx_Add  = addr;                      // адрес на прием
  txAdd   = addr;                      // адрес на передачу
  Rx_Sta  = WAIT_FEND;                 // ожидание пакета
  txSta   = send_idle;                 // ничего пока не передаем
  Command = cmd_nop;                   // нет команды на выполнение
}


// Вычисление контрольной суммы
void doCrc8(char b, char *crc)
{
	char i;
  for(i = 0; i < 8; b = b >> 1, i++)
    if((b ^ *crc) & 1) *crc = ((*crc ^ 0x18) >> 1) | 0x80;
     else *crc = (*crc >> 1) & ~0x80;
}


// Передача пакета
void wakeWrite()
{
  char dataByte;

  if( txPre == fend )                     // если производится стаффинг,
  {
    dataByte = tfend;                     // передача TFEND вместо FEND
    txPre = dataByte;
		Serial1.write( dataByte );            // dataByte -> UART
    return;
  }

  if( txPre == fesc )                     // если производится стаффинг,
  {
    dataByte = tfesc;                     // передача TFESC вместо FESC
    txPre = dataByte;
		Serial1.write( dataByte );            // dataByte -> UART
    return;
  }

  switch( txSta )
  {
  case send_addr:                         // -----> передача адреса
    {
      if( txAdd )                         // если адрес не равен нулю, передаем его
      {
        dataByte = txAdd;
        doCrc8( dataByte, &txCrc );       // вычисление CRC для истинного адреса
        dataByte |= 0x80;                 // установка бита 7 для передачи адреса
        txSta = send_cmd;
        break;
      }                                   // иначе сразу передаем команду
    }
  case send_cmd:                          // -----> передача команды
    {
      dataByte = TxCmd & 0x7F;
      txSta = send_nbt;
      break;
    }
  case send_nbt:                          // -----> передача количества байт
    {
      dataByte = TxNbt;
      txSta = send_data;
      txPtr = 0;                          // обнуление указателя данных для передачи
      break;
    }
  case send_data:                         // -----> передача данных
    {
      if(txPtr < TxNbt)
        dataByte = TxDat[ txPtr++ ];  
      else
      {
        dataByte = txCrc;                 // передача CRC
        txSta = send_crc;
      }
      break;
    }
  default:
    {
      txSta = send_idle;                  // передача пакета завершена
      return;
    }
  }

  if(txSta != send_cmd)                   // если не передача адреса, то
  {
    doCrc8( dataByte, &txCrc );           // вычисление CRC
  }

  txPre = dataByte;
                         // сохранение пре-байта
  if( dataByte == fend || dataByte == fesc )
  {
    dataByte = fesc;                      // передача FESC, если нужен стаффинг
  }

  Serial1.write( dataByte );
}

// Инициализация передачи пакета
void wakeStartWrite()
{
  char dataByte = fend;
	int countByte = 0;
  txCrc = crc_init;                       // инициализация CRC,
  doCrc8( dataByte, &txCrc );             // обновление CRC
  txSta = send_addr;
  txPre = tfend;
	Serial1.write( dataByte );              // dataByte -> UART
	  
	do
	{
		wakeWrite();                          // all bytes -> UART
		countByte++;
	}
	while( txSta != send_idle || countByte < 518 );

	//if(txSta == 0x02)       //??
	if( countByte >= 518 )
	{
		txSta = send_idle;
	}
}

// передача ответа на команду 
void txReplay(char n, char err)
{
  TxNbt = n;                      // количество байт
  TxDat[0] = err;                 // код ошибки
  TxCmd = Command;                // команда
  wakeStartWrite();               // инициализация передачи
  Command = cmd_nop;              // команда обработана
}