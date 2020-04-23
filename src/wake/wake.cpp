/*


*/

#include <Arduino.h>
#include "wake/wake.h"


char          rxSta;        // состояние процесса приема пакета
char          rxPre;        // предыдущий принятый байт
char          rxAdd;        // адрес, с которым сравнивается принятый
char          rxCmd;        // принятая команда
char          rxCrc;        // контрольная сумма принимаемого пакета
unsigned char rxPtr;        // указатель на массив принимаемых данных

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
enum {  send_idle,     //состояние бездействия == 0
        send_addr,     //передача адреса
        send_cmd,      //передача команды
        send_nbt,      //передача количества байт в пакете
        send_data,     //передача данных
        send_crc,      //передача CRC
        send_end };    //окончание передачи пакета nu?

// Переменные - уточнить типы  
char    rxNbt;         // принятое количество байт в пакете
char    rxDat[frame];  // массив принятых данных
uint8_t command;    // код команды на выполнение

char    txCmd;         // команда, передаваемая в пакете
uint8_t txNbt;         // количество байт данных в пакете
char    txDat[frame];  // массив данных для передачи


void wakeInit( uint8_t addr )
{
  rxAdd   = addr;                      // адрес на прием
  txAdd   = addr;                      // адрес на передачу
  rxSta   = WAIT_FEND;                 // ожидание пакета
  txSta   = send_idle;                 // ничего пока не передаем
  command = cmd_nop;                   // нет команды на выполнение
}


// Вычисление контрольной суммы
void doCrc8(char b, char *crc)
{
	char i;
  for(i = 0; i < 8; b = b >> 1, i++)
    if((b ^ *crc) & 1) *crc = ((*crc ^ 0x18) >> 1) | 0x80;
     else *crc = (*crc >> 1) & ~0x80;
}


// Чение данных порта, при обнаружении конца пакета
// устанавливает ненулевой код требующей обработки команды.
void wakeRead()
{
  char error_flags = 0;               // чтение флагов ошибок UART - пока нет
  
  uint8_t dataByte;
  Serial1.readBytes( &dataByte, 1 );  // чтение одного байта с удалением из приемного буфера
    // SerialUSB.print("Byte ->0x"); SerialUSB.println( dataByte, HEX );
  char Pre = rxPre;                   // сохранение старого пре-байта

  if( error_flags )                   // если обнаружены ошибки при приеме байта
  {
    rxSta = WAIT_FEND;                // ожидание нового пакета
    command = cmd_err;                // сообщаем об ошибке
    return;
  }

  if( dataByte == fend )              // если обнаружено начало фрейма,
  {
    rxPre = dataByte;                // то сохранение пре-байта,
    rxCrc = crc_init;                // инициализация CRC,
    rxSta = WAIT_ADDR;               // сброс указателя данных,
    doCrc8( dataByte, &rxCrc );      // обновление CRC,
    return;                           // выход
  }

  if( rxSta == WAIT_FEND )           // -----> если ожидание FEND,
    return;                           // то выход

  rxPre = dataByte;                  // обновление пре-байта
  if( Pre == fesc )                   // если пре-байт равен FESC,
  {
    if( dataByte == tfesc )           // а байт данных равен TFESC,
      dataByte = fesc;                // то заменить его на FESC
    else if( dataByte == tfend )      // если байт данных равен TFEND,
           dataByte = fend;           // то заменить его на FEND
         else
         {
           rxSta = WAIT_FEND;        // для всех других значений байта данных,
           command = cmd_err;         // следующего за FESC, ошибка
           return;
         }
  }
  else
  {
    if( dataByte == fesc )            // если байт данных равен FESC, он просто
      return;                         // запоминается в пре-байте
  }

  switch( rxSta )
  {
    case WAIT_ADDR:                     // -----> ожидание приема адреса
    {
      if( dataByte & 0x80 )           // если dataByte.7 = 1, то это адрес
      {
        dataByte = dataByte & 0x7F;   //обнуляем бит 7, получаем истинный адрес
        if( !dataByte || dataByte == rxAdd )   // если нулевой или верный адрес,
        {
          doCrc8( dataByte, &rxCrc );  // то обновление CRC и
          rxSta = WAIT_CMD;          // переходим к приему команды
          break;
        }
        rxSta = WAIT_FEND;           // адрес не совпал, ожидание нового пакета
        break;
      }                               // если dataByte.7 = 0, то
      rxSta = WAIT_CMD;              // сразу переходим к приему команды
    }
    case WAIT_CMD:                      // -----> ожидание приема команды
    {
      if( dataByte & 0x80 )           // проверка бита 7 данных
      {
        rxSta = WAIT_FEND;           // если бит 7 не равен нулю,
        command = cmd_err;            // то ошибка
        break;
      }
      rxCmd = dataByte;              // сохранение команды
      doCrc8( dataByte, &rxCrc );    // обновление CRC
      rxSta = WAIT_NBT;              // переходим к приему количества байт
      break;
    }
    case WAIT_NBT:                      // -----> ожидание приема количества байт
    {
      if( dataByte > frame )          // если количество байт > FRAME,
      {
        rxSta = WAIT_FEND;
        command = cmd_err;            // то ошибка
        break;
      }
      rxNbt = dataByte;
      doCrc8( dataByte, &rxCrc );    // обновление CRC
      rxPtr = 0;                     // обнуляем указатель данных
      rxSta = WAIT_DATA;             // переходим к приему данных
      break;
    }
    case WAIT_DATA:                     // -----> ожидание приема данных
    {
      if( rxPtr < rxNbt )           // если не все данные приняты,
      {
        rxDat[rxPtr++] = dataByte;  // то сохранение байта данных,
        doCrc8( dataByte, &rxCrc );  // обновление CRC
        break;
      }

      //SerialUSB.print("Crc ->0x"); SerialUSB.println( rxCrc, HEX );


      if(dataByte != rxCrc)          // если приняты все данные, то проверка CRC
      {
        rxSta = WAIT_FEND;           // если CRC не совпадает,
        command = cmd_err;            // то ошибка
        break;
      }
      rxSta = WAIT_FEND;             // прием пакета завершен,
      command = rxCmd;               // загрузка команды на выполнение
      break;
    }
  }
}


  //char dataByte = Serial1.read();    // считывает все!!
/*   надо:
int readBytes(* buf, len)
    Считывает байты, поступающие на последовательный порт, и записывает их в буфер. 
    Прекращает работу после приема заданного количества байтов или в случае тайм-аута. 
    Возвращает количество принятых байтов. Тайм-аут задается функцией setTimeout().

setTimeout(long time)
    Задает время тайм-аута для функции readBytes(). 
    Время time указывается в мс, по умолчанию оно равно 1000 мс.
*/

// bool commandRead()
// {  // // Буфер пуст, но не опустел в результате обработки
//   // if( !Serial1.available() ) 
//   // {
//   //   return false;
//   // }

//   // // Если буфер не опустел - продолжить обработку
//   // while ( Serial1.available() )
//   // {
    
    
    
//   //   /* code */
//   // }

//   return true;
// }




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
      dataByte = txCmd & 0x7F;
      txSta = send_nbt;
      break;
    }
  case send_nbt:                          // -----> передача количества байт
    {
      dataByte = txNbt;
      txSta = send_data;
      txPtr = 0;                          // обнуление указателя данных для передачи
      break;
    }
  case send_data:                         // -----> передача данных
    {
      if(txPtr < txNbt)
        dataByte = txDat[ txPtr++ ];  
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
  txNbt = n;                      // количество байт
  txDat[0] = err;                 // код ошибки
  txCmd = command;                // команда
  wakeStartWrite();               // инициализация передачи
  command = cmd_nop;              // команда обработана
}