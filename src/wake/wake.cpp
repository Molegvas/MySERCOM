/*


*/

#include <Arduino.h>
#include "wake/wake.h"

    // Переменные - уточнить типы  
    char    RxNbt;         // принятое количество байт в пакете
    char    RxDat[frame];  // массив принятых данных
    uint8_t Command;    // код команды на выполнение

    char    TxCmd;         // команда, передаваемая в пакете
    char    TxNbt;         // количество байт данных в пакете
    char    TxDat[frame];  // массив данных для передачи







  // передача ответа на команду
void txReplay(char n, char err)
{

}