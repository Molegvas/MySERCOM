/*


*/

#ifndef _WAKE_H_
#define _WAKE_H_

#include "stdint.h"

// Константы протокола
  static constexpr uint8_t fend     = 0xC0; // Frame END
  static constexpr uint8_t fesc     = 0xDB; // Frame ESCape
  static constexpr uint8_t tfend    = 0xDC; // Transposed Frame END
  static constexpr uint8_t tfesc    = 0xDD; // Transposed Frame ESCape
  static constexpr uint8_t crc_init = 0xDE; // Initial CRC value
  static constexpr uint8_t frame    = 255;  // Максимальная длина пакета (полезных данных)

    // Коды ошибок:
    static constexpr uint8_t err_no = 0x00;   // no error
    static constexpr uint8_t err_tx = 0x01;   // Rx/Tx error
    static constexpr uint8_t err_bu = 0x02;   // device busy error
    static constexpr uint8_t err_re = 0x03;   // device not ready error
    static constexpr uint8_t err_pa = 0x04;   // parameters value error
    static constexpr uint8_t err_nr = 0x05;   // no replay
    static constexpr uint8_t err_nc = 0x06;   // no carrier




void txReplay(char n, char err);  // передача ответа на команду

#endif //!_WAKE_H_