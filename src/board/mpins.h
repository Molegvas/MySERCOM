#ifndef _MPINS_H_
#define _MPINS_H_
/*
* https://github.com/BLavery/SAMD21-M0-Mini
* select the board: Tools/Board = Arduino SAMD / "Arduino Zero (Native USB Port)".
* Double-click the board reset button. Upload. See if a LED blinks.
* explain a bit:
* 1. The Arduino bootloader uses double click on reset button to enter flash upload mode. Single
*    reset enters normal running mode. (Boot0/1 jumpers, like STM32 uses, are not used on the SAMD.)
* 2. The serial/bossac mode of USB upload is included in the Arduino install.
* 3. The regular user LED is missing. So we choose one of the TX/RX LEDs. For the moment, 
*    the Arduino sketch is NOT otherwise driving these as TX/RX indicators (see above), 
*    so we can re-employ them.
* 4. But what pin or GPIO number? For this board, reference the pins as 0-26 or A0-A5. Not D25, 
*    nor PB03 or PB3.
*
* https://github.com/arduino/ArduinoCore-samd/pull/374/files
* additional analog inputs: A6-A11 #374 
*/

//#include "SAMD_AnalogCorrection.h"
#include "stdint.h"

namespace MPins
{

//#include <Arduino.h>

//                q920_name      arduino_pin  M0-MINI   SAMD21_name
constexpr uint8_t uart_rx           =    0;    // D0   PA11/UART-RX             
constexpr uint8_t uart_tx           =    1;    // D1   PA10/UART-TX  
constexpr uint8_t off_pin           =    2;    // D4   PA14
constexpr uint8_t d3_pin            =    3;    // D3   PA09
constexpr uint8_t rtu_pin           =   50;    // D3   additional analog as PA09
constexpr uint8_t rtu_mux           =    9;    // D3   additional analog as PA09
constexpr uint8_t d2_pin            =    4;    // D2   PA08 nu
constexpr uint8_t din_pin           =   51;    // D2   additional analog as PA08
constexpr uint8_t din_mux           =    8;    // D2   additional analog as PA08
constexpr uint8_t ch_pin            =    5;    // D5   PA15
constexpr uint8_t d8_pin            =    8;    // D8   PA06 nu
constexpr uint8_t d9_pin            =    9;    // D9   PA07 nu

constexpr uint8_t shunt_plus_pin    =   46;    // D8   additional analog as PA06
constexpr uint8_t shunt_minus_pin   =   47;    // D9   additional analog as PA07
constexpr uint8_t shunt_plus_mux    =    6;    // D8   additional analog as PA06
constexpr uint8_t shunt_minus_mux   =    7;    // D9   additional analog as PA07

constexpr uint8_t ss_pin            =   10;    // D10  PA18
constexpr uint8_t mosi_pin          =   11;    // D11  PA16
constexpr uint8_t miso_pin          =   12;    // D12  PA19
constexpr uint8_t sck_pin           =   13;    // D13  PA17

constexpr uint8_t dac_pin           =   14;    // A0   PA02
constexpr uint8_t bat_plus_pin      =   17;    // A3   PA04
constexpr uint8_t bat_minus_pin     =   18;    // A4   PA05
constexpr uint8_t bat_plus_mux      =    4;    // A3   PA04
constexpr uint8_t bat_minus_mux     =    5;    // A4   PA05
constexpr uint8_t icsp_miso         =   22;    //  1   PA12/MISO ICSP  !! На схеме 2
constexpr uint8_t icsp_mosi         =   23;    //  4   PB10/MOSI ICSP              3
constexpr uint8_t icsp_sck          =   24;    //  3   PB11/SCK  ICSP              4
constexpr uint8_t led_rx            =   25;    // no   PB03/LED1 (LED_BUILTIN, LED_RX)
constexpr uint8_t led_tx            =   26;    // no   PA27/LED2 (LED_TX)
constexpr uint8_t foff_pin          =   21;    // D21  PA23

// Not used
//    6    // D6   PA20       // d21
//    7    // D7   PA21       // d21
//   15    // A1   PB08       // d21
//   16    // A2   PB09       // d21
//   19    // A5   PB02       // d21
//   20    // D20  PA22/SDA
//   21    // D21  PA23/SCL

};

#endif // !_MPINS_H_