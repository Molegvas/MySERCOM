/*


*/

#include "commands.h"
#include "adc_commands.h"
#include "pid_commands.h"
#include "board/mpins.h"
#include "power/power_reg.h"
#include "wake/wake.h"
#include "stdint.h"
#include <Arduino.h>

// Имя устройства
static constexpr char Info[] = {"Q920dn Rev0.0\n\0"};   //

  // state1
bool _switchStatus          = true;  // коммутатор ( foff_pin 21 D21 PA23 )
bool _converterStatus       = false;  // преобразователь
bool _currentControlStatus  = false;  // регулирование по току
bool _voltageControlStatus  = false;  // регулирование по напряжению
bool _chargeStatus          = false;  // заряд
bool _dischargeStatus       = false;  // разряд
bool _pauseStatus           = false;  // пауза
bool _reserve1Status        = false;  // резерв 1

  // state2
bool _overHeatingStatus     = false;  // перегрев
bool _overloadStatus        = false;  // перегрузка
bool _powerLimitationStatus = false;  // ограничение мощности
bool _reversePolarityStatus = false;  // обратная полярность
bool _shortCircuitStatus    = false;  // короткое замыкание
bool _calibrationStatus     = false;  // калибровка
bool _upgradeStatus         = false;  // обновление
bool _reserve2Status        = false;  // резерв 2


    // ПИД-регулятор
static constexpr uint8_t cmd_pid_configure        = 0x40; // kp, ki, kd, hz, bits, sign 
static constexpr uint8_t cmd_pid_coefficients     = 0x41; // kp, ki, kd, hz 
static constexpr uint8_t cmd_pid_output_range     = 0x42; // min, max
static constexpr uint8_t cmd_pid_output_config    = 0x43; // bits, sign
static constexpr uint8_t cmd_pid_clear            = 0x44; 

static constexpr uint8_t cmd_pid_stop_go          = 0x45; // стоп-пауза-пуск 
static constexpr uint8_t cmd_pid_test             = 0x46; // mode, setpoint, min, max

    // АЦП - настройки
static constexpr uint8_t cmd_adc_read_probes          = 0x50; // Read all probes
static constexpr uint8_t cmd_adc_config               = 0x51; // параметры измерителя
static constexpr uint8_t cmd_adc_config52             = 0x52; // параметры АЦП
static constexpr uint8_t cmd_adc_ref_compensation     = 0x53; // reference buffer offset compensation
static constexpr uint8_t cmd_adc_read_mv              = 0x54; // Read probes, mV
static constexpr uint8_t cmd_set_adc_bat              = 0x55;
static constexpr uint8_t cmd_set_adc_shunt            = 0x56;
static constexpr uint8_t cmd_set_adc_rtu              = 0x57;
static constexpr uint8_t cmd_offset_compensation      = 0x58;
static constexpr uint8_t cmd_offset_gain_compensation = 0x59;

        // Команды управления процессами
static constexpr uint8_t cmd_switch_foff              = 0x60; // foff_pin = 21  D21 PA23
static constexpr uint8_t cmd_converter_off            = 0x61; // off_pin  =  2  D4  PA14
static constexpr uint8_t cmd_charger_ch               = 0x62; // ch_pin   =  5  D5  PA15



    // ЦАП - настройки
    // https://stackoverflow.com/questions/53542591/using-external-vref-for-samd21-dac
enum dac_reference 
{
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


uint8_t cmd = cmd_nop;

uint8_t state1 = 0b00000000;
uint8_t state2 = 0b00000000;


void doInfo();
void doEcho();
void doErr();


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
        // Команды управления процессами
      case cmd_switch_foff:               doSwitchFoff();             break;  // 0x60
      case cmd_converter_off:             doConverterOff();           break;  // 0x61
      case cmd_charger_ch:                doChargerCh();              break;  // 0x62 
      

        // Команды работы с измерителями
      case cmd_adc_read_probes:           doReadProbes();             break;  // 0x50
      case cmd_adc_config:                doAdcConfig();              break;  // 0x51
      case cmd_adc_config52:              doAdcConfig52();            break;  // 0x52
      case cmd_adc_ref_compensation:      doAdcRefCompensation();     break;  // 0x53
      case cmd_adc_read_mv:               doReadValues();             break;  // 0x54
      case cmd_set_adc_bat:               doAdcBat();                 break;  // 0x55
      case cmd_set_adc_shunt:             doAdcShunt();               break;  // 0x56
      case cmd_set_adc_rtu:               doAdcRtu();                 break;  // 0x57
      case cmd_offset_compensation:       doOffsetCompensation();     break;  // 0x58
      case cmd_offset_gain_compensation:  doOffsetGainCompensation(); break;  // 0x59
      //case cmd_:                        do();                       break;  // 0x5a

        // Команды работы с регуляторами
      case cmd_pid_configure:             doPidConfigure();           break;  // 0x40
      case cmd_pid_coefficients:          doPidCoefficients();        break;  // 0x41
      case cmd_pid_output_range:          doPidOutputRange();         break;  // 0x42
      case cmd_pid_output_config:         doPidOutputConfig();        break;  // 0x43
      case cmd_pid_clear:                 doPidClear();               break;  // 0x44
      case cmd_pid_test:                  doPidTest();                break;  // 0x46

        // Команды управления
      //case cmd_:                        do();                       break;  // 0x20


        // Команды универсальные
      case cmd_err:                       doErr();                    break;  // 0x01
      case cmd_echo:                      doEcho();                   break;  // 0x02
      case cmd_info:                      doInfo();                   break;  // 0x03

      default: 
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
  
  txReplay( i, txDat[0] );        // Искусственный прием, об ошибках не сообщается
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

// Формирование регистра состояния 1
void doState1()
{
  _switchStatus         ? state1 |= 0b10000000 : state1 &= 0b01111111; 
  _converterStatus      ? state1 |= 0b01000000 : state1 &= 0b10111111; 
  _currentControlStatus ? state1 |= 0b00100000 : state1 &= 0b11011111; 
  _voltageControlStatus ? state1 |= 0b00010000 : state1 &= 0b11101111; 
  _chargeStatus         ? state1 |= 0b00001000 : state1 &= 0b11110111; 
  _dischargeStatus      ? state1 |= 0b00000100 : state1 &= 0b11111011; 
  _pauseStatus          ? state1 |= 0b00000010 : state1 &= 0b11111101; 
  _reserve1Status       ? state1 |= 0b00000001 : state1 &= 0b11111110;

  // Непрерывное подтверждение состояния управляющих выходов
  switchFoff(_switchStatus);
  converterOff(_converterStatus);
  chargerCh(_chargeStatus);
}

// Формирование регистра состояния 2 
void doState2()
{
  _overHeatingStatus     ? state2 |= 0b10000000 : state2 &= 0b01111111; 
  _overloadStatus        ? state2 |= 0b01000000 : state2 &= 0b10111111; 
  _powerLimitationStatus ? state2 |= 0b00100000 : state2 &= 0b11011111; 
  _reversePolarityStatus ? state2 |= 0b00010000 : state2 &= 0b11101111; 
  _shortCircuitStatus    ? state2 |= 0b00001000 : state2 &= 0b11110111; 
  _calibrationStatus     ? state2 |= 0b00000100 : state2 &= 0b11111011; 
  _upgradeStatus         ? state2 |= 0b00000010 : state2 &= 0b11111101; 
  _reserve2Status        ? state2 |= 0b00000001 : state2 &= 0b11111110; 
}