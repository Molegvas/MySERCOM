/*

*/

#ifndef _POWER_REG_H_
#define _POWER_REG_H_

#include "stdint.h"

void portsInit();
void initPid();
void doPid();

bool setCoefficients( float kp, float ki, float kd, float hz ); 
bool setOutputConfig( int bits, bool sign );
bool setOutputRange( int16_t min, int16_t max );
bool configure( float kp, float ki, float kd, float hz, int bits, bool sign );
void clear();

void switchFoff(bool);        // Включение/отключение коммутатора
void converterOff(bool);      // Включение/отключение преобразователя
void chargerCh(bool);         // Переключение напряжения с выхода DAC

  // Управление дискретными выходами
void doSwitchFoff();          // 0x60
void doConverterOff();        // 0x61
void doSetVoltage();          // 0x62 задать напряжение и включить
void doSetCurrent();          // 0x63 задать ток и включить
void doChargerCh();           // 0x64 ch_pin = 5  D5  PA15  on/off
void doSetDiscurrent();       // 0x65 задать ток и включить


#endif  //!_POWER_REG_H_