/*
  https://github.com/mike-matera/FastPID/tree/master/examples/VoltageRegulator
  FastPID: A fast 32-bit fixed-point PID controller for Arduino

*/

#include <Arduino.h>
#include "board/mpins.h"
#include "power/power_reg.h"
#include <FastPID.h>
#include "stdint.h"

float kp =  0.1;
float ki =  0.5;
float kd =  0.0;
float hz = 10.0; 
int output_bits = 10; // Set analog out resolution to max, 10-bits
bool output_signed = false; 

uint16_t output     = 0x0000;
bool     pidStatus  = false;    // false - PID-регулятор отключен 
uint8_t  pidMode    = 0;        // 0-1-2 - тестирование: задать напряжение, ток заряда или ток разряда
// pidReference

FastPID myPID( kp, ki, kd, hz, output_bits, output_signed );

uint16_t setpoint = 512;
uint16_t feedback = 511;

void initPid()
{
  analogWriteResolution( output_bits );
}

void doPid()
{
  if( pidStatus )
  {
    uint32_t before;
    uint32_t after; 
    before = micros(); 
    //uint16_t output = myPID.step(setpoint, feedback); 
    output = myPID.step(setpoint, feedback); 
    after = micros();
 
    analogWrite( MPins::dac_pin, output );

    #ifdef DEBUG_PID
    // SerialUSB.print("runtime: "); 
    // SerialUSB.print(after - before); 
    // SerialUSB.print(" sp: "); 
    // SerialUSB.print(setpoint); 
    // SerialUSB.print(" fb: "); 
    // SerialUSB.print(feedback); 
    // SerialUSB.print(" out: "); 
    // SerialUSB.println(output); 
    #endif
  }
  else
  {
    // output in millivolts
output = 250;         // Test
    //analogReference(AR_DEFAULT);
    //DAC->CTRLB.reg = 0x40;  // ???use AVCC as the reference - DAC Off
    analogWrite( MPins::dac_pin, ( output * 0x3ff /4  ) / 3300 ); //1200 = AREF
                // найти откуда /4 ?
    #ifdef DEBUG_PID
      SerialUSB.print(" out: "); 
      SerialUSB.println(output); 
    #endif
  }
  


}

bool setCoefficients( float kp, float ki, float kd, float hz )
{
  return myPID.setCoefficients( kp, ki, kd, hz);
} 

bool setOutputConfig( int bits, bool sign )
{
  return myPID.setOutputConfig( bits, sign );
}

bool setOutputRange( int16_t min, int16_t max )
{
  return myPID.setOutputRange( min, max );
}

bool configure( float kp, float ki, float kd, float hz, int bits, bool sign )
{
  return myPID.configure( kp, ki, kd, hz, bits, sign );
}

void clear()
{
  return myPID.clear();
}
