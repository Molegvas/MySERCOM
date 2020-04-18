/*
  https://github.com/mike-matera/FastPID/tree/master/examples/VoltageRegulator
  FastPID: A fast 32-bit fixed-point PID controller for Arduino

*/

#include <Arduino.h>
#include "board/mpins.h"
#include "power/power_reg.h"
#include <FastPID.h>
#include "stdint.h"

float Kp =  0.1;
float Ki =  0.5;
float Kd =  0.0;
float Hz = 10.0; 
int output_bits = 10; // Set analog out resolution to max, 10-bits
bool output_signed = false; 

FastPID myPID( Kp, Ki, Kd, Hz, output_bits, output_signed );

uint16_t setpoint = 512;
uint16_t feedback = 511;

void initPid()
{
  analogWriteResolution( output_bits );
}

void doPid()
{
  uint32_t before;
  uint32_t after; 
  before = micros(); 
  uint16_t output = myPID.step(setpoint, feedback); 
  after = micros(); 

  analogWrite( MPins::dac_pin, output );

#ifdef DEBUG_PID
  SerialUSB.print("runtime: "); 
  SerialUSB.print(after - before); 
  SerialUSB.print(" sp: "); 
  SerialUSB.print(setpoint); 
  SerialUSB.print(" fb: "); 
  SerialUSB.print(feedback); 
  SerialUSB.print(" out: "); 
  SerialUSB.println(output); 
#endif

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
