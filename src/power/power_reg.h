/*

*/

#ifndef _POWER_REG_H_
#define _POWER_REG_H_

#include "stdint.h"



void doPid();

bool setCoefficients( float kp, float ki, float kd, float hz ); 
bool setOutputConfig( int bits, bool sign );
bool setOutputRange( int16_t min, int16_t max );
bool configure( float kp, float ki, float kd, float hz, int bits, bool sign );


#endif  //!_POWER_REG_H_