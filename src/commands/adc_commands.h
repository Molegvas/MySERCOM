#ifndef _ADC_COMMANDS_H_
#define _ADC_COMMANDS_H_
/*


*/

#include "stdint.h"


void doReadProbes();
void doAdcConfig();     // параметры измерителя
              //void doAdcConfig52();
void doAdcConfig52();   // параметры АЦП
void doReadValues();    // Чтение данных в физических величинах ( милливольты, миллиамперы, миллиградусы )


#endif  //!_ADC_COMMANDS_H_ 