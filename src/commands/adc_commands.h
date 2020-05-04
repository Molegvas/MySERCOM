#ifndef _ADC_COMMANDS_H_
#define _ADC_COMMANDS_H_
/*


*/

#include "stdint.h"


void doReadProbes();
void doAdcConfig();
void doAdcConfig52();
void doAdcConfig53();   // probe, resolution, samples, divider
void doReadValues();    // Чтение данных в физических величинах ( милливольты, миллиамперы, миллиградусы )


#endif  //!_ADC_COMMANDS_H_ 