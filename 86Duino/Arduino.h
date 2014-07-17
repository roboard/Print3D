#ifndef Arduino_h
#define Arduino_h

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// Vortex86EX ADC
#define ADC_BASEADDR    (0xFE00UL)
#define MAX_ADC_VAL     (2047)
#define MAX_ADC_VOLTAGE (3.3)
int analogRead(int pin);

#endif