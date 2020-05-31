#include "stm32f4xx.h"

#define PRE_MASK     0b11111111000000000000000000000000	// Cars behind stop line
#define PAST_MASK    0b00000000111000001111111100000000	// Cars past stop line, except for the 1 car before the light
#define INTER_MASK   0b00000000001000000000000000000000 // Car right before the light
#define LIGHT_MASK   0b00000000000111000000000000000000	// Lights (Green, Yellow, Red)
#define CAR_MASK (PRE_MASK | PAST_MASK)			    	// All cars

#define LIGHT_GREEN  0b00000000000001000000000000000000	// Green light
#define LIGHT_YELLOW 0b00000000000010000000000000000000	// Yellow light
#define LIGHT_RED    0b00000000000100000000000000000000	// Red light

typedef enum { FALSE, TRUE } bool;

uint32_t moveAllCars(bool newCar, uint32_t boardState);
uint32_t moveSomeCars(bool newCar, uint32_t boardState);
uint32_t changeLightColor(uint32_t nextLightMask, uint32_t boardState);
