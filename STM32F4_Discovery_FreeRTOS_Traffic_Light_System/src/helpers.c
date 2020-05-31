#include "stm32f4xx.h"
#include <math.h>

#define PRE_MASK     0b11111111000000000000000000000000	// Cars behind stop line
#define PAST_MASK    0b00000000111000001111111100000000	// Cars past stop line, except for the 1 car before the light
#define INTER_MASK   0b00000000001000000000000000000000 // Car right before the light
#define LIGHT_MASK   0b00000000000111000000000000000000	// Lights (Green, Yellow, Red)
#define CAR_MASK (PRE_MASK | PAST_MASK)			    	// All cars

#define LIGHT_GREEN  0b00000000000001000000000000000000	// Green light
#define LIGHT_YELLOW 0b00000000000010000000000000000000	// Yellow light
#define LIGHT_RED    0b00000000000100000000000000000000	// Red light

typedef enum { FALSE, TRUE } bool;

uint32_t moveAllCars(bool newCar, uint32_t boardState) {

	uint32_t newState = 0x00000000
			| (CAR_MASK & (boardState >> 1))     // Shift all cars
			| ((INTER_MASK & boardState) >> 6)   // Shift the one car right before the light, so it goes past the lights
			| (LIGHT_MASK & boardState);		 // Insert light state

	if (newCar) {
		newState = newState | 0x80000000; // add new car
	}

	return newState;

}

// Shift all cars past the intersection
// Shift some cars before the intersection
// Insert new car if newCar == TRUE
uint32_t moveSomeCars(bool newCar, uint32_t boardState) {
	// Find rightmost 0 before light
	uint32_t stopLoc = (boardState | ~PRE_MASK);

	// Find least significant 0
	stopLoc = ~stopLoc & (stopLoc+1);

	// Get value of position
	stopLoc = (int)log2(stopLoc) & 0xFF;
	if (stopLoc == 0) {
		stopLoc = 32;
	}

	// Mask for just the cars before the light with room ahead
	uint32_t dynMask = 0x00000000;
	for (int i=0; i < 32-stopLoc; i++) {
		dynMask = ((dynMask >> 1) | 0x80000000);
	}

	// Build new boardState
	uint32_t newBoardState = 0x00000000
			| ((dynMask & boardState) >> 1) 				// Shifts unblocked vehicles
			| (~dynMask & PRE_MASK & boardState)			// Places blocked vehicles
			| (PAST_MASK & ((PAST_MASK & boardState) >> 1)) // Shifts vehicles that are past the stop line
			| (LIGHT_MASK & boardState); 				    // Places light state

	if (newCar) {
		newBoardState = newBoardState | 0x80000000; // add new car
	}

	return newBoardState;
}


/*
 * inputs: new light color, current board state
 * output: new board state with different light
 */
uint32_t changeLightColor(uint32_t nextLightMask, uint32_t boardState) {

	return 0x00000000
			| (~LIGHT_MASK & boardState) // Board state without lights
			| nextLightMask;				 // Lights
}
