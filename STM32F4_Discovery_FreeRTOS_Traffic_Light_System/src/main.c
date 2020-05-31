/*
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.

This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.

This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.

Here is a description of the project's functionality:

The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.

The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html

The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html

An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html

The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.

The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().

The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
*/

/* Standard includes. */
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>

/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

/* custom includes */
#include "helpers.h"
#include "middleware.h"
#include <inttypes.h>
#include <stdio.h>

/*-----------------------------------------------------------*/
/* Priorities at which the tasks are created.  The event semaphore task is
given the maximum priority of ( configMAX_PRIORITIES - 1 ) to ensure it runs as
soon as the semaphore is given. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					( 1 )

// Constants
#define MAX_POT 4095
#define GREEN_BASE_PERIOD 10000
#define YELLOW_PERIOD 2000
#define RED_BASE_PERIOD 5000

// Custom Tasks
static void Traffic_Flow_Adjustment_Task(void *pvParameters);
static void Traffic_Generator_Task(void *pvParameters);
static void Traffic_Light_State_Task(void *pvParameters);
static void System_Display_Task(void *pvParameters);

// Initialize hardware
static void prvSetupHardware( void );

/* The queue used by the queue send and queue receive tasks. */
static xQueueHandle xQueue = NULL;
static xQueueHandle xPotQueue = NULL;

/*-----------------------------------------------------------*/

int main(void)
{
	// Configure hardware
	prvSetupHardware();

	/* Create queue for pot value and current board state */
	xQueue = xQueueCreate( 	mainQUEUE_LENGTH,	sizeof( uint32_t ) );
	xPotQueue = xQueueCreate(mainQUEUE_LENGTH, sizeof( uint16_t ) );

	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( xQueue, "MainQueue" );
	vQueueAddToRegistry( xPotQueue, "PotQueue" );

	uint16_t potValue = getPotValue();
	u_int32_t defaultBoardState = (0x00000000 | LIGHT_GREEN);
	xQueueSend( xPotQueue, &potValue, 0);
	xQueueSend( xQueue, &defaultBoardState, 0);

	xTaskCreate(Traffic_Flow_Adjustment_Task, "FlowAdjustment", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY, NULL );
	xTaskCreate(Traffic_Generator_Task,	"TrafficCreator", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY, NULL );
	xTaskCreate(Traffic_Light_State_Task, "TrafficLight", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY, NULL );
	xTaskCreate(System_Display_Task, "DisplayBoard", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY, NULL );

	/* Start the tasks and timers. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details.  http://www.freertos.org/a00111.html */
	for( ;; );
}
/*-----------------------------------------------------------*/

static void Traffic_Flow_Adjustment_Task(void *pvParameters) {
	u_int16_t ulReceivedValue;
	while (1) {
		vTaskDelay(250);

		// Remove old pot value from potQueue
		xQueueReceive( xPotQueue, &ulReceivedValue, portMAX_DELAY );

		// Get new pot value
		uint16_t potValue = getPotValue();

		// Put new pot value into queue
		xQueueSend( xPotQueue, &potValue, 0);
	}
}

static void Traffic_Generator_Task(void *pvParameters) {
	// Initialize timer
	time_t t;
	u_int16_t pot;
	u_int32_t boardState;
	srand((unsigned) time(&t));

	while (1) {
		vTaskDelay(500);

		// Get value from boardState Queue and PotQueue
		xQueueReceive( xQueue, &boardState, portMAX_DELAY );
		xQueueReceive( xPotQueue, &pot, portMAX_DELAY );

		bool newCar = FALSE;
		if(pot > 4000){ // max resistance, so 20% chance of new car
			if(rand() % 5 == 0) {
				newCar = TRUE;
			}
		} else if (pot < 200) { // min resistance, so 90% chance of new car
			if(rand()/32767 > 32767*0.1 ) {
				newCar = TRUE;
			}
		} else {
			if( rand() % (MAX_POT) > (pot) ) { // random probability of new car, but lower resistance -> higher chance of new car
				newCar = TRUE;
			}
		}

		printf("resistance: %u\n", pot);

		// move all cars on green light
		if ((boardState & LIGHT_MASK) == LIGHT_GREEN) {
			boardState = moveAllCars(newCar, boardState);

		// move some cars on red/yellow light
		} else {
			boardState = moveSomeCars(newCar, boardState);
		}

		// Push value back to queues
		xQueueSend( xPotQueue, &pot, 0);
		xQueueSend( xQueue, &boardState, 0);
	}
}


static void changeTrafficLight( xTimerHandle xTimer )
{
	u_int32_t boardState;

	// get board state from xQueue
	xQueueReceive( xQueue, &boardState, portMAX_DELAY );
	u_int32_t lightColor = boardState & LIGHT_MASK;

	if (lightColor == LIGHT_GREEN) {
		boardState = changeLightColor(LIGHT_YELLOW, boardState);
	} else if (lightColor == LIGHT_YELLOW) {
		boardState = changeLightColor(LIGHT_RED, boardState);
	} else if (lightColor == LIGHT_RED) {
		boardState = changeLightColor(LIGHT_GREEN, boardState);
	}

	// Push value back to Queue
	xQueueSend( xQueue, &boardState, 0);
}

// if pot = 4096, green-period = 5s, red-period = 10s
// if pot = 0, green-period = 10s, red-period = 5s
float getGreenPeriod(int potValue) {
	float x = 1.220703125;
	float period = (GREEN_BASE_PERIOD - (float)potValue * x) / portTICK_RATE_MS;
	return period;
}

float getRedPeriod(int potValue) {
	float period = ((float)RED_BASE_PERIOD + ((float)potValue/(float)MAX_POT) * RED_BASE_PERIOD ) / portTICK_RATE_MS;
	return period;
}

static void Traffic_Light_State_Task(void *pvParameters) {

	// Create software timer variable
	xTimerHandle xTrafficLightSoftwareTimer = NULL;

	u_int16_t pot;
	uint32_t boardState;
	xQueueReceive( xPotQueue, &pot, portMAX_DELAY );

	// Initialize timer
	xTrafficLightSoftwareTimer = xTimerCreate("TrafficLightTimer", getGreenPeriod(pot), pdFALSE, ( void * ) 0, changeTrafficLight);

	xTimerStart( xTrafficLightSoftwareTimer, 0 );

	xQueueSend( xPotQueue, &pot, 0);

	while(1) {
		vTaskDelay(250);

		// Get queue values
		xQueueReceive( xQueue, &boardState, portMAX_DELAY );
		xQueueReceive( xPotQueue, &pot, portMAX_DELAY );

		u_int32_t lightColor = boardState & LIGHT_MASK;

		// Restart timer when complete
	    if( xTimerIsTimerActive( xTrafficLightSoftwareTimer ) == pdFALSE ){

	    	// Green Light period ranges from 5000ms to 10000ms, proportional to pot value
			if (lightColor == LIGHT_GREEN) {

				float newPeriod = getGreenPeriod(pot);
				xTimerChangePeriod(xTrafficLightSoftwareTimer, newPeriod, 0);

			// Red Light period ranges from 5000ms to 10000ms, proportional to pot value
			} else if (lightColor == LIGHT_RED) {

				float newPeriod = getRedPeriod(pot);
				xTimerChangePeriod(xTrafficLightSoftwareTimer, newPeriod, 0);

			} else if (lightColor == LIGHT_YELLOW) {
		    	xTimerChangePeriod(xTrafficLightSoftwareTimer, YELLOW_PERIOD / portTICK_RATE_MS, 0);
			}
	    }

		// Return queue values
		xQueueSend( xPotQueue, &pot, 0);
		xQueueSend( xQueue, &boardState, 0);
	}
}

static void System_Display_Task(void *pvParameters) {
	uint32_t ulReceivedValue;

	while(1) {
		vTaskDelay(100);

		xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );

		writeBoard(ulReceivedValue);

		xQueueSend( xQueue, &ulReceivedValue, 0);
	}
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software 
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	// Setup Port A as input to read potentiometer value ?
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_PortA_InitStructure;
	GPIO_PortA_InitStructure.GPIO_Pin =  GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_PortA_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_PortA_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_PortA_InitStructure);

	// Setup Port B as Output
	// For writing to shift registers
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_PortB_InitStructure;
	GPIO_PortB_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1; // PB1 = Clock, PB0 = A input
	GPIO_PortB_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_PortB_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB,&GPIO_PortB_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);  // Enable clock for ADC Ports
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); // Enable GPIO pin to read analog input

	GPIO_InitTypeDef GPIO_initStructre;
	GPIO_initStructre.GPIO_Pin = GPIO_Pin_1;       //The channel 10 is connected to PC1
	GPIO_initStructre.GPIO_Mode = GPIO_Mode_AN;    //GPIO Pin as analog Mode
	GPIO_initStructre.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC,&GPIO_initStructre);           // GPIO Initialization

	// Analog Mode Deinitialization, Configuration and Initialization
	ADC_DeInit();
	ADC_InitTypeDef ADC_InitStruct;
	ADC_InitStruct.ADC_ScanConvMode=DISABLE;
	ADC_InitStruct.ADC_Resolution=ADC_Resolution_12b;
	ADC_InitStruct.ADC_ContinuousConvMode=ENABLE;
	ADC_InitStruct.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStruct.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfConversion=1;
	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_Cmd(ADC1, ENABLE);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_144Cycles);


	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}
