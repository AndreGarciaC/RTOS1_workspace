/*
 * taskMonitor.c
 *
 *  Created on: 23 abr. 2022
 *      Author: Andrea García
 */

// ------ Includes -------------------------------------------------
/* Project includes. */
#include "main.h"
#include "cmsis_os.h"

/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Demo includes. */
#include "supportingFunctions.h"

/* Application includes. */
#include "taskAll.h"

// ------ Private constants ----------------------------------------

/* Define the strings that will be passed in as the Supporting Functions parameters.
 * These are defined const and off the stack to ensure they remain valid when the
 * tasks are executing. */
const char *pcTextForTaskMonitor    			= "  ==> Task    Monitor - Running\r\n";

const char *pcTextForTaskMonitor_AddQueue     	= "  ==> Task    Monitor - Add to xQueueVehicleDateTime.    ==>\r\n\n";
const char *pcTextForTaskMonitor_GetQueue     	= "  ==> Task    Monitor - Receive from xQueueVehicle    ==>\r\n\n";
// ------ Private variables ----------------------------------------

/* Task a Flag */
uint32_t lTaskMonitorFlag;
/* Structures */
typedef struct
{
    char plate[6];				//config
    xTaskHandle * taskOn;	//variables
} tData;

typedef struct
{
    tData hdr;
    char _DateTime[];
} tDataextended;
// ------ Public functions prototypes ------------------------------

/* Task A thread */
void vTaskMonitor(void *pvParameters);


// ------ Public functions -----------------------------------------

/*------------------------------------------------------------------*/
/* Task A thread */
void vTaskMonitor( void *pvParameters )
{
	/* Print out the name of this task. */
	vPrintString( pcTextForTaskMonitor );

	/* Structures	*/
	tData carData;
	tDataextended extendedcarData;
	while( 1 )
	{
		/* Toggle LD3 state */
		HAL_GPIO_TogglePin( LD3_GPIO_Port, LD3_Pin );
		if(xQueueReceive( xQueueVehicle, &carData,  portMAX_DELAY ))
		{
			vPrintString( pcTextForTaskMonitor_GetQueue );
			extendedcarData.hdr = carData;
			extendedcarData._DateTime = defaultDateTime;
			xQueueSend( xQueueVehicleDateTime, &extendedcarData,  portMAX_DELAY );
			vPrintString(pcTextForTaskMonitor_AddQueue);
		}
	}
}

/*------------------------------------------------------------------*-
  ---- END OF FILE -------------------------------------------------
-*------------------------------------------------------------------*/
