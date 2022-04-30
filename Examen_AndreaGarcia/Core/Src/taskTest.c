/* Copyright 2020, Juan Manuel Cruz.
 * All rights reserved.
 *
 * This file is part of Project => FZ
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */


/*--------------------------------------------------------------------*-

    taskTest.c (Released 2022-04)

--------------------------------------------------------------------

    task file for FreeRTOS - Event Driven System (EDS) - Project for
    STM32F429ZI_NUCLEO_144.

    See readme.txt for project information.

-*--------------------------------------------------------------------*/


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

/* Events to excite tasks */
typedef enum eTaskTest{ Qty, Error, Entry, Exit } eTaskTest_t;


#define TEST_X ( 4 )

#if( TEST_X == 0 )
/* Array of events to excite tasks */
const eTaskTest_t eTaskTestArray[] = {_Qty,_Qty, Error, Exit+1, Exit+2 };
#endif

#if( TEST_X == 1 )
/* Array of events to excite tasks */
const eTaskTest_t eTaskTestArray[] = {_Qty,_Qty, Entry, Exit };
#endif

#if( TEST_X == 2 )
/* Array of events to excite tasks */
const eTaskTest_t eTaskTestArray[] = {_Qty, Entry, Entry, Exit, Exit };
#endif

#if( TEST_X == 3 )
/* Array of events to excite tasks */
const eTaskTest_t eTaskTestArray[] = {_Qty, Entry, Entry, Entry, Entry, Exit, Exit, Exit, Exit };
#endif

#if( TEST_X == 4 )
/* Array of events to excite tasks */
const eTaskTest_t eTaskTestArray[] = {_Qty, Entry, Entry, Entry, Entry, Entry, Entry, Exit, Exit, Exit, Exit, Exit, Exit };
#endif

#if( TEST_X == 5 )
/* Array of events to excite tasks */
const eTaskTest_t eTaskTestArray[] = {_Qty, Entry, Entry, Exit };
#endif

/* Define the strings that will be passed in as the Supporting Functions parameters.
 * These are defined const and off the stack to ensure they remain valid when the
 * tasks are executing. */
const char *pcTextForTaskTest						= "  <=> Task Test - Running\r\n\n";

const char *pcTextForTaskTest_TEST_X				= "  <=> Task Test - TEST_X : ";
const char *pcTextForTaskTest_eTaskTestArrayIndex	= "  <=> Task Test - eTaskTestArray Index :";

const char *pcTextForTaskTest_SignalEntry 			= "  <=> Task Test - Signal: Entry  <=>\r\n";
const char *pcTextForTaskTest_SignalExit			= "  ==> Task Test - Signal: Exit   <=>\r\n";
const char *pcTextForTaskTest_SignalError			= "  <=> Task Test - Signal: Error  <=>\r\n";
const char *pcTextForTaskTest_Wait5000mS			= "  <=> Task Test - Wait:   5000mS <=>\r\n\n";


// ------ Public functions prototypes ------------------------------

/* Task Test thread */
void vTaskTest(void *pvParameters);


// ------ Public functions -----------------------------------------

/*------------------------------------------------------------------*/
/* Task Test thread */
void vTaskTest( void *pvParameters )
{
	uint32_t i = TEST_X;
	portTickType xLastWakeTime;
	char *pcTaskParameters;

	/* The string to print out is passed in via the parameter.  Cast this to a
	 * character pointer. */
	pcTaskParameters = (char *) pvParameters;

	/* Print out the name, parameters and TEST_X of this task. */
	vPrintString( pcTextForTaskTest );
	vPrintString( pcTaskParameters );
	vPrintStringAndNumber( pcTextForTaskTest_TEST_X, i);


	/* The xLastWakeTime variable needs to be initialized with the current tick
	 * count.  Note that this is the only time we access this variable. From this
	 * point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	 * API function. */
	xLastWakeTime = xTaskGetTickCount();

	while( 1 )
	{
		/* Scanning the array of events to excite tasks */
		for ( i = 0; i < (sizeof(eTaskTestArray)/sizeof(eTaskTest_t)); i++ )
		{
		    /* Toggle LD1 state */
			HAL_GPIO_TogglePin( LD1_GPIO_Port, LD1_Pin );

			vPrintTwoStrings( pcTaskGetName( vTaskTestHandle ), "- Running" );
			vPrintStringAndNumber( pcTextForTaskTest_eTaskTestArrayIndex, i);

			switch( eTaskTestArray[i] ) {

		    	case Entry:

				    /* 'Give' the semaphore to unblock the task A. */
		    		vPrintString( pcTextForTaskTest_SignalEntry );
		    		xSemaphoreGive( xBinarySemaphoreEntry);

					xSemaphoreGive( semEntry[currentEntry].sem );
					break;

		    	case Exit:

				    /* 'Give' the semaphore to unblock the task B. */
		    		vPrintString( pcTextForTaskTest_SignalExit );
		    		xSemaphoreGive( xBinarySemaphoreExit );
					break;

		    	case Error:
		    	default:

		    		vPrintString( pcTextForTaskTest_SignalError );
		    		break;
		    }
		    /* About a 5000 mS delay here */
			/* We want this task to execute exactly every 5000 milliseconds.  As per
			 * the vTaskDelay() function, time is measured in ticks, and the
			 * portTICK_RATE_MS constant is used to convert this to milliseconds.
			 * xLastWakeTime is automatically updated within vTaskDelayUntil() so does not
			 * have to be updated by this task code. */
		    vPrintString( pcTextForTaskTest_Wait5000mS );
    		vTaskDelayUntil( &xLastWakeTime, (5000 / portTICK_RATE_MS) );
		}
	}
}

/*------------------------------------------------------------------*-
  ---- END OF FILE -------------------------------------------------
-*------------------------------------------------------------------*/
