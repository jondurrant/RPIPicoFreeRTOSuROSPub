/*
 * main.cpp
 *
 * Demo application to Publish msg to Topics from multiple Tasks
 *
 *  Created on: 9 Feb 2023
 *      Author: jondurrant
 */


#include <stdio.h>
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "BlinkAgent.h"
#include "URosBridge.h"
#include "PubEntities.h"

extern"C"{
#include "pico/stdio/driver.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdio_uart.h"

}

//Standard Task priority
#define TASK_PRIORITY		( tskIDLE_PRIORITY + 1UL )

const uint LED_PIN = 14;
const uint PULSE_LED = 15;





void runTimeStats(   ){
	TaskStatus_t *pxTaskStatusArray;
	volatile UBaseType_t uxArraySize, x;
	unsigned long ulTotalRunTime;


   // Get number of takss
   uxArraySize = uxTaskGetNumberOfTasks();
   printf("Number of tasks %d\n", uxArraySize);

   //Allocate a TaskStatus_t structure for each task.
   pxTaskStatusArray = (TaskStatus_t *)pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

   if( pxTaskStatusArray != NULL ){
      // Generate raw status information about each task.
      uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
                                 uxArraySize,
                                 &ulTotalRunTime );

	 // Print stats
	 for( x = 0; x < uxArraySize; x++ )
	 {
		 printf("Task: %d \t cPri:%d \t bPri:%d \t hw:%d \t%s\n",
				pxTaskStatusArray[ x ].xTaskNumber ,
				pxTaskStatusArray[ x ].uxCurrentPriority ,
				pxTaskStatusArray[ x ].uxBasePriority ,
				pxTaskStatusArray[ x ].usStackHighWaterMark ,
				pxTaskStatusArray[ x ].pcTaskName
				);
	 }


      // Free array
      vPortFree( pxTaskStatusArray );
   } else {
	   printf("Failed to allocate space for stats\n");
   }

   //Get heap allocation information
   HeapStats_t heapStats;
   vPortGetHeapStats(&heapStats);
   printf("HEAP avl: %d, blocks %d, alloc: %d, free: %d\n",
		   heapStats.xAvailableHeapSpaceInBytes,
		   heapStats.xNumberOfFreeBlocks,
		   heapStats.xNumberOfSuccessfulAllocations,
		   heapStats.xNumberOfSuccessfulFrees
		   );
}


/***
 * Boot up of the Tasks
 * @param params
 */
void bootTask(void *params){
	//Start blink
	BlinkAgent blink(13);
	blink.start("Blink", TASK_PRIORITY);

	//Start up a uROS Bridge and a publisher
	uRosBridge *bridge = uRosBridge::getInstance();
	PubEntities pubEnts;

	bridge->setuRosEntities(&pubEnts);
	bridge->setLed(LED_PIN);
	bridge->start("Bridge",  TASK_PRIORITY+1);
	pubEnts.start("Pub Task", TASK_PRIORITY);


	for (;;){
		//runTimeStats();
		vTaskDelay(30000);
	}


}


/***
 * Launch the tasks and scheduler
 */
void vLaunch( void) {

	TaskHandle_t task;
    xTaskCreate(bootTask, "Boot Task", 1024, NULL, TASK_PRIORITY, &task);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

int main(){
	stdio_init_all();
	sleep_ms(2000);
	stdio_filter_driver(&stdio_uart);
	printf("Start\n");

	vLaunch();

	return 0;
}

