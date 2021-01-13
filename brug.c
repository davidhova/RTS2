#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "partest.h"
#include "driverlib.h"
#include "gpio.h"

#define mainQUEUE_RECEIVE_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define mainQUEUE_SEND_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define mainQUEUE_SEND_FREQUENCY_MS (pdMS_TO_TICKS(200))
#define mainQUEUE_LENGTH (1)

TaskHandle_t myTask;

void bridge(void);
void mcu(void *pvParameters);
void bridgeInterface(void *pvParameters);
void detectionSensor(void *pvParameters);
void trafficLights(void *pvParameters);
void bridgeBarriers(void *pvParameters);
void motor(void *pvParameters);

void bridge(void);

static QueueHandle_t mcuMainQueue = NULL;
static QueueHandle_t mcuToBridgeInterface = NULL;
static QueueHandle_t mcuToDetectionSensor = NULL;
static QueueHandle_t mcuToTrafficLights = NULL;
static QueueHandle_t mcuToBridgeBarriers = NULL;
static QueueHandle_t mcuToMotor = NULL;

unsigned long bridgeState =0; // 0 -> bridge is closed, 1 -> bridge is open

void bridge(void)
{
    mcuMainQueue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
    

    mcuToBridgeInterface = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
    mcuToDetectionSensor = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
    mcuToTrafficLights = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
    mcuToBridgeBarriers =  xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
    mcuToMotor =  xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));



        xTaskCreate(bridgeInterface,                 /* The function that implements the task. */
                    "bridgeInbridgeInterfaceToMcuterface",               /* The text name assigned to the task - for debug only as it is not used by the kernel. */
                    configMINIMAL_STACK_SIZE,        /* The size of the stack to allocate to the task. */
                    NULL,                            /* The parameter passed to the task - not used in this case. */
                    mainQUEUE_RECEIVE_TASK_PRIORITY, /* The priority assigned to the task. */
                    NULL);                           /* The task handle is not required, so NULL is passed. */

        xTaskCreate(detectionSensor, "detectionSensor", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL);
        xTaskCreate(trafficLights, "trafficLights", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL);
        //        xTaskCreate(brdigeBarriers, "brdigeBarriers", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL);
        //        xTaskCreate(motor, "motor", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL);

        vTaskStartScheduler();


    for (;;);
}
void mcu(void *pvParameters) // sends request to open the bridge when button is pressed
{
    TickType_t xNextWakeTime;
    unsigned long ulReceivedValue;
//    const unsigned long bridgeIsOpen, bridgeIsClosed = 10UL;
    const unsigned long ulCheckMovement, ulBlinkTrafficLights, ulBringDownBridgeBarriers, ulTurnMotorRight= 100UL;
    const unsigned long ulCheckBoatMovement, ulTurnOffTrafficLights, ulBringUpBridgeBarriers, ulTurnMotorLeft= 200UL;

    (void)pvParameters;

    xNextWakeTime = xTaskGetTickCount();

    for (;;)
    {
        xQueueReceive( mcuMainQueue, &ulReceivedValue, portMAX_DELAY );
       switch (bridgeState)
       {
       case 0 : // case for bridge is closed
        switch (ulReceivedValue)
        {
        case 100:
            xQueueSend( mcuToDetectionSensor, &ulCheckMovement, 0U ); // Check if movement is detected
            break;
        case 200:
            xQueueSend( mcuToTrafficLights, &ulBlinkTrafficLights, 0U );  // Turn on (blink) traffic lights
            break;
        case 300:
            xQueueSend( mcuToBridgeBarriers, &ulBringDownBridgeBarriers, 0U ); // Bring down bridge barriers
            break;
        case 400:
            xQueueSend( mcuToMotor, &ulTurnMotorRight, 0U ); // Turn motor right
            break;
        case 500: // Bridge is open
            bridgeState=1;
            break;
        }
        break;

       case 1: // case for bridge is open

           switch (ulReceivedValue)
                   {
       case 100:
           xQueueSend( mcuToDetectionSensor, &ulCheckBoatMovement, 0U ); // Check if boat movement is detected
           break;
       case 200:
           xQueueSend( mcuToMotor, &ulTurnMotorLeft, 0U ); // Turn motor left
           break;
       case 300:
           xQueueSend( mcuToBridgeBarriers, &ulBringUpBridgeBarriers, 0U ); // Bring up bridge barriers
           break;
       case 400:
           xQueueSend( mcuToTrafficLights, &ulTurnOffTrafficLights, 0U );  // Turn off traffic lights
           break;
       case 500: // Bridge is closed
           bridgeState=0;
           break;
       }
           break;
    }
}
}



void bridgeInterface(void *pvParameters) // sends request to open the bridge when button is pressed
{
    TickType_t xNextWakeTime;
    unsigned long ulReceivedValue;

    const unsigned long openBridge = 100UL;
    (void)pvParameters;

    xNextWakeTime = xTaskGetTickCount();

    for (;;)
    {

        if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == 0){
            xQueueSend(mcuMainQueue, &openBridge, 0U);
        }

    }
}

void detectionSensor(void *pvParameters) // Checks if users are detected
{
    TickType_t xNextWakeTime;
    unsigned long ulReceivedValue;
    const unsigned long getMovementSensor = 100UL;
    const unsigned long ulMovementDetected = 200UL;
    const unsigned long ulNoMovementDetected = 300UL;

    xNextWakeTime = xTaskGetTickCount();

    for (;;)
    {
            xQueueReceive(mcuToDetectionSensor, &ulReceivedValue, portMAX_DELAY );

            if (ulReceivedValue == getMovementSensor)
            {
                if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == 0) // Check movement on bridge
                {
                    GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN6 );
                    xQueueSend(mcuMainQueue, &ulMovementDetected, 0U);

                }
            else
            {
                xQueueSend(mcuMainQueue, &ulNoMovementDetected, 0U);
                GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN6 );

            }
            }
        
    }
}

void trafficLights(void *pvParameters)
{
    unsigned long ulReceivedValue;
    const unsigned long ulExpectedValue = 200UL;
    //        const unsigned long ulValueToSend = 300UL;



        xQueueReceive(mcuToTrafficLights, &ulReceivedValue, portMAX_DELAY);

        if (ulReceivedValue == ulExpectedValue)
        {
            //                    GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 );
            //                    GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN6 );

            //                    xQueueSend(xQueue, &ulValueToSend, 0U);
        }

}

void bridgeBarriers(void *pvParameters)
{
}

void motor(void *pvParameters)
{
}
