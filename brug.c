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

unsigned long bridgeState = 0; // 0 -> bridge is closed, 1 -> bridge is open

void bridge(void)
{

    mcuMainQueue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
    mcuToBridgeInterface = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
    mcuToDetectionSensor = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
    mcuToTrafficLights = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
    mcuToBridgeBarriers = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
    mcuToMotor = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));

    xTaskCreate(mcu, "mcu", configMINIMAL_STACK_SIZE, NULL,mainQUEUE_SEND_TASK_PRIORITY, NULL);

//    xTaskCreate(bridgeInterface, /* The function that implements the task. */
//                "bridgeInbridgeInterfaceToMcuterface", /* The text name assigned to the task - for debug only as it is not used by the kernel. */
//                configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
//                NULL, /* The parameter passed to the task - not used in this case. */
//                mainQUEUE_RECEIVE_TASK_PRIORITY, /* The priority assigned to the task. */
//                NULL); /* The task handle is not required, so NULL is passed. */

    xTaskCreate(detectionSensor, "detectionSensor", configMINIMAL_STACK_SIZE, NULL,mainQUEUE_SEND_TASK_PRIORITY, NULL);
    xTaskCreate(trafficLights, "trafficLights", configMINIMAL_STACK_SIZE, NULL,
    mainQUEUE_SEND_TASK_PRIORITY,
                NULL);
    //        xTaskCreate(brdigeBarriers, "brdigeBarriers", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL);
    //        xTaskCreate(motor, "motor", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL);

    vTaskStartScheduler();

    for (;;)

        ;
}

void mcu(void *pvParameters) // sends request to open the bridge when button is pressed
{
    TickType_t xNextWakeTime;
    unsigned long ulReceivedValue;
    const unsigned long ulCheckMovement = 100UL, ulBlinkTrafficLights = 100UL, ulBringDownBridgeBarriers = 100UL, ulTurnMotorRight = 100UL;
    const unsigned long ulCheckBoatMovement = 200UL, ulTurnOffTrafficLights = 200UL, ulBringUpBridgeBarriers = 200UL, ulTurnMotorLeft = 200UL;

    (void) pvParameters;

    xNextWakeTime = xTaskGetTickCount();

    for (;;)
    {
        if (!(P1IN & BIT1))
        {
            P1OUT |= BIT0;
            P1DIR |= BIT0;

//            GPIO_setOutputHighOnPin( GPIO_PORT_P1, GPIO_PIN0 );
//            vTaskDelay(1000);
            xQueueSend(mcuMainQueue, 100UL, 0U);
        }

        xQueueReceive(mcuMainQueue, &ulReceivedValue, portMAX_DELAY);

        switch (bridgeState)
        {
        case 0: // case for bridge is closed
            switch (ulReceivedValue)
            {
            case 100:
                ulReceivedValue=0;
                P1OUT |= BIT0;
                P1DIR |= BIT0;
//                GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 );

//                xQueueSend(mcuToDetectionSensor, &ulCheckMovement, 0U); // Check if movement is detected
                break;
            case 200:
                ulReceivedValue=0;

                xQueueSend(mcuToTrafficLights, &ulBlinkTrafficLights, 0U); // Turn on (blink) traffic lights
                break;
            case 300:
                ulReceivedValue=0;

                xQueueSend(mcuToBridgeBarriers, &ulBringDownBridgeBarriers, 0U); // Bring down bridge barriers
                break;
            case 400:
                ulReceivedValue=0;

                xQueueSend(mcuToMotor, &ulTurnMotorRight, 0U); // Turn motor right
                break;
            case 500: // Bridge is open
                ulReceivedValue=0;

                bridgeState = 1;
                break;
            }
            break;

        case 1: // case for bridge is open

            switch (ulReceivedValue)
            {
            case 100:
                ulReceivedValue=0;

                xQueueSend(mcuToDetectionSensor, &ulCheckBoatMovement, 0U); // Check if boat movement is detected
                break;
            case 200:
                ulReceivedValue=0;

                xQueueSend(mcuToMotor, &ulTurnMotorLeft, 0U); // Turn motor left
                break;
            case 300:
                ulReceivedValue=0;

                xQueueSend(mcuToBridgeBarriers, &ulBringUpBridgeBarriers, 0U); // Bring up bridge barriers
                break;
            case 400:
                ulReceivedValue=0;

                xQueueSend(mcuToTrafficLights, &ulTurnOffTrafficLights, 0U); // Turn off traffic lights
                break;
            case 500: // Bridge is closed
                ulReceivedValue=0;
                bridgeState = 0;
                break;
            }
            break;
        }
    }
}

void bridgeInterface(void *pvParameters) // sends request to open the bridge when button is pressed
{
    TickType_t xNextWakeTime;
    const unsigned long openBridge = 100UL;
    (void) pvParameters;

    xNextWakeTime = xTaskGetTickCount();
    for (;;)
    {
        // GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == 0
        if (!(P1IN & BIT1))
        {
            P1OUT |= BIT0;
            P1DIR |= BIT0;

//            GPIO_setOutputHighOnPin( GPIO_PORT_P1, GPIO_PIN0 );
//            vTaskDelay(1000);
            xQueueSend(mcuMainQueue, &openBridge, 0U);
        }
        }
    }

void detectionSensor(void *pvParameters) // Checks if users are detected
{
    TickType_t xNextWakeTime;
    unsigned long ulReceivedValue;
    const unsigned long ulCheckMovement = 100UL;
    const unsigned long ulCheckBoatMovement = 200UL , ulNoMovementDetected = 200UL, ulNoBoatDetected = 200UL;

    xNextWakeTime = xTaskGetTickCount();

    for (;;)
    {
        xQueueReceive(mcuToDetectionSensor, &ulReceivedValue, portMAX_DELAY);

        switch (ulReceivedValue)
        {
        case ulCheckMovement:

            if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5) == 0) // Check movement on bridge
            {
                xQueueSend(mcuMainQueue, &ulNoMovementDetected, 0U);
            }
            else
            {
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0 );
            }
            break;
        case ulCheckBoatMovement:
            if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == 0) // Check boat movement
            {
                xQueueSend(mcuMainQueue, &ulNoBoatDetected, 0U);
            }
            else
            {
                GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN6 );
            }
            break;
        }

    }
}

void trafficLights(void *pvParameters)
{
    unsigned long ulReceivedValue;
    const unsigned long ulBlinkTrafficLights = 100UL;
    const unsigned long ulTurnOffTrafficLights = 200UL;
    const unsigned long ulTrafficLightsAreBlinking = 300UL;
    const unsigned long ulTrafficLightsOff= 500UL;
    xQueueReceive(mcuToTrafficLights, &ulReceivedValue, portMAX_DELAY);

    switch (ulReceivedValue)
           {
           case ulBlinkTrafficLights:
                   GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
                   xQueueSend(mcuMainQueue, &ulTrafficLightsAreBlinking, 0U);
               break;
           case ulTurnOffTrafficLights:
                   GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN6 );
                   xQueueSend(mcuMainQueue, &ulTrafficLightsOff, 0U);
               break;
           }
}
void bridgeBarriers(void *pvParameters)
{
    unsigned long ulReceivedValue;
        const unsigned long ulBringDownBridgeBarriers = 100UL;
        const unsigned long ulBringUpBridgeBarriers = 200UL;
        const unsigned long ulBridgeBarriersAreDown = 400UL;
        const unsigned long ulBridgeBarriersAreUp= 400UL;

        xQueueReceive(mcuToTrafficLights, &ulReceivedValue, portMAX_DELAY);

        switch (ulReceivedValue)
               {
               case ulBringDownBridgeBarriers:
                   while(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5))
                       GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
                       vTaskDelay(5000);
                       GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);
                       GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);

                       xQueueSend(mcuMainQueue, &ulBridgeBarriersAreDown, 0U);
                   break;
               case ulBringUpBridgeBarriers:
                       vTaskDelay(5000);
                       GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN4);
                       xQueueSend(mcuMainQueue, &ulBridgeBarriersAreUp, 0U);
                   break;
               }
}

void motor(void *pvParameters)
{
}
