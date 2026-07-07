#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

/* TI Drivers */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Power.h>

/* TI Driver Porting Layer (DPL) */
#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/dpl/SemaphoreP.h>

/* Board Header (Replace with your actual SysConfig generated header if different) */
#include "ti_drivers_config.h" 

// ============================================================================
// Global Variables & State
// ============================================================================

/* RTOS Objects */
ClockP_Struct debounceClockStruct;
ClockP_Struct holdClockStruct;
SemaphoreP_Struct buttonSemStruct;
SemaphoreP_Handle buttonSemHandle;

/* State Tracking */
uint32_t pressTime = 0;
volatile bool isButtonPressed = false; 
volatile bool pendingShutdown = false; 

/* User Application Variables */
volatile bool systemModeActive = false; 

/* Event Flags for the Task */
typedef enum {
    EVENT_NONE,
    EVENT_SHORT_PRESS,
    EVENT_SHUTDOWN
} ButtonEvent_t;

volatile ButtonEvent_t pendingButtonEvent = EVENT_NONE;

// ============================================================================
// Function Prototypes
// ============================================================================
void powerOffSensors(void);
void shortPressHandler(void);
void gpioButtonFxn(uint_least8_t index);
void debounceCallback(uintptr_t arg);
void holdCallback(uintptr_t arg);
void initButtonLogic(void);
void *buttonAppTask(void *arg0);
void buttonTask_create(void);

// ============================================================================
// Hardware / Application Handlers
// ============================================================================

void powerOffSensors(void) {
    // TODO: Send I2C/SPI sleep commands or toggle a load switch to cut sensor power.
    // Because this runs in a Task context, blocking TI drivers are 100% safe here.
}

void shortPressHandler(void) {
    // Modify global variable or execute quick logic
    // systemModeActive = !systemModeActive;
    GPIO_toggle(CONFIG_LED_0_GPIO);
}

// ============================================================================
// RTOS Callbacks (Software Interrupts - NO BLOCKING CALLS)
// ============================================================================

// 3-Second Hold Callback
void holdCallback(uintptr_t arg) {
    // 3 seconds elapsed while the button is held down. 
    // Flag the pending shutdown, but wait for the user to let go before sleeping.
    GPIO_write(CONFIG_LED_0_GPIO, CONFIG_GPIO_LED_ON);
    pendingShutdown = true; 
}

// 20ms Debounce Callback
void debounceCallback(uintptr_t arg) {
    // Read actual state after mechanical bouncing has settled
    uint32_t pinState = GPIO_read(CONFIG_GPIO_BUTTON_0_INPUT);

    if (pinState == 0) { 
        // State is LOW -> Solid PRESS
        if (!isButtonPressed) {
            isButtonPressed = true;
            pendingShutdown = false; 
            pressTime = ClockP_getSystemTicks();

            // Start 3-second hold timer
            ClockP_start(&holdClockStruct);

            // Switch interrupt to catch the user letting go (Rising edge)
            GPIO_setConfig(CONFIG_GPIO_BUTTON_0_INPUT, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);
        }
    } 
    else { 
        // State is HIGH -> Solid RELEASE (or a false bounce)
        if (isButtonPressed) {
            isButtonPressed = false;
            
            // Stop the hold timer in case they let go early
            ClockP_stop(&holdClockStruct);

            if (pendingShutdown) {
                pendingShutdown = false;
                
                // The user held it for 3s and let go. The bounce is over.
                // 1. Arm the CC2340R5 hardware latch to wake up on the next LOW state
                GPIO_setConfig(CONFIG_GPIO_BUTTON_0_INPUT, GPIO_CFG_IN_PU | GPIO_CFG_SHUTDOWN_WAKE_LOW);
                
                // 2. Signal the Task to execute shutdown safely
                pendingButtonEvent = EVENT_SHUTDOWN;
                SemaphoreP_post(buttonSemHandle);
            }
            else {
                // Short Press Logic
                uint32_t releaseTime = ClockP_getSystemTicks();
                uint32_t durationTicks = releaseTime - pressTime;
                
                // Prevent integer division zeroing out the duration
                uint32_t tickPeriodUs = ClockP_getSystemTickPeriod();
                uint32_t durationMs = (durationTicks * tickPeriodUs) / 1000; 

                // Validate short press (between 50ms and 1000ms)
                if (durationMs >= 50 && durationMs < 1000) {
                    pendingButtonEvent = EVENT_SHORT_PRESS;
                    SemaphoreP_post(buttonSemHandle);
                }

                // Switch interrupt back to catch the next normal press (Falling edge)
                GPIO_setConfig(CONFIG_GPIO_BUTTON_0_INPUT, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
            }
        }
    }

    // Clear any hardware interrupt flags triggered during the 20ms blind spot
    GPIO_clearInt(CONFIG_GPIO_BUTTON_0_INPUT);
    
    // Re-enable the hardware interrupt
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0_INPUT);
}

// ============================================================================
// Hardware Interrupts (Keep these lightning fast)
// ============================================================================

void gpioButtonFxn(uint_least8_t index) {
    // Disable the interrupt to create the "blind spot" and start debounce timer.
    GPIO_disableInt(CONFIG_GPIO_BUTTON_0_INPUT);
    ClockP_start(&debounceClockStruct);
}

// ============================================================================
// Task Setup & Execution
// ============================================================================

void initButtonLogic(void) {
    // 1. Initialize Semaphore
    buttonSemHandle = SemaphoreP_constructBinary(&buttonSemStruct, 0);

    // 2. Initialize Clocks
    ClockP_Params clockParams;
    ClockP_Params_init(&clockParams);
    clockParams.period = 0; 
    clockParams.startFlag = false;
    clockParams.arg = (uintptr_t)NULL;

    uint32_t tickPeriodUs = ClockP_getSystemTickPeriod();

    // 20ms Debounce Clock
    ClockP_construct(&debounceClockStruct, debounceCallback,
                     20000 / tickPeriodUs, &clockParams);

    // 3-Second Hold Clock
    ClockP_construct(&holdClockStruct, holdCallback,
                     3000000 / tickPeriodUs, &clockParams);

    // 3. Initialize GPIO state flags
    isButtonPressed = false;
    pendingShutdown = false;
    pendingButtonEvent = EVENT_NONE;

    // 4. Arm GPIO
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0_INPUT, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0_INPUT, gpioButtonFxn);
    GPIO_clearInt(CONFIG_GPIO_BUTTON_0_INPUT);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0_INPUT);
}

// The core infinite loop for this thread
void *buttonAppTask(void *arg0) {
    
    initButtonLogic();

    while (1) {
        // Sleep completely until a callback posts the semaphore (0% CPU usage)
        SemaphoreP_pend(buttonSemHandle, SemaphoreP_WAIT_FOREVER);

        // Wake up and process the requested event
        if (pendingButtonEvent == EVENT_SHORT_PRESS) {
            pendingButtonEvent = EVENT_NONE;
            shortPressHandler();
        }
        else if (pendingButtonEvent == EVENT_SHUTDOWN) {
            pendingButtonEvent = EVENT_NONE;
            
            // Execute shutdown sequence
            powerOffSensors(); 
            Power_shutdown(0, 0); 
            
            // The CC2340R5 is now completely off. 
            // Code execution will permanently halt here.
            // The next falling edge on BTN1 will trigger a full device reboot.
        }
    }
    
    return NULL;
}

// Call this from your main() file to spawn the task
void buttonTask_create(void) {
    pthread_t thread;
    pthread_attr_t attrs;
    struct sched_param priParam;

    pthread_attr_init(&attrs);
    
    // Set priority lower than the BLE stack
    priParam.sched_priority = 2; 
    pthread_attr_setschedparam(&attrs, &priParam);
    pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&attrs, 1024); // 1KB stack

    pthread_create(&thread, &attrs, buttonAppTask, NULL);
}