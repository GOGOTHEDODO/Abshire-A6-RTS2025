/* --------------------------------------------------------------
   Application: 06
   Release Type: final custom project
   Class: Real Time Systems - fall 2025
   Author: [Henry Abshire]
   Email: [He248516@ucf.edu]
   Company: [University of Central Florida]
   Website:
   AI Use: Please commented inline where you use(d) AI
---------------------------------------------------------------*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "ultrasonic.h"
#include "esp_sleep.h"

#define HEARTBEAT_LED_PIN 5
#define TRIP_WIRE_LED_PIN 4
#define DISABLE_BUTTON_PIN 15
#define SONIC_SENSOR_ECHO_PIN 12
#define SONIC_SENSOR_TRIG_PIN 13
#define MAX_DISTANCE_CM 400
#define BUFFER_SIZE 100
#define SUMMARY_TASK_ANALYZIE_PIN 23
#define LOG_TASK_ANALYIZE_PIN 22


float GLOBAL_DISTANCE_BUFFER[BUFFER_SIZE];
long GLOBAL_TIME_BUFFER[BUFFER_SIZE];
int pos = 0;

//sychron prematives
static SemaphoreHandle_t xDisableButtonSemaphore;
static SemaphoreHandle_t xTimeBufferMutex;
static SemaphoreHandle_t xDistanceBufferMutex;
static SemaphoreHandle_t xPosMutex;
static SemaphoreHandle_t xUARTPrintMutex;  
static bool securitySystemEnabled = true;
static TaskHandle_t sensor_task_handle;
static TaskHandle_t heartbeat_task_handle;
static TaskHandle_t summary_task_handle; 
/*
On button press call the function that suspends all tasks until the button is pressed again
*/
QueueHandle_t sensor_trigger_queue; 
    // this isr is activated when the button is pressed and calls the disable security system task
void IRAM_ATTR disable_button_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xDisableButtonSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
Prints the times of the last 10 occurances of the sensor being triggered incase the watcher missed a notification
Type: Hard, this is very important for users to see if they may have missed a trigger 
Period/Deadline: 1s 
*/
void summary_task(void *pvParameters){
    while(1){
        if (xSemaphoreTake(xDistanceBufferMutex, portMAX_DELAY)){
            if (xSemaphoreTake(xTimeBufferMutex, portMAX_DELAY)){
                //if got here we have permission for both buffers
                if(xSemaphoreTake(xUARTPrintMutex, portMAX_DELAY)){
                    gpio_set_level(SUMMARY_TASK_ANALYZIE_PIN,1);
                    printf("The last 10 triggers were at: \n");
                    int currentPos = -1;
                    // load global pos to local var
                    if (xSemaphoreTake(xPosMutex, portMAX_DELAY)){
                        currentPos = (pos - 10);
                        xSemaphoreGive(xPosMutex);
                    }
                    if (currentPos < 0){
                        currentPos += 100;
                    }
                    for (int i = 0; i < 10; i++){
                        printf("time: %ld\n", GLOBAL_TIME_BUFFER[currentPos]);
                        currentPos = ((currentPos + 1) % 100);
                    }
                    xSemaphoreGive(xUARTPrintMutex); 
                }
                xSemaphoreGive(xTimeBufferMutex);
            }
            xSemaphoreGive(xDistanceBufferMutex); 
        }
        gpio_set_level(SUMMARY_TASK_ANALYZIE_PIN,0); 
        vTaskDelay(pdMS_TO_TICKS(1000)); //once a second
    }
}

/*
Logs the time and the distance where the sensor triggered someone passing thorugh in global buffers
Type Hard, its important that the data be logged so the summary task can correctly dispaly security 
Period/Deadline: 1ms from queue recieve
*/
void log_trip_task(void *pvParameters){
    float distance_recieved = -1.0;
    while(1){
        if(xQueueReceive(sensor_trigger_queue, &distance_recieved, portMAX_DELAY) == pdTRUE){
            gpio_set_level(LOG_TASK_ANALYIZE_PIN,1); 
            int tempTime = pdTICKS_TO_MS(xTaskGetTickCount()); //get time here incase the mutex takes time to get
            if(xSemaphoreTake(xDistanceBufferMutex, portMAX_DELAY)){
                if(xSemaphoreTake(xTimeBufferMutex, portMAX_DELAY)){
                    if(xSemaphoreTake(xPosMutex,portMAX_DELAY)){
                        GLOBAL_DISTANCE_BUFFER[pos] = distance_recieved*100;
                        GLOBAL_TIME_BUFFER[pos] = tempTime;
                        //notify that alarm has been triggered
                        if(xSemaphoreTake(xUARTPrintMutex, portMAX_DELAY)){
                            printf("Alarm Triggered: %f cm at time %ld ms stored in pos %d \n", GLOBAL_DISTANCE_BUFFER[pos], GLOBAL_TIME_BUFFER[pos], pos);
                            xSemaphoreGive(xUARTPrintMutex);
                        }
                        gpio_set_level(TRIP_WIRE_LED_PIN, 1);
                        pos = (pos + 1) % BUFFER_SIZE;
                        xSemaphoreGive(xPosMutex); 
                    }
                    
                    xSemaphoreGive(xTimeBufferMutex); 
                }//end of time mutex 
                xSemaphoreGive(xDistanceBufferMutex);
            }//end of distance mutex
        //Timing Debug Code
        //printf("Process ended at time: %ld \n", pdTICKS_TO_MS(xTaskGetTickCount()));
        gpio_set_level(LOG_TASK_ANALYIZE_PIN,0); 
        }//end of queue recieved

    }//end of while loop
   
}

/*
Flashes a green led at 1hz to show the system is on and currently sensing 
Type: Soft, while the heartbeat is important to show the system is active it doesn't perform any system critical funcitons
period/deadline: 1 second, 
*/
void heartbeat_task(void *pvParameters){
    while(1){
        gpio_set_level(HEARTBEAT_LED_PIN, 0); 
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(HEARTBEAT_LED_PIN,1); 
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
}

/*
This task sets up the ultrasonic sensor and then reads from the sensor, if below threshold, uses a queue to send distance data to log task
Type:Hard, if a deadline is missed someone may be able to sneak by the security system.
Period/daedline: reads the sensor once every 20ms
Function obtained then modified from: 
Bilal. “HC-SR04 Ultrasonic Sensor with Esp32 Esp-IDF.” HC-SR04 Ultrasonic Sensor with ESP32 ESP-IDF, ESP32 ESP-IDF, 
11 Oct. 2022, esp32tutorials.com/hc-sr04-ultrasonic-sensor-esp32-esp-idf/.
*/
void ultrasonic_sensor_task(void *pvParameters)
{
    //create and initalize ultrasonic sensor object with correct pins
    ultrasonic_sensor_t sensor = {.trigger_pin = SONIC_SENSOR_TRIG_PIN,.echo_pin = SONIC_SENSOR_ECHO_PIN};
    ultrasonic_init(&sensor);

    const TickType_t periodTicks = pdMS_TO_TICKS(20); // e.g. 200 ms period
    TickType_t lastWakeTime = xTaskGetTickCount();     // Initialize last wake time
    while (true){
        //timing testing code
        //printf("sensor start time: %ld\n", pdTICKS_TO_MS(xTaskGetTickCount())); 
        float distance =-1;
        //read from sensor
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
        //Error Handling code:
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
            case ESP_ERR_ULTRASONIC_PING:
                printf("Cannot ping (device is in invalid state)\n");
                break;
            case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                printf("Ping timeout (no device found)\n");
                break;
            case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                printf("Echo timeout (i.e. distance too big)\n");
                break;
            default:
                printf("%s\n", esp_err_to_name(res));
            }
        }
        else{
             //something is too close, trigger the sensor
            if(distance*100 < 30.0){
                //test debug code
                //printf("Distance is below 30 send distance via queue in pos %d\n", pos);
                xQueueSend(sensor_trigger_queue, &distance, portMAX_DELAY);
            }
        //timing test code
        //printf("Distance: %0.04f cm at time: %ld \n", distance * 100, pdTICKS_TO_MS(xTaskGetTickCount()));
        vTaskDelayUntil(&lastWakeTime, periodTicks);
        }
    }//end of while loop
}

/*
This task waits for the button to be pressed then checks if the system is current off or on, 
if off: awakes the sensor and heart beat tasks, then goes to sleep
is on: sleeps the heart beat and sensor tasks, then turns off both leds 
Deadline: 100ms, should suspend the system fast enough that people don't trigger the alarm system after turning it off
*/
void toggle_security_system_task(void *pvParameters){
    while (1){
        //printf("waiting for semaphore \n");
        if (xSemaphoreTake(xDisableButtonSemaphore, portMAX_DELAY) == pdTRUE){
            if (securitySystemEnabled){
                securitySystemEnabled = false;
                //suspend all tasks to save on power consumption
                //couldn't find a way to put esp to sleep
                gpio_set_level(TRIP_WIRE_LED_PIN, 0); //disable the alarm led
                vTaskSuspend(heartbeat_task_handle);
                gpio_set_level(HEARTBEAT_LED_PIN, 0); 
                vTaskSuspend(sensor_task_handle);
                vTaskSuspend(summary_task_handle);
                printf("disabling system"); 
            }
            else{
                securitySystemEnabled = true;
                vTaskResume(heartbeat_task_handle);
                vTaskResume(sensor_task_handle); 
                vTaskResume(summary_task_handle); 
                if(xSemaphoreTake(xUARTPrintMutex, portMAX_DELAY)){
                    printf("enabling security system \n");
                    xSemaphoreGive(xUARTPrintMutex); 
                }
            }//end of enable security else
        }//of of button isr
    }//end of while loop
} // end of toggle_security_system_task function

void app_main() {
    // Initialize GPIOs
    //analyizer pins
    gpio_reset_pin(SUMMARY_TASK_ANALYZIE_PIN);
    gpio_reset_pin(LOG_TASK_ANALYIZE_PIN); 
    gpio_set_direction(SUMMARY_TASK_ANALYZIE_PIN, GPIO_MODE_OUTPUT); 
    gpio_set_direction(LOG_TASK_ANALYIZE_PIN, GPIO_MODE_OUTPUT); 

    //heartBeat LED
    gpio_reset_pin(HEARTBEAT_LED_PIN);
    gpio_set_direction(HEARTBEAT_LED_PIN, GPIO_MODE_OUTPUT);

    // Trip Wire LED
    gpio_reset_pin(TRIP_WIRE_LED_PIN);
    gpio_set_direction(TRIP_WIRE_LED_PIN, GPIO_MODE_OUTPUT);

    //Button 
    gpio_reset_pin(DISABLE_BUTTON_PIN);
    gpio_set_direction(DISABLE_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(DISABLE_BUTTON_PIN);
    gpio_set_intr_type(DISABLE_BUTTON_PIN, GPIO_INTR_NEGEDGE); 
    //test pins
    gpio_set_level(HEARTBEAT_LED_PIN, 1);
    gpio_set_level(TRIP_WIRE_LED_PIN, 0);

    //Semaphores
    xDisableButtonSemaphore = xSemaphoreCreateBinary();
    //Mutexes
    xDistanceBufferMutex = xSemaphoreCreateMutex();
    xTimeBufferMutex = xSemaphoreCreateMutex();
    xPosMutex = xSemaphoreCreateMutex();
    xUARTPrintMutex = xSemaphoreCreateMutex();
    //Queue
    sensor_trigger_queue = xQueueCreate(100, sizeof(float));

    //isr things
    gpio_install_isr_service(0);// no flags
    gpio_isr_handler_add(DISABLE_BUTTON_PIN, disable_button_isr_handler, NULL); // binds the interupt function to the pin

    //initalize buffers:
    for(int i=0; i<BUFFER_SIZE; i++){
        GLOBAL_DISTANCE_BUFFER[i] = 0;
        GLOBAL_TIME_BUFFER[i] =0;
    }

    //esp_sleep_enable_ext0_wakeup(DISABLE_BUTTON_PIN, 0);
    xTaskCreate(heartbeat_task, "heartbeat task", 2048, NULL, 1, &heartbeat_task_handle);
    xTaskCreate(toggle_security_system_task, "toggle security",2048,NULL,5,NULL); 
    xTaskCreate(ultrasonic_sensor_task, "ultrasonic_sensor_task", configMINIMAL_STACK_SIZE * 3, NULL, 4, &sensor_task_handle);
    xTaskCreate(log_trip_task, "log sensor trigger task", 2048, NULL, 2, NULL);
    xTaskCreate(summary_task, "Summary Task", 2048, NULL, 3, &summary_task_handle);
}