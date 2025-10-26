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

#define HEARTBEAT_LED_PIN 5
#define TRIP_WIRE_LED_PIN 4
#define BUTTON_PIN 19
#define SONIC_SENSOR_ECHO_PIN 12
#define SONIC_SENSOR_TRIG_PIN 13
#define MAX_DISTANCE_CM 400


void ultrasonic_test(void *pvParameters)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = SONIC_SENSOR_TRIG_PIN,
        .echo_pin = SONIC_SENSOR_ECHO_PIN};

    ultrasonic_init(&sensor);

    while (true)
    {
        float distance;
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
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
        else
            printf("Distance: %0.04f cm\n", distance * 100);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main() {
    printf("Hello, World!\n");

    // Initialize GPIOs
    //heartBeat LED
    gpio_reset_pin(HEARTBEAT_LED_PIN);
    gpio_set_direction(HEARTBEAT_LED_PIN, GPIO_MODE_OUTPUT);

    // Trip Wire LED
    gpio_reset_pin(TRIP_WIRE_LED_PIN);
    gpio_set_direction(TRIP_WIRE_LED_PIN, GPIO_MODE_OUTPUT);

    //Button 
    gpio_reset_pin(BUTTON_PIN);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_PIN);
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_NEGEDGE); 
    //test pins
    gpio_set_level(HEARTBEAT_LED_PIN, 1);
    gpio_set_level(TRIP_WIRE_LED_PIN, 1);

    xTaskCreate(ultrasonic_test, "ultrasonic_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}