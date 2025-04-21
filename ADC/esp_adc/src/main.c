#include <stdio.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc_cal.h"

#define ADC_PIN ADC1_CHANNEL_0
#define LED_PIN GPIO_NUM_2
#define READ_INTERVAL 500
#define BLINK_INTERVAL 50

void app_main() {
    esp_adc_cal_characteristics_t adc_chars;
    // set 12-bit width
    adc1_config_width(ADC_WIDTH_BIT_12);
    // using adc1 0, 0-3.2V
    adc1_config_channel_atten(ADC_PIN, ADC_ATTEN_DB_12);
    // Ensures accurate ADC readings
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 0, &adc_chars);

    // LED
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    
    while(1){
        // read adc
        int adc_reading_raw = adc1_get_raw(ADC_PIN);
        float voltage = esp_adc_cal_raw_to_voltage(adc_reading_raw, &adc_chars) / 1000.0; // Convert mV to V

        printf("\r\nADC RAW Value: %d\n", adc_reading_raw);
        printf("Voltage: %.3f V\n", voltage);
        
        int light = (voltage > 1.6)? 1: 0;
        int TIME = 0;
        int led_state = 0;
        if (light){
            while(1){
                led_state = !led_state;
                gpio_set_level(LED_PIN, led_state);
                vTaskDelay(pdMS_TO_TICKS(BLINK_INTERVAL));
                TIME += BLINK_INTERVAL;
                if (TIME == READ_INTERVAL) break;
            }
        }else{
            vTaskDelay(pdMS_TO_TICKS(READ_INTERVAL));
        }
    }
}