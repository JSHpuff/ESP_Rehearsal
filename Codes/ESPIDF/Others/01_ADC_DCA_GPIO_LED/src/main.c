/**
 * ESP32 WROOM 32
 * ADC Voltage Monitor
 * 
 * Monitors voltages levels through ADC and detects line breaks
 * based on voltage stability analysis.
*/

#include <stdio.h>
#include "esp_err.h"                    /* ESP Error */
#include "driver/gpio.h"                /* GPIO Setting */
#include "driver/adc.h"                 /* ADC Setting */
#include "freertos/FreeRTOS.h"          /* For [task.h] */
#include "freertos/task.h"              /* Delay Functions */

/* ======== Data Config ======== */
typedef struct{
    float values[5];
    int count;
} voltage_buffer_t;

/* ======== App Switch Config ======== */
#define ENABLE_GPIO_OUTPUT  1
#define ENABLE_GPIO_INPUT   0
#define ENABLE_LED          0
#define ENABLE_DEBUG        0
#define ENABLE_DISPLAY      1
#define ADC_PORT_SELECT     1   // 0=ADC1, 1=ADC2

/* ======== Time Config ======== */
#define SAMPLE_INTERVAL     100
#define BLINK_INTERVAL      50

/* ======== ADC ======== */
#define ADC_CHANNEL         ADC2_CHANNEL_9
#define ADC_WIDTH           ADC_WIDTH_BIT_12
#define ADC_ATTEN           ADC_ATTEN_DB_12

/* ======== GPIO ======== */
#define OUTPUT_PIN          GPIO_NUM_18
#define LED_PIN             GPIO_NUM_2

/* ======== Value Config ======== */
#define INPUT_VOLT           3.3f
#define ADC_MAX_VALUE        4095.0f
#define VOLTAGE_THRESHOLD    1.6f
#define DELTA_THRESHOLD      0.2f

/* ======== Buffer Operations ======== */
static void buffer_add(voltage_buffer_t *buffer, float value) {
    if (buffer->count < 5) {
        buffer->values[buffer->count++] = value;
    }
}

static void buffer_reset(voltage_buffer_t *buffer) {
    buffer->count = 0;
}

static void buffer_analyze(voltage_buffer_t *buffer, float *max, float *min) {
    *max = buffer->values[0];
    *min = buffer->values[0];
    
    for (int i = 1; i < buffer->count; i++) {
        if (buffer->values[i] > *max) *max = buffer->values[i];
        if (buffer->values[i] < *min) *min = buffer->values[i];
    }
}

/* ======== LED Control ======== */
#if ENABLE_LED
static void blink_led(int led_pin, int duration) {
    int time_elapsed = 0;
    int state = 0;
    
    while (time_elapsed < duration) {
        state = !state;
        gpio_set_level(led_pin, state);
        vTaskDelay(pdMS_TO_TICKS(BLINK_INTERVAL));
        time_elapsed += BLINK_INTERVAL;
    }
}
#endif

/* ======== MAIN FUNCTION ======== */
void app_main() {
    esp_err_t ret;
    voltage_buffer_t buffer = {0};
    int alert_count = 0;
    int cons_count = 0;
    float voltage = 0.0f;

    /* Initialize GPIO */
#if ENABLE_GPIO_OUTPUT
    gpio_reset_pin(OUTPUT_PIN);
    gpio_set_direction(OUTPUT_PIN, GPIO_MODE_OUTPUT);
#else
    dac_output_enable(OUTPUT_PIN);
#endif

#if ENABLE_LED
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
#endif

    /* Initialize ADC */
#if ADC_PORT_SELECT == 0
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
#else 
    ret = adc2_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
#endif

    /* Main loop */
    while(1){
        /* Set output pin high 3.3V */
#if ENABLE_GPIO_OUTPUT
        gpio_set_level(OUTPUT_PIN, 1);
#else
        dac_output_voltage(OUTPUT_PIN, 255);
#endif

        /* Reading voltage state from GPIO */
#if ENABLE_GPIO_INPUT
        int state = gpio_get_level(OUTPUT_PIN);
        printf("\r\n GPIO state: %s\n", state? "HIGH":"LOW");
#endif

        /* Read ADC value */
        int raw_reading;
#if ADC_PORT_SELECT == 0
        raw_reading = adc1_get_raw(ADC_CHANNEL);
        ret = ESP_OK;
#else
        ret = adc2_get_raw(ADC_CHANNEL, ADC_WIDTH, &raw_reading);
#endif

        if (ret == ESP_OK){
            /* Process voltage reading */
            voltage = ((raw_reading * INPUT_VOLT) / ADC_MAX_VALUE);
            buffer_add(&buffer, voltage);

#if ENABLE_DEBUG
            printf("\r\n ADC RAW: %d, Voltage: %.3f V\n", raw_reading, voltage);
#endif
            /* Analyze buffer when full */
            if (buffer.count >= 5){
                float max_val, min_val, delta;
                buffer_analyze(&buffer, &max_val, &min_val);
                buffer_reset(&buffer);
                delta = max_val - min_val;
                cons_count++;
                if (delta < DELTA_THRESHOLD) {
                    alert_count++;
                }

#if ENABLE_DEBUG || ENABLE_DISPLAY
                printf("\r\n Max: %.3f V, Min: %.3f V, Delta: %.3f V\n", 
                    max_val, min_val, delta);
#endif
                /* Line break detection logic */
                if (cons_count == 2) {
                    if (alert_count == 2) {
                        printf(" ****** LINE BREAK DETECTED! ******\n");
                        alert_count = 1;
                        cons_count = 1;
                    } else {
                        alert_count = 0;
                        cons_count = 0;
                    }
                }
            }
        }else printf(" ADC Reading ERROR!\n");
        
#if ENABLE_LED
        if (voltage > VOLTAGE_THRESHOLD) {
            blink_led(LED_PIN, SAMPLE_INTERVAL);
        } else {
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL));
        }
#else
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL));
#endif
    }
}