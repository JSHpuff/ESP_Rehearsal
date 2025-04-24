/**
 * ESP32 WROOM 32
 * ADC Voltage Monitor
 * 
 * Monitors voltages levels through ADC and detects line breaks
 * based on voltage stability analysis.
 * 
 * 0.5M:
 *      linebreak - max:3.3,   min:3.202
 *      plug-in   - max:2.019, min:1.859
 * 
 * 1M:
 *      linebreak - max:2.832, min:2.660
 *      plug-in   - max:1.496, min:1.363
 *      detect 2  - max:0.310, min:0.271
 *      detect 3  - max:0.288, min:0.240
 * 
 * 2M:
 *      linebreak - max:2.416, min:2.342
 *      plug-in   - max:0.967, min:0.818
 * 
*/

#include <stdio.h>
#include "esp_err.h"                    /* ESP Error */
#include "driver/gpio.h"                /* GPIO Setting */
#include "driver/adc.h"                 /* ADC Setting */
#include "freertos/FreeRTOS.h"          /* For [task.h] */
#include "freertos/task.h"              /* Delay Functions */
#include "driver/ledc.h"                /* PWM Setting */

/* ======== Data Config ======== */
typedef struct{
    float values[5];
    int count;
} voltage_buffer_t;

/* ======== App Switch Config ======== */
#define ENABLE_GPIO_OUTPUT      1
#define ENABLE_GPIO_INPUT       0
#define ENABLE_LED              1
#define ENABLE_DEBUG_1          0
#define ENABLE_DEBUG_2          0
#define ENABLE_DISPLAY          1
#define ADC_PORT_SELECT         1   // 0=ADC1, 1=ADC2
#define ENABLE_PWM              1
#define ENABLE_PWM_FADE         0

/* ======== Time Config ======== */
#define SAMPLE_INTERVAL         100
#define SAMPLE_TIME             5
#define BLINK_INTERVAL          100
#define BLINK_INTERVAL_2        50
#define BLINK_INTERVAL_3        300

/* ======== ADC ======== */
#define ADC_CHANNEL             ADC2_CHANNEL_9
#define ADC_WIDTH               ADC_WIDTH_BIT_12
#define ADC_ATTEN               ADC_ATTEN_DB_12

/* ======== GPIO ======== */
#define OUTPUT_PIN              GPIO_NUM_18
#define LED_PIN                 GPIO_NUM_2

/* ======== Value Config ======== */
#define INPUT_VOLT              3.3f
#define ADC_MAX_VALUE           4095.0f
#define LINEBREAK_THRESHOLD     2.0f
#define DETECTED_THRESHOLD      1.2f

/* ======== PWM Config ======== */
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_GPIO        19
#define LEDC_CHANNEL            LEDC_CHANNEL_0
/** Resolution of PWM duty
 * 13-bit (2 ** 13) - 1 = 8191
 * 20-bit (2 ** 20) - 1 = 1048575
 */
#define LEDC_DUTY_RES           LEDC_TIMER_20_BIT
/** Initial duty cyclwe
 * (2 ** 13) * 50% = 4096
 * (2 ** 20) * 50% = 524288
 * (2 ** 20) * 75% = 786432
 */
#define LEDC_DUTY               (786432)              
#define LEDC_FREQUENCY          3.75                // Frequency in Hz (3.75 kHz)
#define FADE_TIME               (2000)              // Fade time in milliseconds
#define PWM_TIME                (2000)

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
static void blink_led_wirebreak(int led_pin, int duration) {
    int time_elapsed = 0;
    int state = 0;
    int count = 0;
    
    while (time_elapsed < duration) {
        count ++;
        state = !state;
        gpio_set_level(led_pin, state);
            if (count < 4){
                vTaskDelay(pdMS_TO_TICKS(BLINK_INTERVAL));
                time_elapsed += BLINK_INTERVAL;
            }else{
                count = 0;
                vTaskDelay(pdMS_TO_TICKS(BLINK_INTERVAL_3));
                time_elapsed += BLINK_INTERVAL_3;
            }
    }
}
static void blink_led_leakage(int led_pin, int duration) {
    int time_elapsed = 0;
    int state = 0;

    while (time_elapsed < duration) {
        state = !state;
        gpio_set_level(led_pin, state);
        vTaskDelay(pdMS_TO_TICKS(BLINK_INTERVAL_2));
        time_elapsed += BLINK_INTERVAL_2;
    }
}
#endif

/* ======== PWM Control ======== */
#if ENABLE_PWM
static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 3.75 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_GPIO,
        .duty           = 0, // Start with duty 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Install LEDC fade function
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
}
#endif

/* ======== MAIN FUNCTION ======== */
void app_main() {
    esp_err_t ret;
    voltage_buffer_t buffer = {0};
    int alert_count = 0;
    int detected_count = 0;
    int current_state = 0;
    int cons_count = 0;
    float voltage = 0.0f;
    int wirebreak = 0;
    int leakage = 0;

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

    /* Initialize PWM */
#if ENABLE_PWM
    ledc_init();
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

#if ENABLE_DEBUG_2
            printf("\r\n ADC RAW: %d, Voltage: %.3f V\n", raw_reading, voltage);
#endif
            /* Analyze buffer when full */
            if (buffer.count >= 5){
                float max_val, min_val;
                buffer_analyze(&buffer, &max_val, &min_val);
                buffer_reset(&buffer);
                cons_count++;
                
                if (min_val > LINEBREAK_THRESHOLD) {
                    alert_count = 1;
                    detected_count = 0;
                    current_state += 1; 
                }else if (DETECTED_THRESHOLD > min_val) {
                    alert_count = 0;
                    detected_count = 1;
                    current_state += 3;
                }else{
                    alert_count = 0;
                    detected_count = 0;
                    current_state += 7;
                }

                if (cons_count == 2){
#if ENABLE_DISPLAY
                    printf("\r\n Max: %.3f V, Min: %.3f V\n",
                    max_val, min_val);
#endif
#if ENABLE_DEBUG_1
                    printf("current_state: %d\n", current_state);
                    printf("alert_count: %d\n", alert_count);
                    printf("detected_count: %d\n", detected_count);
#endif
                    if (current_state == 2){
                        
                        printf(" *** WIRE BREAK DETECTED! ***\n");
                        current_state = 1;
                        cons_count = 1;
                        wirebreak = 1;
                        leakage = 0;
                    }else if (current_state == 6){
                        
                        printf(" ==== FOUND LEAKAGE ==== \n");
                        current_state = 3;
                        cons_count = 1;
                        wirebreak = 0;
                        leakage = 1;
                    }else if (alert_count == 1){
                        gpio_set_level(LED_PIN, 1);
                        current_state = 1;
                        cons_count = 1;
                        wirebreak = 0;
                        leakage = 0;
                    }else if (detected_count == 1){
                        gpio_set_level(LED_PIN, 1);
                        current_state = 3;
                        cons_count = 1;
                        wirebreak = 0;
                        leakage = 0;
                    }else{
                        gpio_set_level(LED_PIN, 1);
                        cons_count = 0;
                        current_state = 0;
                        wirebreak = 0;
                        leakage = 0;
                    }
                }
                if (wirebreak == 1) blink_led_wirebreak(LED_PIN, 500);
            }
        }

#if ENABLE_PWM && !ENABLE_PWM_FADE
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        // vTaskDelay(PWM_TIME / portTICK_PERIOD_MS);

#elif ENABLE_PWM && ENABLE_PWM_FADE
        // Fade to 100% duty cycle within FADE_TIME milliseconds
        ESP_ERROR_CHECK(ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY, FADE_TIME));
        ESP_ERROR_CHECK(ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_WAIT_DONE));
        vTaskDelay(FADE_TIME / portTICK_PERIOD_MS);

        // Fade back to 0% duty cycle within FADE_TIME milliseconds
        ESP_ERROR_CHECK(ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, 0, FADE_TIME));
        ESP_ERROR_CHECK(ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_WAIT_DONE));
        vTaskDelay(FADE_TIME / portTICK_PERIOD_MS);
#endif

#if ENABLE_LED
        // if (wirebreak == 1) blink_led_wirebreak(LED_PIN, 500);
        // else if (leakage == 1) blink_led_leakage(LED_PIN, 200);
        if (leakage == 1) blink_led_leakage(LED_PIN, 200);
        else vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL));
#endif
    }
}