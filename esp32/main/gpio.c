#include "gpio.h"
#include "const.h"

// Clears pending interrupts on gpio pin
inline void clear_gpio_intr_status(gpio_num_t pin){
    uint32_t m = 1u << (uint32_t)pin;
    if(pin < 32) { GPIO.status_w1tc = m; }
    else { GPIO.status1_w1tc.val = 1u << (pin - 32); }
}

// Spawns send gps task when ptt releases
static void IRAM_ATTR ptt_interrupt_handler() {
    
    // ignore this edge; this is from a gps transmit, not a ptt release
    if (gps_tx == 1) { return; }

    int level = gpio_get_level(PTT_INPUT_PIN);
    if (level == 0) {
        return;
    }

    uint64_t now_ticks = xTaskGetTickCountFromISR();
    uint64_t elapsed_isr_ms = (now_ticks - last_isr_ticks) * portTICK_PERIOD_MS;
    if (elapsed_isr_ms < DEBOUNCE_MS) return; // debounce
    last_isr_ticks = now_ticks;

    BaseType_t hpw = pdFALSE;

    #if AUTOMATIC_TRANSMISSION // reset internal gps timer
        xTimerResetFromISR(auto_gps_timer, &hpw); 
    #endif 

    // ptt release -> send gps
    uint64_t elapsed_tx_ms = (now_ticks - last_tx_ticks) * portTICK_PERIOD_MS;
    if (elapsed_tx_ms < MIN_INTERVAL_MS) return;   // ignore too-soon interrupts
    last_tx_ticks = now_ticks;

    gpio_intr_disable(PTT_INPUT_PIN); // disable interrupts on ptt
    clear_gpio_intr_status(PTT_INPUT_PIN);
    gps_tx = 1;

    vTaskNotifyGiveFromISR(send_gps_handle, &hpw); 
    if (hpw) portYIELD_FROM_ISR();

}

// Build and config all
void config_gpio(){
    
    config_dac();
    config_ptt();

    #if AUTOMATIC_TRANSMISSION
    config_auto_gps_timer();
    #endif
}

// Build and configure dac
static void config_dac() {

    #if BUF_LEN > SAMPLES_PER_SYMBOL
        ESP_LOGI(TAG, "Ensure that DAC buffer length is less than samples per symbol");
    #endif

    i2s_config_t config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = true,
    };

    i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
    i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN); // Mono; GPIO 25 carries output
    i2s_set_pin(I2S_NUM_0, NULL);
    i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);

}

// Build and configure ptt pins and handlers
static void config_ptt() {

    gpio_config_t ptt_in_conf = {
        .intr_type = GPIO_INTR_POSEDGE,  // activates on rising edge
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PTT_INPUT_PIN),
        .pull_up_en = GPIO_PULLUP_ENABLE,      // floats ptt. radio should also, but this works better
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&ptt_in_conf);

    gpio_config_t ptt_out_conf = {
        .pin_bit_mask = 1ULL << PTT_OUTPUT_PIN,
        .mode = GPIO_MODE_OUTPUT_OD,                  // Open drain. "Pull Down"
        .pull_up_en = GPIO_PULLUP_DISABLE,     // Do not pull up or down for output
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&ptt_out_conf);

    // TODO: move. Only call once per application (shared across all GPIOs)
    gpio_install_isr_service(0);

    // Attach handler
    gpio_isr_handler_add(PTT_INPUT_PIN, ptt_interrupt_handler, NULL);

    gpio_set_level(PTT_OUTPUT_PIN, PTT_INACTIVE_LVL);

}