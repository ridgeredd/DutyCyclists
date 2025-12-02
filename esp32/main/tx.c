#include "tx.h"
#include "const.h"
#include "gps.c"

int16_t *dac_buf;           // DAC buffer
uint8_t *tx_data;           // Pointer to transmitted data array
uint8_t *payload;           // Pointer to payload (message) bytes
uint8_t *parity;            // Pointer to Reed Solomon bytes
static void* rs_encoder;    // Reed Solomon encoder settings

static TimerHandle_t ptt_disable_timer;      // Waits to reset PTT to inactive  

#if AUTOMATIC_TRANSMISSION
static TimerHandle_t auto_tx_timer = NULL;  // Waits to transmit at regular intervals
#endif

// Initializes all timers, memory, tasks, and reed solomon encoder. Returns the handle of the task to wake to send gps
TaskHandle_t tx_task_config(){

    dac_buf = (int16_t*)heap_caps_malloc(BUF_LEN * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    tx_data = (uint8_t*)heap_caps_malloc(TRANSMISSION_BYTES * sizeof(uint8_t), MALLOC_CAP_DEFAULT);

    tx_data[0] = START_STOP;
    tx_data[1] = CALL_SIGN;
    tx_data[TRANSMISSION_BYTES - 1] = START_STOP;

    uint8_t *payload = (uint8_t*)(tx_data + 2);             // pointer to the payload bytes
    uint8_t *parity = (uint8_t*)(payload + PAYLOAD_BYTES);  // pointer to the parity bytes

    // Initialize reed-solomon encoder. Requirements for library specified here: 
    // https://manpages.debian.org/unstable/libfec-dev/rs.3.en.html?utm_source=chatgpt.com
    rs_encoder = init_rs_char(
        8, // gives the symbol size in bits, up to 32
        0x11D, // gives the extended Galois field generator polynomial coefficients, with the 0th coefficient in the low order bit. The polynomial must be primitive; if not, the call will fail and NULL will be returned.
		0, // gives, in index form, the first consecutive root of the Reed Solomon code generator polynomial
        1, // gives, in index form, the primitive element in the Galois field used to generate the Reed Solomon code generator polynomial.
        PARITY_BYTES, // gives the number of roots in the Reed Solomon code generator polynomial. This equals the number of parity symbols per code block.
		255 - PARITY_BYTES - PAYLOAD_BYTES // gives the number of leading symbols in the codeword that are implicitly padded to zero in a shortened code block.
        // The resulting Reed-Solomon code has parameters (N,K), where N = 2^symsize - pad - 1 and K = N-nroots.
    );

    TaskHandle_t send_gps_handle;   // task handle to request transmission

    #if AUTOMATIC_TRANSMISSION
    config_auto_tx_timer();
    #endif

    config_ptt_disable_timer();

    xTaskCreate(tx_gps_task, "gps_task", 4096, NULL, 3 /*priority*/, &send_gps_handle);
    return send_gps_handle;

}

// Build and configure internal gps timer
static void config_auto_tx_timer() {

    #if TIMER_DELAY_MS < MIN_INTERVAL_MS
        ESP_LOGI(TAG, "Ensure that timer delay is greater than minimum interval between tx and total transmission time");
    #endif

    auto_gps_timer = xTimerCreate(
        "gps_timer",
        pdMS_TO_TICKS(TIMER_DELAY_MS),
        pdTRUE,           // auto-reload = true (repeats)
        NULL,
        auto_gps_timer_callback
    );

    xTimerStart(auto_tx_timer, 0);

}

// Build and configure timer to disable ptt
static void config_ptt_disable_timer() {

    ptt_disable_timer = xTimerCreate(
        "ptt_disable_timer",
        pdMS_TO_TICKS(TRANSMISSION_TIME_MS + PTT_BUFFER_MS),
        pdFALSE,           // auto-reload = false
        NULL,
        disable_ptt_timer_callback
    );

}

// Start the automatic transmit timer
// TODO
void start_auto_tx_timer() {}

// Reset the automatic transmit timer to 0
// TODO
void reset_auto_tx_timer() {}

// Send a string of bytes; Sends symbols in little-endian
static void tx_bytes(uint8_t *bytes, uint32_t nbytes, int16_t *buf){

    int byte_idx = 0;
    int bit_idx = BITS_PER_SYMBOL;
    uint32_t byte = bytes[0];
    float freq = SYMBOL_FREQS[byte & SYMBOL_MASK];

    static float phase = 0;
    static int count = 0;   // times the symbol switches

    size_t written = 0;
    
    while (1) {

        int next_switch = SAMPLES_PER_SYMBOL - count; // Ensure that the buffer length is less than the samples per symbol
        for (int i = 0; i < BUF_LEN; i++) {

            float cos01 = 0.5f * cosf(phase) + 0.5f;                                    // cos wave between 0 and 1 with amplitude 0.5
            float voltage = 2 * DAC_AMPLITUDE_V * cos01 + MIN_DAC_V;                    // use parameters to determine voltage to send at
            uint8_t u8 = (uint8_t)(voltage * 255 / 3.3);                                // normalize to 0-255 byte to send
            buf[2 * i] = (int16_t)(u8 << 8);                    // Top byte gets fed to DAC
            buf[2 * i + 1] = 0;                                 // Required for mono

            // Swap frequency mid buffer
            if (i == next_switch) {

                // increment bits
                if (bit_idx < 8) {
                    byte = byte >> BITS_PER_SYMBOL;
                    freq = SYMBOL_FREQS[byte & SYMBOL_MASK];
                    bit_idx += BITS_PER_SYMBOL;
                } 
                // increment to new word
                else {
                    if (byte_idx == nbytes - 1) { // transmission finished; auto flush buffer
                        i2s_write(I2S_NUM_0, buf, 2 * (i + 1) * sizeof(int16_t), &written, portMAX_DELAY); 
                        return;
                    }
                    else {
                        byte_idx++;
                        byte = bytes[byte_idx];
                        freq = SYMBOL_FREQS[byte & SYMBOL_MASK];
                        bit_idx = BITS_PER_SYMBOL;
                    }
                }

            }

            // update phase
            phase += 2 * PI * freq / SAMPLE_RATE;
            if (phase >= 2 * PI) { phase -= 2 * PI; }

        }
        // buffer full; flush
        i2s_write(I2S_NUM_0, buf, 2 * BUF_LEN * sizeof(int16_t), &written, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1)); // delay for some amount of time to free up other tasks
        count += BUF_LEN;
        if (count > SAMPLES_PER_SYMBOL) { count -= SAMPLES_PER_SYMBOL; } // wrap around if transmitted symbol changed
        // it is important this is > not >= bc >= would cause issue where new count == samples per symbol, would reset to 0 rather than changing in next iteration

    }

}

// Continuously running process that wakes to send gps transmission
static void tx_gps_task() {

    while(1) {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);                // wait until woken up
        get_gps(payload);                                       // currently stubbed

        printf("Data: ");
        for (size_t i = 0; i < PAYLOAD_BYTES; i++) {
            printf("%02X ", payload[i]);  // two-digit uppercase hex
        }
        printf("\n");

        encode_rs_char(rs_encoder, payload, parity); // set parity bytes

        printf("Encoded: ");
        for (size_t i = 0; i < TRANSMISSION_BYTES; i++) {
            printf("%02X ", tx_data[i]);  // two-digit uppercase hex
        }
        printf("\n\n");

        gpio_set_level(PTT_OUTPUT_PIN, PTT_ACTIVE_LVL); // set ptt active
        xTimerStart(ptt_disable_timer, 0);  // start timer to reset ptt after some time
        tx_bytes(tx_data, TRANSMISSION_BYTES, dac_buf);

    }

}

// Resets ptt after gps transmission
static void disable_ptt_timer_callback() {
    gpio_set_level(PTT_OUTPUT_PIN, PTT_INACTIVE_LVL);
    clear_gpio_intr_status(PTT_INPUT_PIN);
    gpio_intr_enable(PTT_INPUT_PIN);
    gps_tx = 0;
}

// Callback function automatically send gps
static void auto_gps_timer_callback() {

    int ptt_level = gpio_get_level(PTT_INPUT_PIN);
    if (ptt_level == PTT_ACTIVE_LVL) { return; } // if ptt currently active, abort

    uint64_t now_ticks = xTaskGetTickCount();
    uint64_t elapsed_tx_ms = (now_ticks - last_tx_ticks) * portTICK_PERIOD_MS;
    if (elapsed_tx_ms < MIN_INTERVAL_MS) return;   // ignore too-soon interrupts
    last_tx_ticks = now_ticks;

    gpio_set_level(PTT_OUTPUT_PIN, PTT_ACTIVE_LVL);
    xTaskNotifyGive(send_gps_handle);

}