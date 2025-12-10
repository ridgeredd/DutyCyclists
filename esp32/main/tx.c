#include "tx.h"
#include "libfec/fec.h"
#include "driver/i2s.h"
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

// TODO: replace printfs with esp_logis

#define PI                      3.141592653589793f

#define SAMPLE_RATE             48000  // 48 kHz sample rate; ideally this is a multiple of each symbol frequency
#define DAC_BUF_LEN             256    // samples per DAC buffer

#define SAMPLES_PER_SYMBOL      400
#define BITS_PER_SYMBOL         1                                   // power of 2; will cause error if > 8 because using uint8
#define NSYMBOLS                (1u << BITS_PER_SYMBOL)
#define SYMBOL_MASK             (NSYMBOLS - 1u)

static const float SYMBOL_FREQS[NSYMBOLS] = { 2200.0f, 1200.0f }; // symbol frequencies in order of bit index

#define RADIO_ID                       0x67    // unique id for this device
#define START_STOP                     0x7E   // starting and stopping condition
#define START_NBYTES                   2      // 6 start bytes in a row
#define STOP_NBYTES                    1      // 4 end bytes

#define PARITY_NBYTES                  6      // These must sum to < 255
#define PAYLOAD_NBYTES                 9      // payload + callsign      
#define TRANSMISSION_NBYTES            PAYLOAD_NBYTES + PARITY_NBYTES + START_NBYTES + STOP_NBYTES

const double TRANSMISSION_TIME_MS   = (1000 * SAMPLES_PER_SYMBOL * 8 * TRANSMISSION_NBYTES) / (BITS_PER_SYMBOL * SAMPLE_RATE);
const double MIN_INTERVAL_MS        = 4 * TRANSMISSION_TIME_MS;   // Don't allow transmissions within min interval of eachother 
const double BAUD_RATE              = SAMPLE_RATE / SAMPLES_PER_SYMBOL;
const double BIT_RATE               = BAUD_RATE * BITS_PER_SYMBOL;

// Encoder handle for reed-solomon
static void* rs_encoder = NULL;

// Handle for asynchronous gps sending task
static TaskHandle_t tx_gps_handle = NULL;         // task handle to request transmission

// TODO: create timer to reenable transmission

// TODO: implement in a gps class; currently stubbed
static void get_gps(uint8_t *bytes) {
    for (uint8_t count = 0; count < PAYLOAD_NBYTES - 1; count++) {
        bytes[count] = count;
    }
}

//================================================ Static Def ====================================================

static void tx_gps_task();
static void config_dac();

//================================================ API =============================================================

void tx_init() {

    config_dac();

    // Initialize reed-solomon encoder. Requirements for library specified here: 
    // https://manpages.debian.org/unstable/libfec-dev/rs.3.en.html?utm_source=chatgpt.com
    rs_encoder = init_rs_char(
        8, // gives the symbol size in bits, up to 32
        0x11D, // gives the extended Galois field generator polynomial coefficients, with the 0th coefficient in the low order bit. The polynomial must be primitive; if not, the call will fail and NULL will be returned.
		0, // gives, in index form, the first consecutive root of the Reed Solomon code generator polynomial
        1, // gives, in index form, the primitive element in the Galois field used to generate the Reed Solomon code generator polynomial.
        PARITY_NBYTES, // gives the number of roots in the Reed Solomon code generator polynomial. This equals the number of parity symbols per code block.
		255 - PARITY_NBYTES - PAYLOAD_NBYTES // gives the number of leading symbols in the codeword that are implicitly padded to zero in a shortened code block.
        // The resulting Reed-Solomon code has parameters (N,K), where N = 2^symsize - pad - 1 and K = N-nroots.
    );

    xTaskCreate(tx_gps_task, "gps_task", 4096, NULL, 3 /*priority*/, &tx_gps_handle);
}

// Wake an asynch task to send gps
void tx_gps() { xTaskNotifyGive(tx_gps_handle); }

//==================================================================================================================

// Build and configure dac
static void config_dac() {

    #if DAC_BUF_LEN > SAMPLES_PER_SYMBOL
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

// Send a string of bytes; Sends symbols in little-endian
// Uses bit stuffing to add a 0 after every 5 ones, excluding first and last bytes
static void tx_bytes(uint8_t *bytes, uint32_t nbytes, int16_t *buf){

    uint8_t byte_idx = 0;
    uint8_t byte = bytes[0];
    uint8_t bit_idx = 0;

    #if BITS_PER_SYMBOL == 1

    uint8_t bstuff_en = 0;    // is bit stuffing currently enabled for this bit
    uint8_t prev_nrzi = 0;     // stores last bit transmitted for NRZI
    uint8_t bit = ( byte & SYMBOL_MASK );

    uint8_t nrzi_bit;
    if( bit ) { nrzi_bit = prev_nrzi; } else { nrzi_bit = !prev_nrzi; }
    printf("%d", nrzi_bit);
    uint8_t num_ones = 0;   // used for bit stuffing; after 5 ones insert a 0; not used in first / last byte
    float freq = SYMBOL_FREQS[nrzi_bit];

    #else

    float freq = SYMBOL_FREQS[byte & SYMBOL_MASK]

    #endif

    // do the first read and shift before the loop
    byte = byte >> BITS_PER_SYMBOL; 
    bit_idx += BITS_PER_SYMBOL;

    float phase = 0;
    int count = 0;   // times the symbol switches

    size_t written = 0;
    
    while (1) {

        // determines how many samples until will switch 
        // Ensure that the buffer length is less than the samples per symbol
        int next_switch = SAMPLES_PER_SYMBOL - count; 

        // fill the transmission buffer with samples at a frequency
        for (int i = 0; i < DAC_BUF_LEN; i++) {

            // Write sample to dac
            float cos01 = 0.5f * cosf(phase) + 0.5f;                                    // cos wave between 0 and 1 with amplitude 0.5
            float voltage = 2 * DAC_AMPLITUDE_V * cos01 + MIN_DAC_V;                    // use parameters to determine voltage to send at
            uint8_t u8 = (uint8_t)(voltage * 255 / 3.3);                                // normalize to 0-255 byte to send
            buf[2 * i] = (int16_t)(u8 << 8);                    // Top byte gets fed to DAC
            buf[2 * i + 1] = 0;                                 // Other DAC zeroed; Required for mono

            // If this sample swaps frequency
            // TODO: bit stuffing should happen before nrzi; currently happens after
            if (i == next_switch) {

                // If made through entire byte, continue in byte array
                if (bit_idx == 8) {

                    //printf(" ");

                    // If transmission finished; auto flush buffer
                    if (byte_idx == nbytes - 1) { 
                        //printf("\n\n");
                        i2s_write(I2S_NUM_0, buf, 2 * (i + 1) * sizeof(int16_t), &written, portMAX_DELAY); 
                        return;
                    }
                    // else continue through byte array
                    byte_idx++;
                    byte = bytes[byte_idx];
                    bit_idx = 0;

                }

                // Get frequency of next symbol
                #if BITS_PER_SYMBOL == 1

                    // save the previous level
                    prev_nrzi = nrzi_bit;

                    // normal case; grab next symbol and shift
                    // don't bit stuff on first or last byte
                    if ( !bstuff_en || byte_idx < START_NBYTES || byte_idx >= TRANSMISSION_NBYTES - STOP_NBYTES ) {  
                        bit = (byte & SYMBOL_MASK);
                        bit_idx += BITS_PER_SYMBOL;
                        byte = byte >> BITS_PER_SYMBOL;
                    }
                    // this bit should be stuffed; retain current position and inject 0
                    else { bit = 0; }

                    // if 1 increment consecutive 1's counter 
                    if ( bit && byte_idx != 0 && byte_idx != nbytes - 1 ) { num_ones += 1; } 
                    // else reset consecutive 1's counter 
                    else { 
                        num_ones = 0; 
                        bstuff_en = 0; // catches case where we are currently bit stuffing (bit = 0) and disables it
                    }

                    // five ones in a row, enable bit stuffing for next bit
                    if ( num_ones >= 5 ) { bstuff_en = 1; num_ones = 0; }

                    // calculate nrzi
                    if( bit ) { nrzi_bit = prev_nrzi; } else { nrzi_bit = !prev_nrzi; }
                    //printf("%d", nrzi_bit);

                    // correct frequency to transmit at
                    freq = SYMBOL_FREQS[nrzi_bit];

                #else
                    // simple; no NRZI or stuffing; simply shift and read
                    freq = SYMBOL_FREQS[byte & SYMBOL_MASK];
                    bit_idx += BITS_PER_SYMBOL;
                #endif

            }

            // update phase; use a phase accumulator for smooth transitions between frequencies
            phase += 2 * PI * freq / SAMPLE_RATE;
            if (phase >= 2 * PI) { phase -= 2 * PI; }

        }
        // buffer full; flush
        i2s_write(I2S_NUM_0, buf, 2 * DAC_BUF_LEN * sizeof(int16_t), &written, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1)); // delay for some amount of time to free up other tasks
        count += DAC_BUF_LEN;
        if (count > SAMPLES_PER_SYMBOL) { count -= SAMPLES_PER_SYMBOL; } // wrap around if transmitted symbol changed
        // it is important this is > not >= bc >= would cause issue where new count == samples per symbol, would reset to 0 rather than changing in next iteration

    }

}

// Continuously running process that wakes to send gps transmission; Assumes PTT is already active
static void tx_gps_task() {

    int16_t *buf = (int16_t*)heap_caps_malloc(DAC_BUF_LEN * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    uint8_t *tx_data = (uint8_t*)heap_caps_malloc(TRANSMISSION_NBYTES * sizeof(uint8_t), MALLOC_CAP_DEFAULT);

    tx_data[0] = START_STOP;
    tx_data[1] = START_STOP;
    tx_data[START_NBYTES] = RADIO_ID;
    tx_data[TRANSMISSION_NBYTES - 1] = START_STOP;

    while(1) {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);                // wait until woken up
        uint8_t *payload = (uint8_t*)(tx_data + START_NBYTES);             // pointer to the payload bytes
        uint8_t *gps     = (uint8_t*)(payload + 1);                

        get_gps(gps);                                       // currently stubbed
        uint8_t *parity = (uint8_t*)(payload + PAYLOAD_NBYTES);  // pointer to the parity bytes

        encode_rs_char(rs_encoder, payload, parity); // set parity bytes

        printf("payload: ");
        for (size_t i = 0; i < PAYLOAD_NBYTES; i++) {
            printf("%02X ", payload[i]);  // two-digit uppercase hex
        }
        printf("\n");

        printf("Message: ");
        for (size_t i = 0; i < TRANSMISSION_NBYTES; i++) {
            printf("%02X ", tx_data[i]);  // two-digit uppercase hex
        }
        printf("\n\n");

        tx_bytes(tx_data, TRANSMISSION_NBYTES, buf);

    }

}
