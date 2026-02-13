/* ESP-NOW audio source (IMA ADPCM)
 *
 * Goals:
 * - keep latency low
 * - keep encode/send simple and deterministic
 * - avoid queue-heavy paths
 */

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

static const char *TAG = "ADPCM_SRC";

// ---- Config ----
#define TEST_TONE_MODE    0   // 1=test tone, 0=I2S input

#define SAMPLE_RATE       48000
#define CHANNELS          2
#define PAYLOAD_SIZE      96   // ADPCM bytes per packet
#define SAMPLES_PER_FRAME 96   // 1 ADPCM byte = 1 stereo sample-pair
#define FRAME_MS          ((SAMPLES_PER_FRAME * 1000) / SAMPLE_RATE)  // ~5.3ms
#define PCM24_MAX         8388607
#define PCM24_MIN        -8388608

// I2S ADC pins (PCM1808)
#define PIN_MCLK          GPIO_NUM_0
#define PIN_BCLK          GPIO_NUM_27
#define PIN_WS            GPIO_NUM_25
#define PIN_DIN           GPIO_NUM_26

// ESP-NOW
#define WIFI_CHANNEL      11
#define AUDIO_MAGIC       0xAD  // packet marker
#define WIFI_TX_POWER_QDBM 60   // 15 dBm

// Broadcast MAC
static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---- IMA ADPCM tables ----
static const int16_t ima_index_table[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

static const int16_t ima_step_table[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

// ADPCM encoder state per channel
typedef struct {
    int32_t predicted;
    int8_t index;
} adpcm_state_t;

static adpcm_state_t enc_state_L = {0, 0};
static adpcm_state_t enc_state_R = {0, 0};

// ---- Packet format ----
// Header is 19 bytes: magic + seq + payload_len + L/R state + src timestamp.
typedef struct __attribute__((packed)) {
    uint8_t  magic;           // packet id
    uint16_t seq;             // sequence number
    uint16_t payload_len;     // valid bytes in payload
    int32_t  pred_L;          // left predictor (24-bit domain)
    int8_t   index_L;         // left ADPCM index
    int32_t  pred_R;          // right predictor (24-bit domain)
    int8_t   index_R;         // right ADPCM index
    uint32_t src_t_us;        // source timestamp (lower 32-bit usec)
    uint8_t  payload[PAYLOAD_SIZE];
} audio_packet_t;

// ---- State ----
static i2s_chan_handle_t i2s_rx = NULL;
static int32_t i2s_buf[SAMPLES_PER_FRAME * CHANNELS];
static int32_t pcm_buf[SAMPLES_PER_FRAME * CHANNELS];
static audio_packet_t tx_pkt;
static uint16_t seq_num = 0;

// Runtime counters
static uint32_t tx_ok = 0, tx_fail = 0;
static int32_t audio_peak = 0;

// ---- ADPCM encode ----
static inline int32_t clamp_pcm24(int32_t value) {
    if (value > PCM24_MAX) return PCM24_MAX;
    if (value < PCM24_MIN) return PCM24_MIN;
    return value;
}

static inline uint8_t adpcm_encode_sample(int32_t sample, adpcm_state_t *state) {
    int diff = sample - state->predicted;
    uint8_t nibble = 0;
    
    if (diff < 0) {
        nibble = 8;
        diff = -diff;
    }
    
    int32_t step = ((int32_t)ima_step_table[state->index]) << 8;
    int delta = step >> 3;
    
    if (diff >= step) {
        nibble |= 4;
        diff -= step;
        delta += step;
    }
    step >>= 1;
    if (diff >= step) {
        nibble |= 2;
        diff -= step;
        delta += step;
    }
    step >>= 1;
    if (diff >= step) {
        nibble |= 1;
        delta += step;
    }
    
    // update predictor
    if (nibble & 8)
        state->predicted -= delta;
    else
        state->predicted += delta;
    
    // clamp to 24-bit range
    state->predicted = clamp_pcm24(state->predicted);
    
    // update index
    state->index += ima_index_table[nibble];
    if (state->index < 0) state->index = 0;
    else if (state->index > 88) state->index = 88;
    
    return nibble;
}

// Encode one stereo frame (interleaved samples -> interleaved nibbles)
static void adpcm_encode_frame(const int32_t *pcm, uint8_t *adpcm, int samples) {
    for (int i = 0; i < samples; i++) {
        uint8_t nibble_L = adpcm_encode_sample(pcm[i * 2], &enc_state_L);
        uint8_t nibble_R = adpcm_encode_sample(pcm[i * 2 + 1], &enc_state_R);
        adpcm[i] = (nibble_L << 4) | nibble_R;
    }
}

// ---- Test tone ----
#if TEST_TONE_MODE
static float tone_phase = 0.0f;
static const float TONE_FREQ = 440.0f;

static void generate_tone(int16_t *out, int samples) {
    const float amplitude = 20000.0f;
    const float phase_inc = 2.0f * M_PI * TONE_FREQ / SAMPLE_RATE;
    
    for (int i = 0; i < samples; i++) {
        int16_t sample = (int16_t)(sinf(tone_phase) * amplitude);
        out[i * 2] = sample;      // Left
        out[i * 2 + 1] = sample;  // Right
        tone_phase += phase_inc;
        if (tone_phase >= 2.0f * M_PI) tone_phase -= 2.0f * M_PI;
    }
}
#endif

// ---- I2S setup ----
#if !TEST_TONE_MODE
static void init_i2s(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 2;  // low latency
    chan_cfg.dma_frame_num = 32;   // ~0.67ms per buffer
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &i2s_rx));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_APLL,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = PIN_MCLK,
            .bclk = PIN_BCLK,
            .ws = PIN_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = PIN_DIN,
            .invert_flags = { false, false, false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_rx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_rx));
    ESP_LOGI(TAG, "I2S RX initialized");
}
#endif

// ---- ESP-NOW callbacks ----
static void espnow_send_cb(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) tx_ok++;
    else tx_fail++;
}

// ---- Wi-Fi + ESP-NOW setup ----
static void init_wifi_espnow(void) {
    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Network stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Wi-Fi station, no power save
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(WIFI_TX_POWER_QDBM));

    // ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    // Broadcast peer
    esp_now_peer_info_t peer = {
        .channel = WIFI_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "ESP-NOW ready on channel %d, tx_power=%d qdBm", WIFI_CHANNEL, WIFI_TX_POWER_QDBM);
}

// ---- Audio task ----
static void audio_task(void *arg) {
#if !TEST_TONE_MODE
    init_i2s();
#endif
    
    vTaskDelay(pdMS_TO_TICKS(200));
    
    TickType_t last_wake = xTaskGetTickCount();
    
#if TEST_TONE_MODE
    ESP_LOGI(TAG, "TEST MODE: 440Hz tone, %dms frames", FRAME_MS);
#else
    ESP_LOGI(TAG, "I2S MODE: PCM1808 input, %dms frames", FRAME_MS);
#endif

    while (1) {
#if TEST_TONE_MODE
        // fixed timing for test tone
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FRAME_MS));
        generate_tone(pcm_buf, SAMPLES_PER_FRAME);
        audio_peak = 20000;
#else
        // blocking read from I2S (timing source)
        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(i2s_rx, i2s_buf, sizeof(i2s_buf), 
                                          &bytes_read, pdMS_TO_TICKS(20));
        if (ret != ESP_OK || bytes_read < sizeof(i2s_buf)) {
            continue;
        }
        
        // convert 32-bit lane -> signed 24-bit and track peak
        int32_t peak = 0;
        for (int i = 0; i < SAMPLES_PER_FRAME * CHANNELS; i++) {
            int32_t s24 = clamp_pcm24(i2s_buf[i] >> 8);
            pcm_buf[i] = s24;
            int32_t abs_s = s24 < 0 ? -s24 : s24;
            if (abs_s > peak) peak = abs_s;
        }
        audio_peak = peak;
#endif

        // fill packet header with encoder state
        tx_pkt.magic = AUDIO_MAGIC;
        tx_pkt.seq = seq_num++;
        tx_pkt.payload_len = PAYLOAD_SIZE;  // fixed payload size
        tx_pkt.pred_L = enc_state_L.predicted;
        tx_pkt.index_L = enc_state_L.index;
        tx_pkt.pred_R = enc_state_R.predicted;
        tx_pkt.index_R = enc_state_R.index;
        tx_pkt.src_t_us = (uint32_t)esp_timer_get_time();
        
        // ADPCM encode
        adpcm_encode_frame(pcm_buf, tx_pkt.payload, SAMPLES_PER_FRAME);
        
        // send packet (19-byte header + payload)
        int pkt_size = 19 + tx_pkt.payload_len;
        esp_err_t send_ret = esp_now_send(BROADCAST_MAC, (uint8_t*)&tx_pkt, pkt_size);
        if (send_ret != ESP_OK) {
            tx_fail++;
            // brief back-off when TX queue is busy
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

// ---- Runtime stats ----
static void status_task(void *arg) {
    uint32_t last_ok = 0, last_seq = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        uint32_t rate = (tx_ok - last_ok) / 5;
        uint32_t pps = (seq_num - last_seq) / 5;
        last_ok = tx_ok;
        last_seq = seq_num;
        ESP_LOGI(TAG, "tx=%lu ok=%lu fail=%lu peak=%ld rate=%lu/s pps=%lu/s",
                 (tx_ok + tx_fail), tx_ok, tx_fail, audio_peak, rate, pps);
    }
}

// ---- App entry ----
void app_main(void) {
    ESP_LOGI(TAG, "ESP-NOW ADPCM source online");
    ESP_LOGI(TAG, "frame=%dms samples=%d packet=%dB", 
             FRAME_MS, SAMPLES_PER_FRAME, (int)sizeof(tx_pkt));
    
    init_wifi_espnow();
    
    // Keep audio task off Wi-Fi core.
    xTaskCreatePinnedToCore(audio_task, "audio", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(status_task, "status", 2048, NULL, 1, NULL, 0);
}
