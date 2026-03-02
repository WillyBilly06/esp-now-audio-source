// MY_NOTE: I split this file out so it's easier for me to tune and debug quickly.
#pragma once

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_random.h"

#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_http_server.h"

#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "lwip/sockets.h"
#include "lwip/ip_addr.h"
#include "lwip/ip4_addr.h"

#include "lc3.h"

extern const char *TAG;

#define SAMPLE_RATE_HZ          48000
#define CHANNELS               2
#define LC3_FRAME_US           7500
#define LC3_BYTES_PER_CH       72
#define SAMPLES_PER_FRAME      ((SAMPLE_RATE_HZ * LC3_FRAME_US) / 1000000)

#define PCM24_MAX              8388607
#define PCM24_MIN             -8388608
#define SILENCE_GATE_PEAK_TH   1200
#define SILENCE_GATE_HANG_FRAMES 6

#define PIN_MCLK               GPIO_NUM_0
#define PIN_BCLK               GPIO_NUM_27
#define PIN_WS                 GPIO_NUM_25
#define PIN_DIN                GPIO_NUM_26

#define ROOM_CODE              0x7F
#define WIFI_CHANNEL_DEFAULT   6
#define WIFI_TX_POWER_QDBM     84
#define BEACON_PERIOD_MS       400
#define MAX_SINKS              10

#define PROV_AP_SSID           "ALS_SETUP"
#define PROV_AP_PASSWORD       ""
#define WIFI_CONNECT_TIMEOUT_MS 15000
#define WIFI_MAX_RETRY         20
#define WIFI_RECONNECT_BASE_MS 250
#define WIFI_RECONNECT_MAX_MS  5000
#define WIFI_CRED_NAMESPACE    "wifi_cfg"
#define WIFI_CRED_KEY_SSID     "ssid"
#define WIFI_CRED_KEY_PASS     "pass"

#define SOURCE_AP_SSID         "ALS_TX"
#define SOURCE_AP_PASSWORD     "openals123"

#ifndef CONFIG_SOFTAP_SSID
#define CONFIG_SOFTAP_SSID SOURCE_AP_SSID
#endif

#ifndef CONFIG_SOFTAP_PASSWORD
#define CONFIG_SOFTAP_PASSWORD SOURCE_AP_PASSWORD
#endif

#ifndef CONFIG_SOURCE_AP_IP_C
#define CONFIG_SOURCE_AP_IP_C 50
#endif

#ifndef CONFIG_AUDIO_QUEUE_LEN
#define CONFIG_AUDIO_QUEUE_LEN 12
#endif

#ifndef CONFIG_AUDIO_TX_TOKENS
#define CONFIG_AUDIO_TX_TOKENS 16
#endif

#ifndef CONFIG_AUDIO_TASK_PRIO_CAP_ENC
#define CONFIG_AUDIO_TASK_PRIO_CAP_ENC 7
#endif

#ifndef CONFIG_AUDIO_TASK_PRIO_SEND
#define CONFIG_AUDIO_TASK_PRIO_SEND 8
#endif

#define SOURCE_AP_IP_A         192
#define SOURCE_AP_IP_B         168
#define SOURCE_AP_IP_C         CONFIG_SOURCE_AP_IP_C
#define SOURCE_AP_IP_D         1

#define AUDIO_QUEUE_LEN         CONFIG_AUDIO_QUEUE_LEN
#define AUDIO_TX_TOKENS         CONFIG_AUDIO_TX_TOKENS
#define AUDIO_TX_TOKEN_WAIT_MS  2
#define TASK_PRIO_CAP_ENC       CONFIG_AUDIO_TASK_PRIO_CAP_ENC
#define TASK_PRIO_SEND          CONFIG_AUDIO_TASK_PRIO_SEND
#define TASK_PRIO_UDP_SEND      4
#define UDP_AUDIO_QUEUE_LEN     2

#define UDP_PACE_DIV_NORMAL     2
#define UDP_PACE_DIV_STRONG     4

#define ESPNOW_PHY_MODE         WIFI_PHY_MODE_11G
#if defined(CONFIG_ESPNOW_RATE_6M)
#define ESPNOW_PHY_RATE         WIFI_PHY_RATE_6M
#elif defined(CONFIG_ESPNOW_RATE_12M)
#define ESPNOW_PHY_RATE         WIFI_PHY_RATE_12M
#else
#define ESPNOW_PHY_RATE         WIFI_PHY_RATE_12M
#endif

#define UDP_PORT               5000
#define MAX_UDP_CLIENTS        10
#define UDP_CLIENT_TIMEOUT_US  5000000UL

#define PROTO_MAGIC            0xA5

typedef enum {
    MSG_BEACON     = 0x01,
    MSG_JOIN       = 0x02,
    MSG_LEAVE      = 0x03,
    MSG_KEEPALIVE  = 0x04,
    MSG_AUDIO      = 0x10,
} msg_type_t;

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t type;
    uint8_t room_code;
    uint8_t reserved;
} msg_hdr_t;

typedef struct __attribute__((packed)) {
    msg_hdr_t h;
    uint8_t  wifi_channel;
    uint8_t  channels;
    uint16_t sample_rate_hz;
    uint16_t frame_us;
    uint8_t  bytes_per_ch;
    uint32_t stream_id;
} beacon_msg_t;

typedef struct __attribute__((packed)) {
    msg_hdr_t h;
    uint32_t stream_id;
} join_msg_t;

typedef struct __attribute__((packed)) {
    msg_hdr_t h;
    uint32_t stream_id;
} leave_msg_t;

typedef struct __attribute__((packed)) {
    msg_hdr_t h;
    uint32_t stream_id;
    uint16_t last_seq_rx;
} keepalive_msg_t;

typedef struct __attribute__((packed)) {
    msg_hdr_t h;
    uint16_t seq;
    uint16_t payload_len;
    uint32_t src_t_us;
    uint8_t  flags;
    uint32_t capture_us;
    uint8_t  payload[LC3_BYTES_PER_CH * CHANNELS];
} audio_msg_t;

typedef struct {
    uint8_t mac[6];
    bool in_use;
    uint32_t last_seen_us;
} sink_peer_t;

typedef struct {
    struct sockaddr_in addr;
    bool in_use;
    uint32_t last_seen_us;
} udp_client_t;

extern i2s_chan_handle_t i2s_rx;
extern lc3_encoder_mem_48k_t lc3_enc_mem[CHANNELS];
extern lc3_encoder_t lc3_enc[CHANNELS];
extern uint32_t stream_id;
extern uint16_t seq_num;
extern sink_peer_t sinks[MAX_SINKS];
extern QueueHandle_t audio_q;
extern QueueHandle_t udp_audio_q;
extern SemaphoreHandle_t tx_tokens;
extern const uint8_t BROADCAST_MAC[6];
extern uint8_t wifi_channel;
extern EventGroupHandle_t wifi_ev;
extern int wifi_retry;
extern esp_netif_t *wifi_sta_netif;
extern esp_netif_t *wifi_ap_netif;
extern bool napt_enabled;
extern bool streaming_mode_active;
extern volatile uint32_t sta_disconnect_count;
extern volatile uint32_t sta_reconnect_count;
extern volatile uint32_t sta_last_drop_us;
extern volatile uint32_t sta_last_recover_ms;
extern TimerHandle_t sta_reconnect_timer;
extern uint32_t sta_reconnect_backoff_ms;
extern wifi_phy_rate_t espnow_rate_current;
extern volatile uint32_t espnow_send_ok;
extern volatile uint32_t espnow_send_fail;
extern volatile uint32_t espnow_token_drop;
extern volatile bool radio_congested;
extern volatile bool provision_done;
extern udp_client_t udp_clients[MAX_UDP_CLIENTS];
extern int udp_sock;
extern volatile uint32_t stat_src_drop_espnow_q;
extern volatile uint32_t stat_src_drop_udp_q;
extern volatile uint32_t stat_udp_paced_drop;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

int find_sink_slot_by_mac(const uint8_t mac[6]);
int alloc_sink_slot(const uint8_t mac[6]);
void remove_sink_slot(const uint8_t mac[6]);
int count_active_sinks(void);
int count_active_udp_clients(void);
int32_t clamp_pcm24(int32_t v);
int32_t abs_i32(int32_t v);

void load_or_start_provisioning(char *sta_ssid, size_t ssid_len, char *sta_pass, size_t pass_len);
void wifi_start_apsta(const char *ssid, const char *password);
void start_provisioning_portal(void);
void wifi_init_espnow(void);
void add_peer_if_needed(const uint8_t mac[6]);
void ensure_napt_enabled(void);

const char *espnow_rate_name(wifi_phy_rate_t rate);
wifi_phy_rate_t espnow_rate_step_down(wifi_phy_rate_t rate);
wifi_phy_rate_t espnow_rate_step_up(wifi_phy_rate_t rate);
void apply_espnow_rate_all_peers(wifi_phy_rate_t rate);

void espnow_send_cb(const esp_now_send_info_t *info, esp_now_send_status_t status);
void espnow_recv_cb(const esp_now_recv_info_t *ri, const uint8_t *data, int len);

void beacon_task(void *arg);
void audio_capture_encode_task(void *arg);
void audio_send_task(void *arg);
void udp_audio_send_task(void *arg);
void udp_server_task(void *arg);
void source_stats_task(void *arg);

void source_core_main(void);
