// MY_NOTE: I split this file out so it's easier for me to tune and debug quickly.
#include "../core/source_app_internal.h"

const char *espnow_rate_name(wifi_phy_rate_t rate) {
    switch (rate) {
        case WIFI_PHY_RATE_6M: return "6M";
        case WIFI_PHY_RATE_12M: return "12M";
        case WIFI_PHY_RATE_24M: return "24M";
        default: return "other";
    }
}

wifi_phy_rate_t espnow_rate_step_down(wifi_phy_rate_t rate) {
    if (rate == WIFI_PHY_RATE_24M) return WIFI_PHY_RATE_12M;
    if (rate == WIFI_PHY_RATE_12M) return WIFI_PHY_RATE_6M;
    return WIFI_PHY_RATE_6M;
}

wifi_phy_rate_t espnow_rate_step_up(wifi_phy_rate_t rate) {
    if (rate == WIFI_PHY_RATE_6M) return WIFI_PHY_RATE_12M;
    if (rate == WIFI_PHY_RATE_12M) return WIFI_PHY_RATE_24M;
    return WIFI_PHY_RATE_24M;
}

void apply_espnow_rate_all_peers(wifi_phy_rate_t rate) {
    esp_now_rate_config_t rcfg = {
        .phymode = ESPNOW_PHY_MODE,
        .rate = rate,
        .ersu = false,
        .dcm = false,
    };
    (void)esp_now_set_peer_rate_config(BROADCAST_MAC, &rcfg);

    for (int i = 0; i < MAX_SINKS; i++) {
        if (!sinks[i].in_use) continue;
        if (!esp_now_is_peer_exist(sinks[i].mac)) continue;
        (void)esp_now_set_peer_rate_config(sinks[i].mac, &rcfg);
    }

    espnow_rate_current = rate;
    ESP_LOGW(TAG, "ESP-NOW PHY rate adjusted to %s", espnow_rate_name(rate));
}

static void lc3_init_encoder(void) {
    for (int ch = 0; ch < CHANNELS; ch++) {
        lc3_enc[ch] = lc3_setup_encoder(LC3_FRAME_US, SAMPLE_RATE_HZ, 0, &lc3_enc_mem[ch]);
        if (!lc3_enc[ch]) {
            ESP_LOGE(TAG, "lc3_setup_encoder failed ch=%d", ch);
            abort();
        }
        lc3_encoder_disable_ltpf(lc3_enc[ch]);
    }
    ESP_LOGI(TAG, "LC3 encoder ready (%dus, %dHz, %dB/ch)", LC3_FRAME_US, SAMPLE_RATE_HZ, LC3_BYTES_PER_CH);
}

static bool lc3_encode_stereo_s24(const int32_t *pcm_interleaved_s24, uint8_t *out_payload) {
    for (int ch = 0; ch < CHANNELS; ch++) {
        uint8_t *out = out_payload + ch * LC3_BYTES_PER_CH;
        int rc = lc3_encode(lc3_enc[ch], LC3_PCM_FORMAT_S24,
                            pcm_interleaved_s24 + ch, CHANNELS,
                            LC3_BYTES_PER_CH, out);
        if (rc != 0) return false;
    }
    return true;
}

static void i2s_init_rx(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = 4;
    chan_cfg.dma_frame_num = 120;

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &i2s_rx));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE_HZ,
            .clk_src = I2S_CLK_SRC_APLL,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = PIN_MCLK,
            .bclk = PIN_BCLK,
            .ws   = PIN_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = PIN_DIN,
            .invert_flags = {0},
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_rx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_rx));
    ESP_LOGI(TAG, "I2S RX ready (PCM1808) @ %d Hz", SAMPLE_RATE_HZ);
}

void beacon_task(void *arg) {
    (void)arg;
    beacon_msg_t b = {0};
    b.h.magic = PROTO_MAGIC;
    b.h.type = MSG_BEACON;
    b.h.room_code = ROOM_CODE;
    b.wifi_channel = wifi_channel;
    b.channels = CHANNELS;
    b.sample_rate_hz = SAMPLE_RATE_HZ;
    b.frame_us = LC3_FRAME_US;
    b.bytes_per_ch = LC3_BYTES_PER_CH;
    b.stream_id = stream_id;

    add_peer_if_needed(BROADCAST_MAC);

    while (1) {
        b.wifi_channel = wifi_channel;
        esp_now_send(BROADCAST_MAC, (const uint8_t *)&b, sizeof(b));
        vTaskDelay(pdMS_TO_TICKS(BEACON_PERIOD_MS));
    }
}

void audio_capture_encode_task(void *arg) {
    (void)arg;

    static int32_t i2s_buf[SAMPLES_PER_FRAME * CHANNELS];
    static int32_t pcm_s24[SAMPLES_PER_FRAME * CHANNELS];
    static uint8_t quiet_frames = 0;

    lc3_init_encoder();
    i2s_init_rx();

    while (1) {
        size_t got = 0;
        ESP_ERROR_CHECK(i2s_channel_read(i2s_rx, i2s_buf, sizeof(i2s_buf), &got, portMAX_DELAY));
        uint32_t t_end_us = (uint32_t)esp_timer_get_time();
        if (got < sizeof(i2s_buf)) {
            memset(((uint8_t*)i2s_buf) + got, 0, sizeof(i2s_buf) - got);
        }

        int32_t peak = 0;
        for (int i = 0; i < SAMPLES_PER_FRAME * CHANNELS; i++) {
            int32_t sample = clamp_pcm24(i2s_buf[i] >> 8);
            pcm_s24[i] = sample;
            int32_t mag = abs_i32(sample);
            if (mag > peak) peak = mag;
        }

        if (peak < SILENCE_GATE_PEAK_TH) {
            if (quiet_frames < SILENCE_GATE_HANG_FRAMES) {
                quiet_frames++;
            } else {
                memset(pcm_s24, 0, sizeof(pcm_s24));
            }
        } else {
            quiet_frames = 0;
        }

        bool have_sink = false;
        for (int i = 0; i < MAX_SINKS; i++) {
            if (sinks[i].in_use) { have_sink = true; break; }
        }
        bool have_udp = false;
        for (int i = 0; i < MAX_UDP_CLIENTS; i++) {
            if (udp_clients[i].in_use) { have_udp = true; break; }
        }

        bool have_peer = (have_sink || have_udp);
        if (!have_peer) continue;

        audio_msg_t m = {0};
        m.h.magic = PROTO_MAGIC;
        m.h.type = MSG_AUDIO;
        m.h.room_code = ROOM_CODE;
        m.seq = seq_num++;
        m.payload_len = LC3_BYTES_PER_CH * CHANNELS;
        m.src_t_us = stream_id;
        m.flags = 0;
        m.capture_us = t_end_us - LC3_FRAME_US;

        if (!lc3_encode_stereo_s24(pcm_s24, m.payload)) continue;

        if (audio_q) {
            if (xQueueSend(audio_q, &m, 0) != pdTRUE) {
                audio_msg_t drop;
                (void)xQueueReceive(audio_q, &drop, 0);
                (void)xQueueSend(audio_q, &m, 0);
            }
        }
        if (udp_audio_q && have_udp) {
            if (xQueueSend(udp_audio_q, &m, 0) != pdTRUE) {
                audio_msg_t drop;
                (void)xQueueReceive(udp_audio_q, &drop, 0);
                (void)xQueueSend(udp_audio_q, &m, 0);
            }
        }
    }
}

void audio_send_task(void *arg) {
    (void)arg;

    audio_msg_t m;
    while (1) {
        if (xQueueReceive(audio_q, &m, portMAX_DELAY) != pdTRUE) continue;

        uint32_t dropped_local = 0;
        audio_msg_t newer;
        while (xQueueReceive(audio_q, &newer, 0) == pdTRUE) {
            m = newer;
            dropped_local++;
        }
        stat_src_drop_espnow_q += dropped_local;

        uint32_t now = (uint32_t)esp_timer_get_time();
        for (int i = 0; i < MAX_SINKS; i++) {
            if (sinks[i].in_use && (now - sinks[i].last_seen_us) > 5000000UL) {
                if (esp_now_is_peer_exist(sinks[i].mac)) esp_now_del_peer(sinks[i].mac);
                sinks[i].in_use = false;
            }
        }

        int send_len = (int)(offsetof(audio_msg_t, payload) + m.payload_len);

        for (int i = 0; i < MAX_SINKS; i++) {
            if (!sinks[i].in_use) continue;

            if (xSemaphoreTake(tx_tokens, pdMS_TO_TICKS(AUDIO_TX_TOKEN_WAIT_MS)) != pdTRUE) {
                espnow_token_drop++;
                continue;
            }

            esp_err_t err = esp_now_send(sinks[i].mac, (const uint8_t *)&m, send_len);
            if (err != ESP_OK) {
                xSemaphoreGive(tx_tokens);
            }
        }
    }
}

void udp_audio_send_task(void *arg) {
    (void)arg;

    audio_msg_t m;
    uint32_t pace_count = 0;
    while (1) {
        if (xQueueReceive(udp_audio_q, &m, portMAX_DELAY) != pdTRUE) continue;

        bool esp_active = count_active_sinks() > 0;
        if (esp_active && radio_congested) {
            UBaseType_t aq = audio_q ? uxQueueMessagesWaiting(audio_q) : 0;
            uint32_t pace_div = (aq > 0) ? UDP_PACE_DIV_STRONG : UDP_PACE_DIV_NORMAL;
            pace_count++;
            if ((pace_count % pace_div) != 0U) {
                stat_udp_paced_drop++;
                continue;
            }
        }

        uint32_t dropped_local = 0;
        audio_msg_t newer;
        while (xQueueReceive(udp_audio_q, &newer, 0) == pdTRUE) {
            m = newer;
            dropped_local++;
        }
        stat_src_drop_udp_q += dropped_local;

        int send_len = (int)(offsetof(audio_msg_t, payload) + m.payload_len);

        if (udp_sock >= 0) {
            uint32_t udp_now = (uint32_t)esp_timer_get_time();
            for (int i = 0; i < MAX_UDP_CLIENTS; i++) {
                if (!udp_clients[i].in_use) continue;
                if ((udp_now - udp_clients[i].last_seen_us) > UDP_CLIENT_TIMEOUT_US) {
                    udp_clients[i].in_use = false;
                    continue;
                }
                sendto(udp_sock, &m, send_len, MSG_DONTWAIT,
                       (struct sockaddr *)&udp_clients[i].addr,
                       sizeof(udp_clients[i].addr));
            }
        }
    }
}

void source_stats_task(void *arg) {
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        EventBits_t bits = wifi_ev ? xEventGroupGetBits(wifi_ev) : 0;
        bool sta_up = (bits & WIFI_CONNECTED_BIT) != 0;
        if (sta_up && !napt_enabled) {
            ensure_napt_enabled();
        }

        int sinks_n = count_active_sinks();
        int udp_n = count_active_udp_clients();
        UBaseType_t qfill = audio_q ? uxQueueMessagesWaiting(audio_q) : 0;
        UBaseType_t uqfill = udp_audio_q ? uxQueueMessagesWaiting(udp_audio_q) : 0;
        uint32_t drop_es = stat_src_drop_espnow_q;
        uint32_t drop_ud = stat_src_drop_udp_q;
        uint32_t drop_paced = stat_udp_paced_drop;
        uint32_t sta_disc = sta_disconnect_count;
        uint32_t sta_rec = sta_reconnect_count;
        uint32_t sta_gap = sta_last_recover_ms;

        static uint32_t prev_ok = 0, prev_fail = 0, prev_tok = 0;
        uint32_t ok_now = espnow_send_ok;
        uint32_t fail_now = espnow_send_fail;
        uint32_t tok_now = espnow_token_drop;
        uint32_t d_ok = ok_now - prev_ok;
        uint32_t d_fail = fail_now - prev_fail;
        uint32_t d_tok = tok_now - prev_tok;
        prev_ok = ok_now;
        prev_fail = fail_now;
        prev_tok = tok_now;

        uint32_t attempts = d_ok + d_fail;
        uint32_t fail_pct = (attempts > 0) ? (d_fail * 100U / attempts) : 0;
        radio_congested = (d_tok > 0) || (attempts >= 40 && fail_pct >= 4);
        if (attempts >= 40 && fail_pct >= 6) {
            wifi_phy_rate_t lower = espnow_rate_step_down(espnow_rate_current);
            if (lower != espnow_rate_current) {
                apply_espnow_rate_all_peers(lower);
            }
        } else if (attempts >= 80 && fail_pct <= 1 && d_tok == 0) {
            wifi_phy_rate_t higher = espnow_rate_step_up(espnow_rate_current);
            if (higher != espnow_rate_current) {
                apply_espnow_rate_all_peers(higher);
            }
        }

        ESP_LOGI(TAG, "alive ch=%u seq=%u sinks=%d udp=%d q=%u uq=%u dq_es=%lu dq_udp=%lu dq_up=%lu cong=%u sta=%u napt=%u disc=%lu rec=%lu gap=%lums rate=%s fail=%u%% tok=%lu",
                 (unsigned)wifi_channel,
                 (unsigned)seq_num,
                 sinks_n,
                 udp_n,
                 (unsigned)qfill,
                 (unsigned)uqfill,
                 (unsigned long)drop_es,
                 (unsigned long)drop_ud,
                 (unsigned long)drop_paced,
                 radio_congested ? 1U : 0U,
                 sta_up ? 1U : 0U,
                 napt_enabled ? 1U : 0U,
                 (unsigned long)sta_disc,
                 (unsigned long)sta_rec,
                 (unsigned long)sta_gap,
                 espnow_rate_name(espnow_rate_current),
                 (unsigned)fail_pct,
                 (unsigned long)d_tok);
    }
}
