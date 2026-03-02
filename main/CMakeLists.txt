idf_component_register(
    SRCS
        "main_adpcm.c"
        "source_app.c"
        "core/source_core.c"
        "audio/source_audio.c"
        "wifi/source_wifi.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_timer driver nvs_flash esp_http_server liblc3
)
