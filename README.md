# ESP-NOW Audio Source

Wireless audio transmitter built on the ESP32. Captures stereo audio from a PCM1808 ADC over I2S, compresses it with IMA ADPCM, and broadcasts it over ESP-NOW. Designed to pair with the [ESP-NOW Audio Sink](https://github.com/WillyBilly06/esp-now-audio-sink) receiver.

---

## Overview

This is one half of a wireless audio link. The source board reads 48 kHz / 24-bit stereo PCM from a PCM1808, encodes each frame into IMA ADPCM (roughly 12:1 compression), and sends it as a broadcast ESP-NOW packet. No pairing or handshake is needed -- any sink on the same Wi-Fi channel will pick it up.

Each packet carries 96 stereo sample-pairs (~2 ms of audio), plus a small header with the ADPCM predictor state and a microsecond timestamp so the sink can track clock offset.

### Key specs

| Parameter            | Value                              |
|----------------------|------------------------------------|
| Sample rate          | 48 kHz                             |
| Bit depth            | 24-bit (I2S 32-bit slot)           |
| Channels             | 2 (stereo)                         |
| Codec                | IMA ADPCM, interleaved L/R nibbles |
| Frame size           | 96 samples (~2 ms)                 |
| Packet size          | 115 bytes (19 header + 96 payload) |
| Wi-Fi channel        | 11 (configurable)                  |
| TX power             | 15 dBm (configurable)              |
| Clock source         | APLL (low-jitter)                  |

---

## Hardware

### Components

- **ESP32 dev board** (dual-core, any variant with enough GPIOs)
- **PCM1808 ADC module** -- stereo 24-bit audio ADC, I2S/left-justified output

<p align="center">
  <img src="docs/images/pcm1808.jpg" alt="PCM1808 ADC module" width="300"/>
</p>

### Wiring

| PCM1808 Pin | ESP32 GPIO | Function       |
|-------------|------------|----------------|
| SCK (BCLK)  | GPIO 27    | Bit clock       |
| WS (LRCK)   | GPIO 25    | Word select     |
| DOUT        | GPIO 26    | Serial data in  |
| SCKI / MCLK | GPIO 0     | Master clock    |
| VCC         | 3.3V       | Power           |
| GND         | GND        | Ground          |

Set the PCM1808 FMT pins for I2S / Philips mode. The ESP32 provides MCLK at 256x the sample rate (12.288 MHz) via APLL on GPIO 0.

---

## Requirements

- **ESP-IDF v5.5.2**
- An ESP32 target (not ESP32-S3). The project is configured for plain ESP32 in `sdkconfig.defaults`.

---

## Build and Flash

```bash
# Set up ESP-IDF environment (adjust path to your installation)
. $HOME/esp/esp-idf/export.sh

# Build
idf.py build

# Flash (replace /dev/ttyUSB0 with your port)
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor
```

On Windows with PowerShell:

```powershell
D:\esp-idf\export.ps1
idf.py build
idf.py -p COM10 flash monitor
```

---

## Configuration

All tunable values are `#define` constants at the top of `main/main_adpcm.c`:

| Define              | Default   | Description                                |
|---------------------|-----------|--------------------------------------------|
| `TEST_TONE_MODE`    | 0         | Set to 1 to output a 440 Hz sine instead of I2S input |
| `SAMPLE_RATE`       | 48000     | Audio sample rate in Hz                    |
| `PAYLOAD_SIZE`      | 96        | ADPCM bytes per packet (1 byte = 1 stereo sample-pair) |
| `WIFI_CHANNEL`      | 11        | Must match the sink                        |
| `WIFI_TX_POWER_QDBM` | 60      | TX power in quarter-dBm (60 = 15 dBm)     |
| `PIN_MCLK`          | GPIO 0    | I2S master clock                           |
| `PIN_BCLK`          | GPIO 27   | I2S bit clock                              |
| `PIN_WS`            | GPIO 25   | I2S word select                            |
| `PIN_DIN`           | GPIO 26   | I2S data input from PCM1808               |

---

## How It Works

1. **I2S capture** -- The ESP32 acts as I2S master, providing MCLK/BCLK/WS to the PCM1808. Audio comes in as 32-bit slots in Philips format; the upper 24 bits are extracted and clamped.

2. **ADPCM encoding** -- Each frame of 96 stereo sample-pairs is encoded into 96 bytes of IMA ADPCM. Left and right channels are interleaved per-nibble within each byte (high nibble = left, low nibble = right). The encoder maintains separate predictor/index state for each channel.

3. **Packet assembly** -- A 19-byte header is prepended with the magic marker, a sequence number, the current ADPCM predictor state for both channels, and a source timestamp in microseconds. The sink uses the predictor state to initialize its decoder (no state dependency between packets), and the timestamp for clock-sync.

4. **ESP-NOW broadcast** -- The assembled packet is sent to the broadcast MAC address. No encryption, no peer pairing. Any ESP-NOW receiver on the same channel will get it.

5. **Task pinning** -- The audio capture/encode task runs on CPU core 1 to stay off the Wi-Fi core (core 0). A low-priority status task on core 0 prints throughput and peak-level stats every 5 seconds.

---

## Serial Output

At runtime the status task logs something like:

```
I (5000) ADPCM_SRC: tx=940 ok=938 fail=2 peak=1234567 rate=188/s pps=188/s
```

- **tx** -- total packets attempted
- **ok** -- successful sends
- **fail** -- send failures (usually TX queue full, recovers on its own)
- **peak** -- peak absolute sample value (24-bit scale, max 8388607)
- **rate** -- successful sends per second
- **pps** -- packets per second

---

## Project Structure

```
esp-now-audio-source/
  CMakeLists.txt            Root project file
  sdkconfig.defaults        ESP32 build defaults (240 MHz, WDT, Wi-Fi buffers)
  main/
    CMakeLists.txt           Component registration
    idf_component.yml        IDF component dependencies
    main_adpcm.c             All source code (capture, encode, transmit)
  docs/
    images/                  Component photos for reference
```

---

## License

This project is provided as-is for personal and educational use.
