# [HLK-LD2410S](#hlk-ld2410s)
Arduino library for **low power human presence sensor**  module **HLK-LD2410S**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![GitHub stars](https://img.shields.io/github/stars/phuongnamzz/HLK-LD2410S)](https://github.com/phuongnamzz/HLK-LD2410S/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/phuongnamzz/HLK-LD2410S)](https://github.com/phuongnamzz/HLK-LD2410S/network)

![Platforms](https://img.shields.io/badge/Platforms-ESP32%20%7C%20AVR%20%7C%20STM32-green)
![Version](https://img.shields.io/github/v/release/phuongnamzz/HLK-LD2410S?color=brightgreen)
![Arduino Library](https://img.shields.io/badge/Arduino-Library-yellow)
![PlatformIO](https://img.shields.io/badge/PlatformIO-Library-orange)

📁 [📄 Official Documents & Driver Files](https://drive.google.com/drive/folders/1lCQv3mfHJ3XKXzweeHPFnJ_8_D_EWEKk)

---

## [Library structure (Doxygen)](https://phuongnamzz.github.io/HLK-LD2410S/class_l_d2410_s.html)
## [Pages html](https://phuongnamzz.github.io/HLK-LD2410S/Readme.html)
## 🧭 [Module Types](#-module-types)

<p float="left">
  <img src="https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/HLK-2410S_pinout.png" width="45%" />
  <img src="https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/LD2410S.jpg" width="45%" />
</p>

---


## 📚 Table of Contents
- [HLK-LD2410S](#hlk-ld2410s)
- [Module Types](#-module-types)
- [Overview](#overview)
- [Important Notes](#️-important-notes)
- [Pin Description](#-pin-description)
- [ESP32 Connection Table](#-esp32-connection-table)
- [Serial Protocols](#-serial-protocols)
    - [Switch the module output mode](#--switch-the-module-output-mode)
    - [Read the firmware version command](#--read-the-firmware-version-command)
    - [Enable configuration command](#--enable-configuration-command)
    - [End configuration command](#--end-configuration-command)
    - [Write Serial number command](#--write-serial-number-command)
    - [Read serial number command](#--read-serial-number-command)
    - [Write generic parameter commands](#--write-generic-parameter-commands)
    - [Read common parameter commands](#--read-common-parameter-commands)
    - [Run auto calibration command](#--run-auto-calibration-command)
    - [Read trigger thresholds command](#--read-trigger-thresholds-command)
    - [Read hold thresholds command](#--read-hold-thresholds-command)
    - [Standard frame and Energy Gates](#--standard-frame-and-energy-gates)
- [Example Code (ESP32)](#-example-code-esp32)
- [Paxcounter Wi-Fi Bridge Integration](#-paxcounter-wi-fi-bridge-integration)
## [Overview](#overview)
The **HLK-LD2410S** is a battery-powered ultra-low-power milimeter-wave sensor based on Hi-Link's milimeter-wave sensor chip.
- Intergrated intelligent milimeter wave sensor SoC 24GHz 
- Uses millimeter wave sensor distance measurement technology and ICL1112 chip
advanced proprietary radar signal processing and low power control technology to achieve accurate
perception of moving, micromotion and standing human body.

## ⚠️ [Important Notes](#️-important-notes)
- Power supply 3.0 -3.6V , recommend 3.3V
- Average working current max 0.6mA 
- Max range detection range 10m

- Interface with sensor
    Basic procedure:
    - 1.Enter the command mode. 
    - 2.Set parameter commands or obtain parameter commands. 
    - 3.Exit the command mode. 
    - HLK-LD2410S data communication uses a small-endian format, and all data in the following table is
    hexadecimal.

## 🧷 [Pin Description](#-pin-description)

![Pin description](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/pinout.png)


## 🔗 [ESP32 Connection Table](#-esp32-connection-table)

| 🆔 No | 📟 LD6002 Pin | ⚙️ Function         | 📲 ESP32 Pin   |
|:----:|:-------------:|:------------------:|:-------------:|
|  1   | **3V3**       | ⚡ Power Input      | **3V3**        |
|  2   | **GND**       | 🛑 Ground           | **GND**        |
|  3   | **TX0**       | 📤 Serial TX        | **GPIO16**     |
|  4   | **RX0**       | 📥 Serial RX        | **GPIO17**     |
|  5   | **OT2**       | 📥IO                | **NC**         |
---


## 📟 [Serial Protocols](#-serial-protocols)

### *- Baudrate: 115200_8N1*

### - [Switch the module output mode](#--switch-the-module-output-mode)
![Switch the module output mode](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/Switch%20the%20module%20output%20mode.png)

``` cpp

bool LD2410S::switchToMinimalMode()
{
    sendFrameHead();

    _serial.write(0x08);
    _serial.write(0x00);
    _serial.write(0x7A);
    _serial.write(0x00);

    const uint8_t param[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    _serial.write(param, sizeof(param));

    sendFrameEnd();

    unsigned long start = millis();
    while (millis() - start < 1000)
    {
        if (_serial.available() >= 14)
        {
            uint8_t ack[14];
            _serial.readBytes(ack, sizeof(ack));
            LD2410S_LOG_LINE("Switch to minimal output mode data:");
            for (uint8_t i = 0; i < sizeof(ack); ++i)
            {
                LD2410S_LOG("%02X ", ack[i]);
            }
            LD2410S_LOG_LINE("");
            if (ack[0] == 0xFD && ack[1] == 0xFC && ack[2] == 0xFB && ack[3] == 0xFA && ack[6] == 0x7A && ack[7] == 0x01)
            {
                LD2410S_LOG_LINE("Change to minimal data output mode success");
                return true;
            }
        }
    }
    return false;
}
```

### - [Read the firmware version command](#--read-the-firmware-version-command)
![Read the firmware version command](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/Read%20the%20firmware%20version%20command.png)

``` cpp

bool LD2410S::readFirmwareVersion(uint16_t &major, uint16_t &minor, uint16_t &patch)
{
    while (_serial.available())
        _serial.read();

    sendFrameHead();
    const uint8_t cmd[] = {0x02, 0x00, 0x00, 0x00};
    _serial.write(cmd, sizeof(cmd));
    sendFrameEnd();
    _serial.flush();

    unsigned long start = millis();
    while (millis() - start < 1000)
    {
        if (_serial.available() >= 24)
        {
            uint8_t ack[24];
            _serial.readBytes(ack, sizeof(ack));
            LD2410S_LOG_LINE("Firmware version data:");
            for (int i = 0; i < 24; i++)
            {
                LD2410S_LOG("%02X ", ack[i]);
            }
            LD2410S_LOG_LINE();

            if (ack[0] == 0xFD && ack[1] == 0xFC && ack[2] == 0xFB && ack[3] == 0xFA &&
                ack[6] == 0x00 && ack[7] == 0x01)
            {
                major = (static_cast<uint16_t>(ack[15]) << 8) | ack[14];
                minor = (static_cast<uint16_t>(ack[17]) << 8) | ack[16];
                patch = (static_cast<uint16_t>(ack[19]) << 8) | ack[18];
                LD2410S_LOG_LINE("Read firmware version  Ok");
                LD2410S_LOG("\nmajor = %d, minor = %d, patch = %d", major, minor, patch);
                delay(50);
                return true;
            }
        }
    }
    delay(50);
    return false;
}
```

### - [Enable configuration command](#--enable-configuration-command)
![Enable configuration command](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/Enable%20configuration%20command.png)

``` cpp
bool LD2410S::enterConfigMode()
{
    sendFrameHead();
    const uint8_t cmd[] = {0x04, 0x00, 0xFF, 0x00, 0x01, 0x00};
    _serial.write(cmd, sizeof(cmd));
    sendFrameEnd();

    unsigned long start = millis();

    while (millis() - start < 500)
    {
        if (_serial.available() >= 18)
        {
            uint8_t ack[18];
            _serial.readBytes(ack, sizeof(ack));
            if (ack[0] == 0xFD && ack[1] == 0xFC && ack[2] == 0xFB && ack[3] == 0xFA &&
                ack[6] == 0xFF && ack[7] == 0x01 && ack[10] == 0x03 && ack[11] == 0x00)
            {
                LD2410S_LOG_LINE("Enter config mode Ok");
                return true;
            }
        }
    }
    return false;
}
```
### - [End configuration command](#--end-configuration-command)
![End configuration command](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/End%20configuration%20command.png)

``` cpp
bool LD2410S::exitConfigMode()
{
    sendFrameHead();
    const uint8_t cmd[] = {0x02, 0x00, 0xFE, 0x00};
    _serial.write(cmd, sizeof(cmd));
    sendFrameEnd();

    unsigned long start = millis();
    while (millis() - start < 500)
    {
        if (_serial.available() >= 14)
        {
            uint8_t ack[14];
            _serial.readBytes(ack, sizeof(ack));
            if (ack[0] == 0xFD && ack[1] == 0xFC && ack[2] == 0xFB && ack[3] == 0xFA &&
                ack[6] == 0xFE && ack[7] == 0x01)
            {
                LD2410S_LOG_LINE("Exit config mode Ok");
                return true;
            }
        }
    }
    return false;
}
```

### - [Write Serial number command](#--write-serial-number-command)
![Write Serial number command](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/Write%20Serial%20number%20command.png)

``` cpp

```

### - [Read serial number command](#--read-serial-number-command)
![Read serial number command](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/Read%20serial%20number%20command.png)

``` cpp
bool LD2410S::readSerialNumber(char *buffer, size_t len)
{
    while (_serial.available())
        _serial.read();

    if (len < 9)
        return false;

    delay(100);

    sendFrameHead();
    const uint8_t cmd[] = {0x02, 0x00, 0x11, 0x00};
    _serial.write(cmd, sizeof(cmd));
    sendFrameEnd();
    _serial.flush();

    unsigned long start = millis();
    while (millis() - start < 1000)
    {
        if (_serial.available() >= 24)
        {
            uint8_t ack[24];
            _serial.readBytes(ack, sizeof(ack));

            LD2410S_LOG_LINE("Serial number data: ");
            for (int i = 0; i < 24; ++i)
                LD2410S_LOG("%02X ", ack[i]);
            LD2410S_LOG_LINE();

            if (ack[0] == 0xFD && ack[1] == 0xFC && ack[2] == 0xFB && ack[3] == 0xFA &&
                ack[6] == 0x11 && ack[7] == 0x01 &&
                ack[8] == 0x00 && ack[9] == 0x00)
            {
                for (uint8_t i = 0; i < 8; i++)
                    buffer[i] = static_cast<char>(ack[12 + i]);
                buffer[8] = '\0';
                delay(50);
                return true;
            }
        }
    }
    delay(50);
    return false;
}


```


### - [Write generic parameter commands](#--write-generic-parameter-commands)
![Write generic parameter commands](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/Write%20generic%20parameter%20commands.png)

``` cpp

bool LD2410S::writeGenericConfig(uint32_t delay_s, float status_report_hz, float range_report_hz, uint32_t response_speed)
{
    // Make sure we clear buffer
    while (_serial.available())
        _serial.read();

    sendFrameHead();

    const uint8_t cmd[] = {0x1A, 0x00, 0x70, 0x00}; // Length is 26 bytes (0x1A) for 4 parameters
    _serial.write(cmd, sizeof(cmd));

    // Parameter 1: Unmanned Delay Time
    uint8_t delay_param[6] = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00};
    delay_param[2] = delay_s & 0xFF;
    delay_param[3] = (delay_s >> 8) & 0xFF;
    _serial.write(delay_param, sizeof(delay_param));

    // Parameter 2: Status Reporting Frequency
    uint8_t freq_param[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t freq_val = (uint32_t)(status_report_hz * 10.0f);
    freq_param[2] = freq_val & 0xFF;
    freq_param[3] = (freq_val >> 8) & 0xFF;
    _serial.write(freq_param, sizeof(freq_param));

    // Parameter 3: Range Reporting Frequency
    uint8_t range_freq_param[6] = {0x0C, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t range_freq_val = (uint32_t)(range_report_hz * 10.0f);
    range_freq_param[2] = range_freq_val & 0xFF;
    range_freq_param[3] = (range_freq_val >> 8) & 0xFF;
    _serial.write(range_freq_param, sizeof(range_freq_param));

    // Parameter 4: Response Speed
    uint8_t resp_param[6] = {0x0B, 0x00, 0x00, 0x00, 0x00, 0x00};
    resp_param[2] = response_speed & 0xFF;
    resp_param[3] = (response_speed >> 8) & 0xFF;
    _serial.write(resp_param, sizeof(resp_param));

    sendFrameEnd();
    _serial.flush();

    unsigned long start = millis();
    int matched = 0;
    uint8_t ack[14];
    int ack_idx = 0;

    while (millis() - start < 1000)
    {
        if (_serial.available())
        {
            uint8_t b = _serial.read();
            if (matched == 0 && b == 0xFD) { ack[0] = b; matched = 1; ack_idx = 1; }
            else if (matched == 1 && b == 0xFC) { ack[1] = b; matched = 2; ack_idx = 2; }
            else if (matched == 2 && b == 0xFB) { ack[2] = b; matched = 3; ack_idx = 3; }
            else if (matched == 3 && b == 0xFA) { ack[3] = b; matched = 4; ack_idx = 4; }
            else if (matched >= 4) {
                ack[ack_idx++] = b;
                if (ack_idx == 14) {
                    if (ack[6] == 0x70 && ack[7] == 0x01) {
                        LD2410S_LOG_LINE("Write generic config Ok");
                        return true;
                    }
                    if (b == 0xFD) { ack[0] = b; matched = 1; ack_idx = 1; } else { matched = 0; }
                }
            } else if (b == 0xFD) {
                ack[0] = b; matched = 1; ack_idx = 1;
            } else {
                matched = 0;
            }
        }
    }
    return false;
}

bool LD2410S::writeDistanceConfig(uint32_t farthest_distance, uint32_t nearest_distance)
{
    // Make sure we clear buffer
    while (_serial.available())
        _serial.read();

    sendFrameHead();

    const uint8_t cmd[] = {0x0E, 0x00, 0x70, 0x00};
    _serial.write(cmd, sizeof(cmd));

    uint8_t far_param[6] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
    far_param[2] = farthest_distance & 0xFF;
    far_param[3] = (farthest_distance >> 8) & 0xFF;
    _serial.write(far_param, sizeof(far_param));

    uint8_t near_param[6] = {0x0A, 0x00, 0x00, 0x00, 0x00, 0x00};
    near_param[2] = nearest_distance & 0xFF;
    near_param[3] = (nearest_distance >> 8) & 0xFF;
    _serial.write(near_param, sizeof(near_param));

    sendFrameEnd();
    _serial.flush();

    // Verify ACK...
}

bool LD2410S::writeSensitivityConfig(uint32_t trigger_threshold, uint32_t hold_threshold)
{
    // Write Trigger Thresholds (Command: 0x0072)
    sendFrameHead();

    // Length is 2 bytes cmd + (16 gates * 6 bytes) = 98 bytes (0x62)
    const uint8_t trigger_cmd[] = {0x62, 0x00, 0x72, 0x00};
    _serial.write(trigger_cmd, sizeof(trigger_cmd));

    for (uint32_t i = 0; i < 16; i++) {
        uint8_t gate_param[6];
        gate_param[0] = i & 0xFF;         // Parameter word LSB
        gate_param[1] = (i >> 8) & 0xFF;  // Parameter word MSB
        gate_param[2] = trigger_threshold & 0xFF;
        gate_param[3] = (trigger_threshold >> 8) & 0xFF;
        gate_param[4] = (trigger_threshold >> 16) & 0xFF;
        gate_param[5] = (trigger_threshold >> 24) & 0xFF;
        _serial.write(gate_param, sizeof(gate_param));
    }

    sendFrameEnd();
    _serial.flush();

    // Wait for ACK
    unsigned long start = millis();
    bool ack_success = false;
    int matched = 0;
    uint8_t ack[14];
    int ack_idx = 0;

    while (millis() - start < 1000)
    {
        if (_serial.available())
        {
            uint8_t b = _serial.read();
            if (matched == 0 && b == 0xFD) { ack[0] = b; matched = 1; ack_idx = 1; }
            else if (matched == 1 && b == 0xFC) { ack[1] = b; matched = 2; ack_idx = 2; }
            else if (matched == 2 && b == 0xFB) { ack[2] = b; matched = 3; ack_idx = 3; }
            else if (matched == 3 && b == 0xFA) { ack[3] = b; matched = 4; ack_idx = 4; }
            else if (matched >= 4) {
                ack[ack_idx++] = b;
                if (ack_idx == 14) {
                    if (ack[6] == 0x72 && ack[7] == 0x01 && ack[8] == 0x00 && ack[9] == 0x00) {
                        ack_success = true;
                        break;
                    }
                    if (b == 0xFD) { ack[0] = b; matched = 1; ack_idx = 1; } else { matched = 0; }
                }
            } else if (b == 0xFD) {
                ack[0] = b; matched = 1; ack_idx = 1;
            } else {
                matched = 0;
            }
        }
    }

    if (!ack_success) return false;

    // Clear buffer just in case
    while(_serial.available()) _serial.read();

    // Write Hold Thresholds (Command: 0x0076)
    sendFrameHead();

    const uint8_t hold_cmd[] = {0x62, 0x00, 0x76, 0x00};
    _serial.write(hold_cmd, sizeof(hold_cmd));

    for (uint32_t i = 0; i < 16; i++) {
        uint8_t gate_param[6];
        gate_param[0] = i & 0xFF;
        gate_param[1] = (i >> 8) & 0xFF;
        gate_param[2] = hold_threshold & 0xFF;
        gate_param[3] = (hold_threshold >> 8) & 0xFF;
        gate_param[4] = (hold_threshold >> 16) & 0xFF;
        gate_param[5] = (hold_threshold >> 24) & 0xFF;
        _serial.write(gate_param, sizeof(gate_param));
    }

    sendFrameEnd();
    _serial.flush();

    // Wait for ACK
    start = millis();
    ack_success = false;
    matched = 0;
    ack_idx = 0;

    while (millis() - start < 1000)
    {
        if (_serial.available())
        {
            uint8_t b = _serial.read();
            if (matched == 0 && b == 0xFD) { ack[0] = b; matched = 1; ack_idx = 1; }
            else if (matched == 1 && b == 0xFC) { ack[1] = b; matched = 2; ack_idx = 2; }
            else if (matched == 2 && b == 0xFB) { ack[2] = b; matched = 3; ack_idx = 3; }
            else if (matched == 3 && b == 0xFA) { ack[3] = b; matched = 4; ack_idx = 4; }
            else if (matched >= 4) {
                ack[ack_idx++] = b;
                if (ack_idx == 14) {
                    if (ack[6] == 0x76 && ack[7] == 0x01 && ack[8] == 0x00 && ack[9] == 0x00) {
                        ack_success = true;
                        break;
                    }
                    if (b == 0xFD) { ack[0] = b; matched = 1; ack_idx = 1; } else { matched = 0; }
                }
            } else if (b == 0xFD) {
                ack[0] = b; matched = 1; ack_idx = 1;
            } else {
                matched = 0;
            }
        }
    }
    return ack_success;
}
```


### - [Read common parameter commands](#--read-common-parameter-commands)
![Read common parameter commands](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/Read%20common%20parameter%20commands.png)


``` cpp

bool LD2410S::readCommonParametersCommand(uint32_t &farthest_distance, uint32_t &nearest_distance, uint32_t &delay_time, uint32_t &freq_status, uint32_t &freq_distance, uint32_t &respond_speed)
{
    sendFrameHead();

    const uint8_t cmd[] = {
        0x0E, 0x00, 0x71, 0x00,
        0x05, 0x00, 0x0A, 0x00,
        0x06, 0x00, 0x02, 0x00,
        0x0C, 0x00, 0x0B, 0x00};
    _serial.write(cmd, sizeof(cmd));
    sendFrameEnd();

    unsigned long start = millis();

    while (millis() - start < 1000)
    {
        if (_serial.available() >= 38)
        {
            uint8_t ack[38];
            _serial.readBytes(ack, sizeof(ack));

            LD2410S_LOG_LINE("Commond parameters data: ");
            for (int i = 0; i < 38; i++)
            {
                LD2410S_LOG("%02X ", ack[i]);
            }

            if (ack[0] == 0xFD && ack[1] == 0xFC && ack[2] == 0xFB && ack[3] == 0xFA &&
                ack[6] == 0x71 && ack[7] == 0x01 && ack[8] == 0x00 && ack[9] == 0x00)
            {
                farthest_distance = parseLEUint32(&ack[10]);
                nearest_distance = parseLEUint32(&ack[14]);
                delay_time = parseLEUint32(&ack[18]);
                freq_status = parseLEUint32(&ack[22]);
                freq_distance = parseLEUint32(&ack[26]);
                respond_speed = parseLEUint32(&ack[30]);

                LD2410S_LOG("\nfarthest_distance = %d", farthest_distance);
                LD2410S_LOG("\nnearest_distance = %d", nearest_distance);
                LD2410S_LOG("\ndelay_time = %d", delay_time);
                LD2410S_LOG("\nfreq_status = %d", freq_status);
                LD2410S_LOG("\nfreq_distance = %d", freq_distance);
                LD2410S_LOG("\nrespond_speed = %d\n", respond_speed);

                return true;
            }
        }
    }
    return false;
}
```

### - [Run auto calibration command](#--run-auto-calibration-command)
Triggers the background noise calibration process.

```cpp
bool LD2410S::runAutoCalibration(uint16_t duration_s)
```

### - [Read trigger thresholds command](#--read-trigger-thresholds-command)
Reads the 16 moving trigger threshold values.

```cpp
bool LD2410S::readTriggerThresholds(uint32_t trigger_thresholds[16])
```

### - [Read hold thresholds command](#--read-hold-thresholds-command)
Reads the 16 static hold threshold values.

```cpp
bool LD2410S::readHoldThresholds(uint32_t hold_thresholds[16])
```

### - [Standard frame and Energy Gates](#--standard-frame-and-energy-gates)
When `#define LD2410S_FULL` is enabled, the library supports parsing standard frames containing energy values for all 16 distance gates.

```cpp
// Get a pointer to the 64-byte array containing the energy gates
const uint8_t* LD2410S::getEnergyGates() const
```

## 📦 [Example Code (ESP32)](#-example-code-esp32)
``` cpp

#include <Arduino.h>
#include "LD2410S.h"

// Pin definitions
constexpr int RX_PIN = 16;
constexpr int TX_PIN = 17;
constexpr size_t SERIAL_NUMBER_BUFFER_SIZE = 16;

// Sensor object
LD2410S ld2410s(Serial1, RX_PIN, TX_PIN);

// Firmware version variables
uint16_t firmwareMajor = 0;
uint16_t firmwareMinor = 0;
uint16_t firmwarePatch = 0;

// Serial number buffer
char serialNumberBuffer[SERIAL_NUMBER_BUFFER_SIZE];

// Common parameters
uint32_t farDistance = 0;
uint32_t nearDistance = 0;
uint32_t delayTime = 0;
uint32_t freqStatus = 0;
uint32_t freqDistance = 0;
uint32_t responseTime = 0;

// Timing variables
unsigned long lastStatusMillis = 0;

void setup() {
  Serial.begin(115200);
  ld2410s.begin();
  delay(1000);

  Serial.println("Starting");

  // Read firmware version
  if (ld2410s.readFirmwareVersion(firmwareMajor, firmwareMinor, firmwarePatch)) {
    Serial.printf("Firmware Version: %d.%d.%d\n", firmwareMajor, firmwareMinor, firmwarePatch);
  } else {
    Serial.println("Failed to read firmware version");
  }

  // Read serial number
  if (ld2410s.readSerialNumber(serialNumberBuffer, sizeof(serialNumberBuffer))) {
    Serial.printf("Serial Number: %s\n", serialNumberBuffer);
  } else {
    Serial.println("Failed to read serial number.");
  }

  // Read common parameters
  if (ld2410s.readCommonParametersCommand(farDistance, nearDistance, delayTime, freqStatus, freqDistance, responseTime)) {
    Serial.println("Read common parameters success");
  } else {
    Serial.println("Failed to read common parameters");
  }

  lastStatusMillis = millis();
}

void loop() {
  ld2410s.loop();

  // Print status every second
  if (millis() - lastStatusMillis >= 1000) {
    Serial.printf("Target Distance: %d cm\n", ld2410s.getDistance());
    Serial.println(ld2410s.isMotionDetected() ? "Motion" : "Static");
    lastStatusMillis = millis();
  }
}
```

```
✅ Result:
```

```
Starting
Firmware Version: 1.1.1
Serial Number: 12345678
Read common parameters success
Target Distance: 0 cm
Static
Target Distance: 0 cm
Static
Target Distance: 0 cm
Static
Target Distance: 35 cm
Motion
Target Distance: 35 cm
Motion
Target Distance: 35 cm
Motion
Target Distance: 35 cm
Motion
Target Distance: 35 cm
Motion
Target Distance: 35 cm
Motion
```

## 📡 [Paxcounter Wi-Fi Bridge Integration](#-paxcounter-wi-fi-bridge-integration)

> **⚠️ Prerequisite:** To use granular array-based threshold configurations, the HLK-LD2410S must be updated to **Firmware Version 1.1.1** or newer. You can perform this update via the official Host Tool using the Wi-Fi bridge described below.

If you are using this library within the **Paxcounter** project, the device features a transparent TCP-to-Serial bridge during the `boot delay` phase. This allows you to connect the official HLK-LD2410S host tool to the radar wirelessly over Wi-Fi without needing a physical USB connection to the sensor's UART pins.

1. Ensure `HAS_LD2410S_RADAR` is enabled in your configuration.
2. Reboot the device so it enters the Wi-Fi boot menu.
3. The device will host a TCP server on port `2323`.

### Connecting via Windows VM (VirtualBox)
Since the official HLK-LD2410S tool is Windows-only, you'll likely run it in a Windows VM. VirtualBox can natively map a raw TCP connection to a virtual COM port without requiring any extra software.

1. In VirtualBox, select your Windows VM and click **Settings**.
2. Go to the **Serial Ports** tab and select **Port 1**.
3. Check **Enable Serial Port**.
4. Set the following:
   * **Port Number:** `COM1`
   * **Port Mode:** `TCP`
   * **Path/Address:** `192.168.1.x:2323` *(replace with your ESP32's actual IP)*
5. **CRITICAL:** Check the box that says **Connect to existing pipe/socket**.
6. Start your Windows VM.

Inside Windows, open the official HLK-LD2410S tool and connect to `COM1` at `115200` baud.

#### Alternative for Mac/Linux: Using a UNIX Pipe (`socat`)
If VirtualBox's native TCP mode gives you trouble, you can use `socat` to bridge the connection via a UNIX socket. VirtualBox handles UNIX sockets much more reliably than `PTY` character devices on Mac.

**Step 1: Configure VirtualBox**
1. Set **Port Mode** to `Host Pipe`.
2. Uncheck **Connect to existing pipe/socket** (VirtualBox will create the pipe).
3. Set **Path/Address** to `/tmp/vboxRadarPipe`.
4. Start your Windows VM.

**Step 2: Run `socat` on your Mac**
Once the VM is running, VirtualBox will have created the UNIX socket. Open your Mac terminal and connect that socket to your ESP32:
```bash
socat UNIX-CONNECT:/tmp/vboxRadarPipe TCP:your-device-name.local:2323,retry=10000,interval=2
```

* **UNIX-CONNECT**: Tells `socat` to connect to the pipe VirtualBox just created.
* **retry=10000,interval=2**: Keeps trying to connect to the ESP32 in the background, making it completely hands-off when you reboot your device.

### Connecting on Windows Natively
If you are running Windows natively, use a Virtual COM port software like [HW VSP3](https://www.hw-group.com/software/hw-vsp3-virtual-serial-port) or [com0com](https://com0com.sourceforge.net/) to map the ESP32's hostname (`your-device-name.local`) or IP and port `2323` to a virtual COM port. Then, open the official HLK-LD2410S tool and select the virtual COM port.