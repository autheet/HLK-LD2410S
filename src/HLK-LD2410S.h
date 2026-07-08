/**
 * @file LD2410S.h
 * @brief Driver library for LD2410S radar sensor (motion & distance)
 * @author PhuongNam720
 * @version 1.0
 * @date 2025-07-03
 */

#pragma once
#include <Arduino.h>

/**
 * @def LD2410S_ENABLE_LOG
 * @brief Enable or disable Serial debug logging (1 = enabled, 0 = disabled)
 */
#define LD2410S_ENABLE_LOG 0

#if LD2410S_ENABLE_LOG
#define LD2410S_LOG(...) Serial.printf(__VA_ARGS__)
#define LD2410S_LOG_LINE(x) Serial.println(x)
#else
#define LD2410S_LOG(...)
#define LD2410S_LOG_LINE(x)
#endif

/**
 * @class LD2410S
 * @brief Class to communicate and configure the LD2410S radar sensor.
 */
class LD2410S
{
public:
    /**
     * @brief Constructor
     * @param serial Reference to HardwareSerial object
     * @param rxPin GPIO used for sensor TX
     * @param txPin GPIO used for sensor RX
     */
    LD2410S(HardwareSerial &serial, int rxPin, int txPin);

    /**
     * @brief Destructor
     */
    ~LD2410S();

    /**
     * @brief Configure serial and wait for valid data. Attempt to enter config mode.
     * @param baudrate 115200 or 256000
     * @return true if successful
     */
    bool begin(uint32_t baudrate = 115200);

    /**
     * @brief Configure serial for passive listening (skip config mode)
     * @param baudrate 115200 or 256000
     */
    void beginPassive(uint32_t baudrate = 115200);

    /**
     * @brief Set generic parameters like unmanned delay and reporting frequency
     * @param delay_s Unmanned delay in seconds (10 to 120)
     * @param status_report_hz Status reporting frequency in Hz (0.5 to 8.0)
     * @param range_report_hz Distance reporting frequency in Hz (0.5 to 8.0)
     * @param response_speed Response speed (5 for Normal, 10 for Fast)
     * @return true if successful
     */
    bool writeGenericConfig(uint32_t delay_s, float status_report_hz, float range_report_hz, uint32_t response_speed);

    /**
     * @brief Read and process incoming frames (non-blocking)
     */
    void loop();

    /**
     * @brief Enter configuration mode
     * @return true if success
     */
    bool enterConfigMode();

    /**
     * @brief Exit configuration mode
     * @return true if success
     */
    bool exitConfigMode();

    /**
     * @brief Switch to standard output mode (detailed)
     * @return true if success
     */
    bool switchToStandardMode();

    /**
     * @brief Switch to minimal output mode (shorter packets)
     * @return true if success
     */
    bool switchToMinimalMode();

    /**
     * @brief Read firmware version of the sensor
     * @param major Major version
     * @param minor Minor version
     * @param patch Patch version
     * @return true if response received
     */
    bool readFirmwareVersion(uint16_t &major, uint16_t &minor, uint16_t &patch);

    /**
     * @brief Read configuration parameters from sensor
     * @param farthest_distance Max detection distance
     * @param nearest_distance Min detection distance
     * @param delay_time Delay before entering static state
     * @param freq_status Report frequency for motion/static state
     * @param freq_distance Report frequency for distance
     * @param respond_speed Sensor response speed (normal/fast)
     * @return true if successful
     */
    bool readCommonParametersCommand(uint32_t &farthest_distance, uint32_t &nearest_distance, uint32_t &delay_time, uint32_t &freq_status, uint32_t &freq_distance, uint32_t &respond_speed);

    /**
     * @brief Write max and min distance (in units of distance gates, where 1 gate = 0.75m)
     * @param farthest_distance Max distance gate (1-16)
     * @param nearest_distance Min distance gate (0-16)
     * @return true if successful
     */
    bool writeDistanceConfig(uint32_t farthest_distance, uint32_t nearest_distance);

    /**
     * @brief Write trigger and hold thresholds for all distance gates (0-15)
     * @param trigger_threshold Moving sensitivity (0-100). Higher = less sensitive.
     * @param hold_threshold Static sensitivity (0-100). Higher = less sensitive.
     * @return true if successful
     */
    bool writeSensitivityConfig(uint32_t trigger_threshold, uint32_t hold_threshold);

    /**
     * @brief Write array of trigger and hold thresholds for all distance gates (0-15)
     * @param trigger_thresholds Array of 16 moving sensitivities.
     * @param hold_thresholds Array of 16 static sensitivities.
     * @return true if successful
     */
    bool writeSensitivityConfig(const uint32_t trigger_thresholds[16], const uint32_t hold_thresholds[16]);

    /**
     * @brief Read 8-byte serial number
     * @param buffer Buffer to store null-terminated serial number
     * @param len Buffer size (must be at least 9)
     * @return true if successful
     */
    bool readSerialNumber(char *buffer, size_t len);

    /**
     * @brief Check if motion is currently detected
     * @return true if motion detected
     */
    bool isMotionDetected() const;

    /**
     * @brief Get the timestamp (millis) of the last parsed frame
     * @return millis timestamp
     */
    uint32_t lastFrameTime() const;

    /**
     * @brief Get target distance
     * @return Distance in cm
     */
    uint16_t getDistance() const;

private:
    /**
     * @brief Parse minimal output frame
     * @param frame Data frame
     * @param len Length
     * @return true if successfully parsed
     */
    bool parseMinimalFrame(uint8_t *frame, size_t len);

protected:
    /**
     * @brief Write frame header: FD FC FB FA
     */
    void sendFrameHead();

    /**
     * @brief Write frame footer: 04 03 02 01
     */
    void sendFrameEnd();

    /**
     * @brief Write default configuration parameters to sensor
     * @return true if ACK received
     */
    bool writeGenericParametersCommand();

    HardwareSerial &_serial; ///< Reference to serial port
    int _rxPin;              ///< GPIO used for sensor TX
    int _txPin;              ///< GPIO used for sensor RX
    bool _motion = false;    ///< Internal motion state
    uint16_t _distance = 0;  ///< Last known distance
    uint32_t _lastFrameTime = 0; ///< Last frame millis
};