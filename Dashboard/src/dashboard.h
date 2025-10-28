#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_NeoPixel.h>
#include <SD.h>
#include <SPI.h>
#include "driver/can.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <cmath>
#include <TinyGPS++.h>
//#include <ArduinoOTA.h>
#include <time.h>
#include <Fonts/FreeSansBold18pt7b.h>

// Configuration namespace to organize constants
namespace Config
{
    // Hardware constants
    constexpr uint8_t NUM_LEDS = 8;
    constexpr uint16_t MAX_RPM = 8000;
    constexpr uint16_t MIN_RPM = 4000;
    constexpr uint16_t SCREEN_WIDTH = 128;
    constexpr uint16_t SCREEN_HEIGHT = 64;

    // CAN IDs
    constexpr uint32_t RPM_CAN_ID = 0x05F0;
    constexpr uint32_t GEAR_CAN_ID = 0x0115;
    constexpr uint32_t TEMP_CAN_ID = 0x05F2;
    constexpr uint32_t VOLT_CAN_ID = 0x05F3;
    constexpr uint32_t GPS_CAN_ID = 0x0117;

    // Network settings
    constexpr const char *MQTT_SERVER = "broker.hivemq.com";
    constexpr const char *NTP_SERVER = "pool.ntp.org";
    constexpr const char *NTP_SERVER_SECONDARY = "time.google.com"; //
    constexpr const char *NTP_SERVER_TERTIARY = "europe.pool.ntp.org"; //
    constexpr int32_t GMT_OFFSET_SEC = 7200; // România summer
    constexpr int32_t DAYLIGHT_OFFSET_SEC = 0;
    constexpr unsigned long WIFI_RETRY_INTERVAL = 10000; // 10 seconds
    constexpr unsigned long MQTT_RETRY_INTERVAL = 5000;  // 5 seconds
    constexpr unsigned long NTP_SYNC_TIMEOUT = 30000;    // 30 seconds to wait for NTP

    // GPS gate coordinates
    constexpr double LEFT_GATE_LAT = 46.525579;
    constexpr double LEFT_GATE_LON = 26.942636;
    constexpr double RIGHT_GATE_LAT = 46.525648;
    constexpr double RIGHT_GATE_LON = 26.942909;

    // Magic number constants for CAN processing
    constexpr double TEMP_SCALE_FACTOR = 10.0;
    constexpr double VOLT_SCALE_FACTOR = 10.0;
    constexpr double GPS_SCALE_FACTOR = 1000000.0;
    constexpr double GPS_BASE_LAT = 46.0;
    constexpr double GPS_BASE_LON = 26.0;

    // Pin definitions
    constexpr gpio_num_t SD_CS_PIN = GPIO_NUM_5;
    constexpr gpio_num_t SD_SCK_PIN = GPIO_NUM_18;
    constexpr gpio_num_t SD_MISO_PIN = GPIO_NUM_19;
    constexpr gpio_num_t SD_MOSI_PIN = GPIO_NUM_23;

    constexpr gpio_num_t TX_GPIO_NUM = GPIO_NUM_32;
    constexpr gpio_num_t RX_GPIO_NUM = GPIO_NUM_33;

    constexpr uint8_t OLED_MOSI = 14;
    constexpr uint8_t OLED_CLK = 13;
    constexpr uint8_t OLED_DC = 12;
    constexpr uint8_t OLED_CS = 15;
    constexpr int8_t OLED_RESET = -1;

    constexpr gpio_num_t RTC_SDA = GPIO_NUM_21;
    constexpr gpio_num_t RTC_SCL = GPIO_NUM_22;

    // Task intervals (ms)
    constexpr unsigned long DISPLAY_UPDATE_INTERVAL = 60;
    constexpr unsigned long SD_FLUSH_INTERVAL = 70;
    constexpr unsigned long MQTT_PUBLISH_INTERVAL = 80;
    constexpr unsigned long SECOND = 1000;
    constexpr unsigned long CAN_TIMEOUT = 3000; // 3 seconds

    // Buffer sizes
    constexpr size_t LOG_LINE_SIZE = 128;
    constexpr size_t FILENAME_SIZE = 64;
    constexpr size_t SD_BUFFER_SIZE = 4096;

    // Debug flags
    static constexpr bool DEBUG_SERIAL = true;
    static constexpr bool DEBUG_CAN = true; // Verbose CAN message logging
}

// CAN configuration (unchanged)
static const can_general_config_t g_config = {
    .mode = TWAI_MODE_NO_ACK,
    .tx_io = Config::TX_GPIO_NUM,
    .rx_io = Config::RX_GPIO_NUM,
    .clkout_io = TWAI_IO_UNUSED,
    .bus_off_io = TWAI_IO_UNUSED,
    .tx_queue_len = 20,
    .rx_queue_len = 100,
    .alerts_enabled = TWAI_ALERT_ALL,
    .clkout_divider = 0,
    .intr_flags = ESP_INTR_FLAG_LEVEL1};
static const can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
static const can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

// GPS Point structure (unchanged)
struct GPSPoint
{
    double lat = 0.0;
    double lon = 0.0;
    double timestamp = 0.0;
    double speed = 0.0;
    // Constructor
    GPSPoint(double lat_ = 0.0, double lon_ = 0.0, double timestamp_ = 0.0, double speed_ = 0.0)
        : lat(lat_), lon(lon_), timestamp(timestamp_), speed(speed_) {}
};

// Dashboard class to encapsulate functionality
class Dashboard
{
public:
    Dashboard();

    // Initialization methods
    bool initialize();
    bool initializeRTC();
    bool initializeSD();
    bool initializeDisplay();
    bool initializeCAN();
    void startWiFi(); // Renamed from initializeWiFi
    bool syncNTP();   // Renamed from initializeNTP
    void createLogFile();

    // Main loop methods
    void update();
    void updateNetwork(); // New method for non-blocking network
    void processCANMessages();
    void updateDisplay();
    void publishMQTT();
    void flushSD();

    // CAN message handlers (unchanged)
    void handleCANMessage(const twai_message_t &msg);
    void processRPMMessage(const twai_message_t &msg);
    void processGearMessage(const twai_message_t &msg);
    void processGPSMessage(const twai_message_t &msg);
    void processTempMessage(const twai_message_t &msg);
    void processVoltMessage(const twai_message_t &msg);

    // Lap timing methods (unchanged)
    bool checkGateCrossing();
    bool getIntersectionTime(const GPSPoint &prev, const GPSPoint &curr);
    bool doIntersect(const GPSPoint &p1, const GPSPoint &q1);
    int orientation(const GPSPoint &p, const GPSPoint &q, const GPSPoint &r);
    bool onSegment(const GPSPoint &p, const GPSPoint &q, const GPSPoint &r);

    // Display methods (unchanged)
    void updateDisplayClean();
    void showRPM(int rpm);

    // SD card methods (unchanged)
    void logToSD(const char *message);
    void flushSDBuffer();
    void listSDFiles();

    // Utility methods (unchanged)
    void formatLogLine(char *buffer, size_t size, uint32_t id, const uint8_t *data, uint8_t length);
    char *formatTimestamp(char *buffer, size_t size, const DateTime &dt);

private:
    void checkCANTimeouts();

    // Hardware components
    //RTC_DS1307 rtc;
    //TwoWire rtcWire;
    Adafruit_NeoPixel rpmLeds;
    SPIClass spiOled;
    SPIClass spiSDCard;
    Adafruit_SSD1306 display;
    WiFiClient wifiClient;
    PubSubClient mqttClient;

    // State variables
    time_t dateTime;
    unsigned long millisBase;
    struct tm timeinfo;

    int currentRPM = 0;
    short int currentGear = 0;
    float currentTemp = -1.0f;
    float currentBatteryVoltage = 5.5f;

    GPSPoint gateL;
    GPSPoint gateR;
    GPSPoint prevLocation;
    GPSPoint currLocation;

    double prevTime = 0.0;
    double currTime = 0.0;
    double lastLapTime = 0.0;

    // File handling
    char filename[Config::FILENAME_SIZE];
    File logFile;
    char logLine[Config::LOG_LINE_SIZE];

    // SD buffer
    char sdBuffer[Config::SD_BUFFER_SIZE];
    size_t sdBufferPos = 0;

    // Timing variables
    unsigned long lastDisplayUpdate = 0;
    unsigned long lastSDFlush = 0;
    unsigned long lastSecond = 0;
    unsigned long lastMQTTPublish = 0;
    unsigned long lastMqttAttempt = 0;
    unsigned long lastWifiAttempt = 0;
    unsigned long ntpAttemptStart = 0;

    // Timestamps for CAN timeout checking
    unsigned long lastRPMMessage = 0;
    unsigned long lastGearMessage = 0;
    unsigned long lastGPSMessage = 0;

    // Flags
    bool displayNeedsUpdate = false;
    bool timeIsSynced = false;
    bool logFileCreated = false;

    // Error flags
    bool NO_WIFI_MODE = true;
    bool NO_ECU = true;
    bool NO_FRONT = true;
    bool NO_REAR = true;

    // Pre-computed values for lap timing
    double gateVectorX;
    double gateVectorY;
};

// Global dashboard instance
extern Dashboard dashboard;