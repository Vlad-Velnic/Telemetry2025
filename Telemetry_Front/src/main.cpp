// 172.20.10.4 pe iPhone
bool vb = false; // Set to 'false' to disable all Serial output

#include <Arduino.h>
#include "ArduinoLog.h"
#include "adcObj/adcObj.hpp"
#include "driver/can.h"
#include <TinyGPS++.h>
#include "aditional/aditional.hpp"
#include <HardwareSerial.h>
#include "MPU9250/MPU9250c.hpp"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Wire.h>

// Configuration Constants
#define TX_GPIO_NUM GPIO_NUM_32
#define RX_GPIO_NUM GPIO_NUM_26

#define GPS_RX_PIN GPIO_NUM_5
#define GPS_TX_PIN GPIO_NUM_4

#define LOG_LEVEL LOG_LEVEL_ERROR
#define GPS_BAUDRATE 9600
#define TELEMETRY_INTERVAL_MS 100
#define CAN_TIMEOUT_MS 0
#define MAX_CAN_RESTART_ATTEMPTS 3
#define WIFI_CONNECT_TIMEOUT_MS 10000
#define MPU_SDA_PIN 21
#define MPU_SCL_PIN 22
#define MPU9250_ADDRESS 0x68

// CAN Message IDs
#define CAN_ID_ADC 0x116
#define CAN_ID_GPS 0x117
#define CAN_ID_MPU_ACCEL 0x118
#define CAN_ID_MPU_GYRO 0x119

// Global Objects
MPU9250 mpu(Wire, MPU9250_ADDRESS);
TinyGPSPlus gps;

// Sensor Data Structure
struct SensorData
{
  // ADC values
  uint16_t steeringAngleVoltage;
  uint16_t damperLeftVoltage;
  uint16_t damperRightVoltage;

  // IMU data (accelerometer and gyroscope)
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;

  // GPS data
  double latitude, longitude;
  float speedKmh;
  bool gpsValid;
} sensorData;

// CAN Messages
twai_message_t canMessages[5]; // Index 0 unused, 1:ADC, 2:GPS, 3:ACCEL, 4:GYRO

// Timing variables
unsigned long lastTelemetryTime = 0;
uint8_t canRestartAttempts = 0;
bool canDriverActive = false;
bool mpuInitialized = false;
unsigned long lastMPUAttempt = 0;
const unsigned long MPU_RETRY_INTERVAL = 5000; // Retry every 5 seconds

// CAN Configuration
static const twai_general_config_t canGeneralConfig = {
    .mode = TWAI_MODE_NO_ACK,
    .tx_io = TX_GPIO_NUM,
    .rx_io = RX_GPIO_NUM,
    .clkout_io = TWAI_IO_UNUSED,
    .bus_off_io = TWAI_IO_UNUSED,
    .tx_queue_len = 200,
    .rx_queue_len = 10,
    .alerts_enabled = TWAI_ALERT_ALL,
    .clkout_divider = 0,
    .intr_flags = ESP_INTR_FLAG_LEVEL1};

// Methods
void initializeWiFi();
void initializeOTA();
bool initializeCAN();
void initializeCANMessages();
void readSensorData();
void readGPSData();
void readMPUData();
void prepareCANMessages();
void prepareADCMessage();
void prepareGPSMessage();
void prepareAccelerometerMessage();
void prepareGyroscopeMessage();
void transmitCANMessages();
bool restartCANDriver();
void initializeI2C();
bool initializeMPU();

static const twai_timing_config_t canTimingConfig = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t canFilterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

void setup()
{
  if (vb)
  {
    Serial.begin(115200);
  }
  Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Log.begin(LOG_LEVEL, &Serial);

  initializeI2C();
  initializeMPU();
  initializeCAN();
  initializeCANMessages();

  if (vb)
  {
    Log.noticeln("Telemetry system initialized");
    Serial.println("\n------------------------------------");
    Serial.println("  ESP32 Telemetry Module Initialized");
    Serial.println("------------------------------------\n");
  }
}

void loop()
{
  static unsigned long lastYield = 0;

  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS)
  {
    readSensorData();
    prepareCANMessages();
    transmitCANMessages();
    lastTelemetryTime = millis();
  }

  if (vb)
    delay(1750);
}

void initializeI2C()
{
  // Initialize I2C with specific pins
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C speed
  if (vb)
  {
    Log.noticeln("I2C initialized on SDA:%d, SCL:%d", MPU_SDA_PIN, MPU_SCL_PIN);
  }
}

bool initializeMPU()
{
  if (vb)
  {
    Log.noticeln("Initializing MPU9250...");
  }

  int status = mpu.begin();
  if (status < 0)
  {
    if (vb)
    {
      Log.errorln("MPU9250 initialization failed, status: %d", status);
    }
    mpuInitialized = false;
    return false;
  }

  // Configure IMU
  mpu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  mpu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  mpu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  mpu.setSrd(19); // 50 Hz

  // Quick gyro calibration
  status = mpu.calibrateGyro();
  if (status < 0)
  {
    if (vb)
    {
      Log.warningln("Gyroscope calibration failed");
    }
  }

  mpuInitialized = true;
  if (vb)
  {
    Log.noticeln("MPU9250 initialized successfully");
  }
  return true;
}

bool initializeCAN()
{
  esp_err_t status = twai_driver_install(&canGeneralConfig, &canTimingConfig,
                                         &canFilterConfig);
  if (status != ESP_OK)
  {
    if (vb)
    {
      Log.errorln("CAN driver installation failed: %s", esp_err_to_name(status));
    }
    return false;
  }

  status = twai_start();
  if (status != ESP_OK)
  {
    if (vb)
    {
      Log.errorln("CAN start failed: %s", esp_err_to_name(status));
    }
    twai_driver_uninstall();
    return false;
  }

  canDriverActive = true;
  canRestartAttempts = 0;
  if (vb)
  {
    Log.noticeln("CAN driver initialized successfully");
  }
  return true;
}

void initializeCANMessages()
{
  // ADC message (steering + dampers)
  canMessages[1].flags = TWAI_MSG_FLAG_NONE;
  canMessages[1].identifier = CAN_ID_ADC;
  canMessages[1].data_length_code = 8;

  // GPS message
  canMessages[2].flags = TWAI_MSG_FLAG_NONE;
  canMessages[2].identifier = CAN_ID_GPS;
  canMessages[2].data_length_code = 8;

  // Accelerometer message
  canMessages[3].flags = TWAI_MSG_FLAG_NONE;
  canMessages[3].identifier = CAN_ID_MPU_ACCEL; // Renamed ID
  canMessages[3].data_length_code = 6;

  // Gyroscope message
  canMessages[4].flags = TWAI_MSG_FLAG_NONE;
  canMessages[4].identifier = CAN_ID_MPU_GYRO; // Renamed ID
  canMessages[4].data_length_code = 6;

  // Initialize all data arrays to zero
  for (int i = 1; i < 5; i++)
  {
    memset(canMessages[i].data, 0, sizeof(canMessages[i].data));
  }
}

void readSensorData()
{
  // Read MPU
  readMPUData();

  // GPS data
  readGPSData();

  if (vb)
  {
    Serial.println("\n--- Sensor Data Read ---");
    Serial.printf("  Steering Angle: %u mV\n", sensorData.steeringAngleVoltage);
    Serial.printf("  Damper Left: %u mV\n", sensorData.damperLeftVoltage);
    Serial.printf("  Damper Right: %u mV\n", sensorData.damperRightVoltage);
    Serial.printf("  Accel (X,Y,Z): %.2f, %.2f, %.2f m/s²\n",
                  sensorData.accelX, sensorData.accelY, sensorData.accelZ);
    Serial.printf("  Gyro (X,Y,Z): %.2f, %.2f, %.2f rad/s\n",
                  sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ);
    if (sensorData.gpsValid)
    {
      Serial.printf("  GPS Location: Lat %.6f, Lon %.6f\n",
                    sensorData.latitude, sensorData.longitude);
      Serial.printf("  GPS Speed: %.1f km/h\n", sensorData.speedKmh);
    }
    else
    {
      Serial.println("  GPS Data: Invalid/No Fix");
    }
  }
}

void readMPUData()
{
  if (!mpuInitialized)
  {
    if (millis() - lastMPUAttempt > MPU_RETRY_INTERVAL)
    {
      if (vb)
      {
        Log.warningln("Attempting to reinitialize MPU9250...");
      }
      if (initializeMPU())
      {
        mpuInitialized = true;
      }
      lastMPUAttempt = millis();
    }

    sensorData.accelX = 0;
    sensorData.accelY = 0;
    sensorData.accelZ = 0;
    sensorData.gyroX = 0;
    sensorData.gyroY = 0;
    sensorData.gyroZ = 0;
    return;
  }

  // Small delay to prevent I2C bus flooding
  static unsigned long lastMPURead = 0;
  if (millis() - lastMPURead < 10) // 10ms delay
  {
    return;
  }
  lastMPURead = millis();

  mpu.readSensor();

  // Update sensor data
  sensorData.accelX = mpu.getAccelX_mss();
  sensorData.accelY = mpu.getAccelY_mss();
  sensorData.accelZ = mpu.getAccelZ_mss();

  // Get gyroscope data (rad/s)
  sensorData.gyroX = mpu.getGyroX_rads();
  sensorData.gyroY = mpu.getGyroY_rads();
  sensorData.gyroZ = mpu.getGyroZ_rads();
}

void readGPSData()
{
  sensorData.gpsValid = false; // Assume invalid until proven otherwise

  while (Serial2.available() > 0)
  {
    if (gps.encode(Serial2.read()))
    {
      if (gps.location.isValid())
      {
        sensorData.latitude = gps.location.lat();
        sensorData.longitude = gps.location.lng();
        sensorData.gpsValid = true;
      }
      if (gps.speed.isValid())
      {
        sensorData.speedKmh = gps.speed.kmph();
      }
    }
  }
}

void prepareCANMessages()
{
  // ADC message
  prepareADCMessage();

  // GPS message
  prepareGPSMessage();

  // Accelerometer message
  prepareAccelerometerMessage();

  // Gyroscope message
  prepareGyroscopeMessage();
}

void prepareADCMessage()
{
  canMessages[1].data[0] = sensorData.damperLeftVoltage / 100;
  canMessages[1].data[1] = sensorData.damperLeftVoltage % 100;
  canMessages[1].data[2] = sensorData.damperRightVoltage / 100;
  canMessages[1].data[3] = sensorData.damperRightVoltage % 100;
  canMessages[1].data[4] = sensorData.steeringAngleVoltage / 100;
  canMessages[1].data[5] = sensorData.steeringAngleVoltage % 100;
  canMessages[1].data[6] = 0; // Reserved
  canMessages[1].data[7] = 0; // Reserved
}

void prepareGPSMessage()
{
  // Lat and Long base for bacau
  int32_t latInt = (int32_t)((sensorData.latitude - 46) * 1000000);
  int32_t lonInt = (int32_t)((sensorData.longitude - 26) * 1000000);

  canMessages[2].data[0] = (latInt >> 16) & 0xFF;
  canMessages[2].data[1] = (latInt >> 8) & 0xFF;
  canMessages[2].data[2] = latInt & 0xFF;
  canMessages[2].data[3] = (lonInt >> 16) & 0xFF;
  canMessages[2].data[4] = (lonInt >> 8) & 0xFF;
  canMessages[2].data[5] = lonInt & 0xFF;
  canMessages[2].data[6] = sensorData.speedKmh;
}

void prepareAccelerometerMessage()
{
  convert(sensorData.accelX, canMessages[3].data);
  convert(sensorData.accelY, canMessages[3].data + 2);
  convert(sensorData.accelZ, canMessages[3].data + 4);
}

void prepareGyroscopeMessage()
{
  convert(sensorData.gyroX, canMessages[4].data);
  convert(sensorData.gyroY, canMessages[4].data + 2);
  convert(sensorData.gyroZ, canMessages[4].data + 4);
}

void transmitCANMessages()
{
  if (!canDriverActive)
  {
    if (vb)
    {
      Serial.println("CAN driver inactive - skipping transmission");
    }
    return;
  }

  if (vb)
  {
    Serial.println("\n--- Transmitting CAN Messages ---");
  }

  const char *messageNames[] = {"N/A", "ADC", "GPS", "ACCEL", "GYRO"};

  for (int i = 1; i < 5; i++)
  {
    esp_err_t status =
        twai_transmit(&canMessages[i],
                      pdMS_TO_TICKS(CAN_TIMEOUT_MS));

    if (vb)
    {
      Serial.printf("  TX %s [ID:0x%03X] [DLC:%d] Data: ", messageNames[i],
                    canMessages[i].identifier, canMessages[i].data_length_code);

      // Print data bytes in hex
      for (int j = 0; j < canMessages[i].data_length_code; j++)
      {
        Serial.printf("%02X ", canMessages[i].data[j]);
      }

      // Print decoded data for readability
      Serial.print(" | Decoded: ");
      switch (i)
      {
      case 1: // ADC
        Serial.printf("Steering:%u mV, DamperL:%u mV, DamperR:%u mV",
                      sensorData.steeringAngleVoltage,
                      sensorData.damperLeftVoltage,
                      sensorData.damperRightVoltage);
        break;

      case 2: // GPS
        if (sensorData.gpsValid)
        {
          Serial.printf("Lat:%.6f, Lon:%.6f, Speed:%.1f km/h",
                        sensorData.latitude, sensorData.longitude,
                        sensorData.speedKmh);
        }
        else
        {
          Serial.print("GPS Invalid");
        }
        break;

      case 3: // ACCEL
        Serial.printf("Accel X:%.2f, Y:%.2f, Z:%.2f m/s²",
                      sensorData.accelX, sensorData.accelY,
                      sensorData.accelZ);
        break;

      case 4: // GYRO
        Serial.printf("Gyro X:%.2f, Y:%.2f, Z:%.2f rad/s",
                      sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ);
        break;
      }

      if (status == ESP_OK)
      {
        Serial.println(" (SENT)");
      }
      else
      {
        Serial.printf(" (FAILED: %s)\n", esp_err_to_name(status));
        Log.errorln("CAN %s message failed: %s", messageNames[i],
                    esp_err_to_name(status));

        if (!restartCANDriver())
        {
          if (vb)
          {
            Log.errorln("CAN driver restart failed - disabling CAN transmission");
          }
          canDriverActive = false;
          return;
        }
      }
    }
    else
    {
      if (status != ESP_OK)
      {
        Log.errorln("CAN %s message failed: %s", messageNames[i], esp_err_to_name(status));
        if (!restartCANDriver())
        {
          canDriverActive = false;
          return;
        }
      }
    }
  }
  if (vb)
  {
    Serial.println("-----------------------------\n");
  }
}

bool restartCANDriver()
{
  if (canRestartAttempts >= MAX_CAN_RESTART_ATTEMPTS)
  {
    if (vb)
    {
      Log.errorln("Maximum CAN restart attempts reached");
    }
    return false;
  }

  canRestartAttempts++;
  if (vb)
  {
    Log.warningln("Attempting CAN driver restart (attempt %d/%d)",
                  canRestartAttempts, MAX_CAN_RESTART_ATTEMPTS);
  }

  twai_stop();
  twai_driver_uninstall();

  // Reinitialize
  if (initializeCAN())
  {
    if (vb)
    {
      Log.noticeln("CAN driver restarted successfully");
    }
    return true;
  }

  return false;
}
