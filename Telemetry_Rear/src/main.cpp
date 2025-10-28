// 172.20.10.2 pe iPhone
bool vb = false; // Set to 'false' to disable all Serial output

#include <Arduino.h>
//#include "adcObj/adcObj.hpp"
#include "driver/can.h"
#include "aditional/aditional.hpp"
#include <WiFi.h>
#include <ArduinoOTA.h>

// Configuration Constants
#define TX_GPIO_NUM GPIO_NUM_32
#define RX_GPIO_NUM GPIO_NUM_27

// CAN Message IDs
#define CAN_ID_REAR_SENSORS 0x115

// Global status variable
esp_err_t can_operation_status;

// Sensor Data Variables
const int SENSOR_MIN_MV = 600;
const int SENSOR_MAX_MV = 3140;
const int OUTPUT_MIN = 0;
const int OUTPUT_MAX = 3300;
bool bspdActive = false;
uint16_t damperRearLeftVoltage; // Use uint16_t for voltage in mV
uint16_t damperRearRightVoltage;
uint8_t currentGear; // Gear is typically 0-6
uint8_t lastGear;
uint16_t brakePressureVoltage;

// CAN Configuration
static const twai_general_config_t g_config = {
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
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

twai_message_t tx_msg_obj;

// Function prototypes
void initializeWiFi();
void initializeOTA();
bool initializeCAN();
void readSensorData();
void prepareCANMessage();
void transmitCANMessage();
bool restartCANDriver();

void setup()
{
  if (vb)
  {
    Serial.begin(115200);
    Serial.println("\n------------------------------------");
    Serial.println("  ESP32 Rear Module Initialized");
    Serial.println("------------------------------------\n");
  }

  // Initialize GEAR pins
  pinMode(2, INPUT_PULLUP);
  pinMode(0, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);

  while (!initializeCAN())
  {
    if (vb)
    {
      Serial.println(
          "Failed to initialize CAN driver. System might not function "
          "correctly.");
    }

    delay(100);
    // Consider an infinite loop or other error handling if CAN is critical
  }

  float pot_cal_dataL[5][2] = {
      {450.0f, 0.0f}, // Min physical position = 0% output
      {850.0f, 825.0f},
      {1370.0f, 1650.0f},
      {2150.0f, 2475.0f},
      {3270.0f, 3300.0f} // Max physical position = 100% output
  };
  float pot_cal_dataR[5][2] = {
      {1550.0f, 0.0f}, // Min physical position = 0% output
      {1750.0f, 825.0f},
      {2250.0f, 1650.0f},
      {2900.0f, 2475.0f},
      {3130.0f, 3300.0f} // Max physical position = 100% output
  };
  Serial.println("Calibrating!");
  //  Prepare the CAN message object structure
  tx_msg_obj.data_length_code = 8;
  tx_msg_obj.identifier = CAN_ID_REAR_SENSORS;
  tx_msg_obj.flags = TWAI_MSG_FLAG_NONE;
}

void loop()
{

  readSensorData();
  prepareCANMessage();
  transmitCANMessage();
  delay(180);

  if (vb)
    delay(1750); // Main loop delay
}

bool initializeCAN()
{
  if (vb)
  {
    Serial.println("Initializing CAN driver...");
  }
  can_operation_status = twai_driver_install(&g_config, &t_config, &f_config);
  if (can_operation_status == ESP_OK)
  {
    if (vb)
    {
      Serial.println("CAN driver installed.");
    }
  }
  else
  {
    if (vb)
    {
      Serial.printf("CAN driver installation failed: %s\n",
                    esp_err_to_name(can_operation_status));
    }
    return false;
  }

  can_operation_status = twai_start();
  if (can_operation_status == ESP_OK)
  {
    if (vb)
    {
      Serial.println("CAN started.");
    }
  }
  else
  {
    if (vb)
    {
      Serial.printf("CAN start failed: %s\n",
                    esp_err_to_name(can_operation_status));
    }
    twai_driver_uninstall();
    return false;
  }
  return true;
}

void readSensorData()
{
  // Gear readings
  int gearInput_a1 = digitalRead(2);
  int gearInput_a2 = digitalRead(0);
  int gearInput_a3 = digitalRead(4);
  int gearInput_a4 = digitalRead(16);
  int gearInput_a5 = digitalRead(17);
  int gearInput_a6 = digitalRead(5);

  if (gearInput_a1 == 0)
    currentGear = 1;
  else if (gearInput_a2 == 0)
    currentGear = 2;
  else if (gearInput_a3 == 0)
    currentGear = 3;
  else if (gearInput_a4 == 0)
    currentGear = 4;
  else if (gearInput_a5 == 0)
    currentGear = 5;
  else if (gearInput_a6 == 0)
    currentGear = 6;
  else if(currentGear!=2)
    currentGear = 0; // Neutral

  if (lastGear == 3 && currentGear == 0)
    currentGear = 2;
  if(currentGear!=0)
    lastGear = currentGear;

  if (vb)
  {
    Serial.println("\n--- Sensor Data Read (Rear Module) ---");
    Serial.printf("  Damper Left Rear Mapped:  %u mV\n", damperRearLeftVoltage);
    Serial.printf("  Damper Right Rear Mapped: %u mV\n", damperRearRightVoltage);
    // Serial.printf("  Damper Left Rear:  %u mV\n", damperLeftRear.getVoltage_mV());
    // Serial.printf("  Damper Right Rear: %u mV\n", damperRightRear.getVoltage_mV());
    Serial.printf("  Current Gear:      %u\n", currentGear);
    //Serial.printf("  Brake Pressure:    %u mV\n", brakePressureVoltage);
    Serial.printf("  BSPD Active:       %s\n", bspdActive ? "YES" : "NO");
    Serial.println("------------------------------------");
  }
}

void prepareCANMessage()
{
  // Data packing into tx_msg_obj.data array

  // Damper Left Rear (2 bytes)
  tx_msg_obj.data[0] = (damperRearLeftVoltage >> 8) & 0xFF; // High byte
  tx_msg_obj.data[1] = damperRearLeftVoltage & 0xFF;        // Low byte

  // Damper Right Rear (2 bytes)
  tx_msg_obj.data[2] = (damperRearRightVoltage >> 8) & 0xFF; // High byte
  tx_msg_obj.data[3] = damperRearRightVoltage & 0xFF;        // Low byte

  // Gear (1 byte)
  tx_msg_obj.data[4] = currentGear;

  // Brake Pressure (2 bytes)
  tx_msg_obj.data[5] = (brakePressureVoltage >> 8) & 0xFF; // High byte
  tx_msg_obj.data[6] = brakePressureVoltage & 0xFF;        // Low byte

  // BSPD Status (1 byte - boolean converted to 0 or 1)
  tx_msg_obj.data[7] = bspdActive ? 1 : 0;
}

void transmitCANMessage()
{
  can_operation_status = twai_transmit(&tx_msg_obj, pdMS_TO_TICKS(100));

  if (vb)
  {
    Serial.printf("  TX RearSensors [ID:0x%03X] [DLC:%d] Data: ",
                  tx_msg_obj.identifier, tx_msg_obj.data_length_code);

    // Print data bytes in hex
    for (int i = 0; i < tx_msg_obj.data_length_code; i++)
    {
      Serial.printf("%02X ", tx_msg_obj.data[i]);
    }

    // Print decoded data for readability
    Serial.print("| Decoded: ");
    Serial.printf("DL:%u mV, DR:%u mV, Gear:%u, BP:%u mV, BSPD:%s",
                  damperRearLeftVoltage, damperRearRightVoltage, currentGear,
                  brakePressureVoltage, bspdActive ? "Active" : "Inactive");

    if (can_operation_status == ESP_OK)
    {
      Serial.println(" (SENT)");
    }
    else
    {
      Serial.printf(" (FAILED: %s)\n",
                    esp_err_to_name(can_operation_status));
    }
  }
  else
  { // If vb is false, only log critical failures
    if (can_operation_status != ESP_OK)
    {
      // Log.errorln("CAN RearSensors message failed: %s", esp_err_to_name(can_operation_status)); // Uncomment if using ArduinoLog
    }
  }

  if (can_operation_status != ESP_OK)
  {
    if (vb)
    {
      Serial.println("Attempting to restart CAN driver...");
    }
    if (!restartCANDriver())
    {
      if (vb)
      {
        Serial.println("Failed to restart CAN driver. Transmission halted.");
      }
    }
  }
}

bool restartCANDriver()
{
  twai_stop();
  twai_driver_uninstall();
  return initializeCAN(); // Re-initialize CAN
}
