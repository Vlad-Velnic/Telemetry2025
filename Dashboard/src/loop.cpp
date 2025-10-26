#include "dashboard.h"

Dashboard::Dashboard() : rtcWire(1),
                         spiOled(HSPI),
                         spiSDCard(VSPI),
                         display(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, &spiOled, Config::OLED_DC, -1, Config::OLED_CS),
                         mqttClient(wifiClient)
{
    // Initialize GPS gate points (unchanged)
    gateL = GPSPoint(Config::LEFT_GATE_LAT, Config::LEFT_GATE_LON, 0.0, 0.0);
    gateR = GPSPoint(Config::RIGHT_GATE_LAT, Config::RIGHT_GATE_LON, 0.0, 0.0);

    // Pre-compute gate vector for lap timing (unchanged)
    gateVectorX = gateR.lon - gateL.lon;
    gateVectorY = gateR.lat - gateL.lat;

    // Initialize flags and timestamps
    // Give a grace period at startup to prevent immediate timeouts
    unsigned long now = millis();
    lastRPMMessage = now;
    lastGearMessage = now;
    lastGPSMessage = now;

    NO_WIFI_MODE = true;
    NO_ECU = false; // Start as false, will be set to true by timeout
    NO_FRONT = false;
    NO_REAR = false;
}

void Dashboard::updateNetwork()
{
    unsigned long currentMillis = millis();

    // Update WiFi status flag
    bool wifi_status = (WiFi.status() == WL_CONNECTED);
    if (wifi_status == NO_WIFI_MODE) // Status changed
    {
        displayNeedsUpdate = true; // Trigger display update
    }
    NO_WIFI_MODE = !wifi_status;

    // 1. Check WiFi Connection Status
    if (WiFi.status() != WL_CONNECTED)
    {
        // Not connected, check if it's time to retry
        if (currentMillis - lastWifiAttempt > Config::WIFI_RETRY_INTERVAL)
        {
            if (Config::DEBUG_SERIAL)
                Serial.println("Retrying WiFi connection...");
            WiFi.begin(); // Re-issue begin
            lastWifiAttempt = currentMillis;
        }
    }
    else
    {
        // WiFi is connected!
        // 2. Check NTP Sync Status (only run once)
        if (!timeIsSynced)
        {
            if (syncNTP())
            {
                // NTP Success!
                timeIsSynced = true;
            }
            else
            {
                // NTP failed. Check for timeout.
                if (currentMillis - ntpAttemptStart > Config::NTP_SYNC_TIMEOUT)
                {
                    if (Config::DEBUG_SERIAL)
                        Serial.println("NTP sync timed out. Using RTC time.");
                    timeIsSynced = true; // Give up on NTP and set flag to true
                }
            }
        }
    }

    // 3. Create Log File (only run once)
    // This will run as soon as NTP is synced OR NTP times out.
    if (timeIsSynced && !logFileCreated)
    {
        createLogFile();
    }

    // 4. Handle MQTT Client
    // Needs to be called on every loop for background tasks
    if (WiFi.status() == WL_CONNECTED)
    {
        mqttClient.loop();
    }
}

void Dashboard::update()
{
    unsigned long currentMillis = millis();

    // Handle non-blocking network connections (WiFi, NTP, MQTT)
    updateNetwork();

    // Process CAN messages (highest priority)
    processCANMessages();

    // Check for CAN message timeouts
    checkCANTimeouts();

    // Update display if needed
    if (displayNeedsUpdate || (currentMillis - lastDisplayUpdate >= Config::DISPLAY_UPDATE_INTERVAL))
    {
        lastDisplayUpdate = currentMillis;
        updateDisplayClean();
        displayNeedsUpdate = false;
    }

    // Publish MQTT data
    if (currentMillis - lastMQTTPublish >= Config::MQTT_PUBLISH_INTERVAL)
    {
        lastMQTTPublish = currentMillis;
        publishMQTT();
    }

    // Flush SD buffer
    if (currentMillis - lastSDFlush >= Config::SD_FLUSH_INTERVAL)
    {
        lastSDFlush = currentMillis;
        flushSDBuffer();
    }

    // Increase dateTime (only if time is synced)
    if (timeIsSynced && (currentMillis - lastSecond >= Config::SECOND))
    {
        lastSecond = currentMillis;
        dateTime++;
    }
}

// Function to check CAN timeouts
void Dashboard::checkCANTimeouts()
{
    unsigned long currentMillis = millis();

    // Check ECU timeout (RPM)
    if (currentMillis - lastRPMMessage > Config::CAN_TIMEOUT)
    {
        if (!NO_ECU) // Only update display if state changes
            displayNeedsUpdate = true;
        NO_ECU = true;
    }

    // Check FRONT timeout (GPS)
    if (currentMillis - lastGPSMessage > Config::CAN_TIMEOUT)
    {
        if (!NO_FRONT) // Only update display if state changes
            displayNeedsUpdate = true;
        NO_FRONT = true;
    }

    // Check REAR timeout (Gear)
    if (currentMillis - lastGearMessage > Config::CAN_TIMEOUT)
    {
        if (!NO_REAR) // Only update display if state changes
            displayNeedsUpdate = true;
        NO_REAR = true;
    }
}

void Dashboard::processCANMessages()
{
    twai_message_t msg;

    // Process ALL available messages in the queue right now
    while (twai_receive(&msg, 0) == ESP_OK)
    {
        if (Config::DEBUG_SERIAL && Config::DEBUG_CAN)
        {
            Serial.printf("Can message recived with ID: 0x%X, Data: ", msg.identifier);
            for (int i = 0; i < 8; i++)
            {
                Serial.printf("%02X ", msg.data[i]);
            }
            Serial.println();
        }
        handleCANMessage(msg);
    }
    // If no messages, this while loop is skipped instantly.
}

void Dashboard::handleCANMessage(const twai_message_t &msg)
{
    // Format log line for SD and MQTT
    formatLogLine(logLine, Config::LOG_LINE_SIZE, msg.identifier, msg.data, msg.data_length_code);

    // Log to SD (only if file is open)
    if (logFile)
    {
        logToSD(logLine);
    }

    // Process message based on ID
    switch (msg.identifier)
    {
    case Config::RPM_CAN_ID:
        processRPMMessage(msg);
        break;
    case Config::GEAR_CAN_ID:
        processGearMessage(msg);
        break;
    case Config::GPS_CAN_ID:
        processGPSMessage(msg);
        break;
    case Config::TEMP_CAN_ID:
        processTempMessage(msg);
        break;
    case Config::VOLT_CAN_ID:
        processVoltMessage(msg);
        break;
    }
}

void Dashboard::processRPMMessage(const twai_message_t &msg)
{
    // Reset ECU timeout flag and timestamp
    if (NO_ECU)
        displayNeedsUpdate = true; // Update display if error is clearing
    NO_ECU = false;
    lastRPMMessage = millis();

    int newRPM = (msg.data[6] << 8) | msg.data[7];
    if (newRPM != currentRPM)
    {
        currentRPM = newRPM;
        displayNeedsUpdate = true;
    }
}

void Dashboard::processGearMessage(const twai_message_t &msg)
{
    // Reset REAR timeout flag and timestamp
    if (NO_REAR)
        displayNeedsUpdate = true; // Update display if error is clearing
    NO_REAR = false;
    lastGearMessage = millis();

    short int newGear = msg.data[4];
    if (newGear != currentGear)
    {
        currentGear = newGear;
        displayNeedsUpdate = true;
    }
}

void Dashboard::processGPSMessage(const twai_message_t &msg)
{
    // Reset FRONT timeout flag and timestamp
    if (NO_FRONT)
        displayNeedsUpdate = true; // Update display if error is clearing
    NO_FRONT = false;
    lastGPSMessage = millis();

    // Store previous location
    prevLocation = currLocation;

    // Update current location
    currLocation.timestamp = dateTime + (millis() % 1000) / 1000.0;
    // Use new constants
    currLocation.lat = Config::GPS_BASE_LAT + (msg.data[2] << 16 | msg.data[1] << 8 | msg.data[0]) / Config::GPS_SCALE_FACTOR;
    currLocation.lon = Config::GPS_BASE_LON + (msg.data[5] << 16 | msg.data[4] << 8 | msg.data[3]) / Config::GPS_SCALE_FACTOR;
    currLocation.speed = msg.data[6];

    // Check if we've moved enough to check for gate crossing
    if (TinyGPSPlus::distanceBetween(prevLocation.lat, prevLocation.lon, currLocation.lat, currLocation.lon) > 1.5)
    {
        if (getIntersectionTime(prevLocation, currLocation))
        {
            // Update lap time
            lastLapTime = currTime - prevTime;
            prevTime = currTime;
            displayNeedsUpdate = true;
        }
    }
}

void Dashboard::processTempMessage(const twai_message_t &msg)
{
    // Use new constant
    float newTemp = ((((msg.data[6] << 8) | msg.data[7]) - 32) / 1.8) / Config::TEMP_SCALE_FACTOR;
    if (abs(newTemp - currentTemp) > 0.5)
    {
        currentTemp = newTemp;
        displayNeedsUpdate = true;
    }
}

void Dashboard::processVoltMessage(const twai_message_t &msg)
{
    // Use new constant
    float newVoltage = ((msg.data[2] << 8) | msg.data[4]) / Config::VOLT_SCALE_FACTOR;
    if (abs(newVoltage - currentBatteryVoltage) > 0.1)
    {
        currentBatteryVoltage = newVoltage;
        displayNeedsUpdate = true;
    }
}

void Dashboard::updateDisplayClean()
{
    display.clearDisplay();

    display.setTextColor(SSD1306_WHITE);

    // Large gear number on left
    display.setTextSize(8);
    display.setCursor(0, 5);
    if (currentGear == 0 && !NO_REAR)
        display.print("N");
    else
        display.print(currentGear);

    // Right side time
    display.setTextSize(2);

    // Time at top right
    unsigned long totalMs = lastLapTime;
    unsigned int mins = (totalMs / 60000) % 60;
    unsigned int secs = (totalMs / 1000) % 60;
    unsigned int tenths = (totalMs / 100) % 10;

    display.setCursor(50, 8);
    display.printf("%01d:%02d:%d", mins, secs, tenths);

    // Right side temp and voltage
    display.setTextSize(1);

    if (NO_WIFI_MODE)
    {
        display.setCursor(55, 30);
        display.printf("W");
    }

    if (NO_FRONT)
    {
        display.setCursor(65, 30);
        display.printf("F");
    }

    if (NO_REAR)
    {
        display.setCursor(75, 30);
        display.printf("R");
    }

    if (NO_ECU)
    {
        display.setCursor(85, 30);
        display.printf("M");
    }

    // Temperature
    display.setCursor(100, 42);
    display.printf("%.0f%cC", currentTemp, 247);

    // RPM
    display.setCursor(51, 42);
    display.printf("%d", currentRPM);

    // Battery voltage
    display.setCursor(95, 57);
    display.printf("%.1fV", currentBatteryVoltage);

    display.display();
    return;
}

void Dashboard::showRPM(int rpm)
{
    // Map RPM to LED count
    int ledCount = map(rpm, Config::MIN_RPM, Config::MAX_RPM, 0, Config::NUM_LEDS);
    ledCount = constrain(ledCount, 0, Config::NUM_LEDS);

    // Set colors based on RPM range
    for (int i = 0; i < Config::NUM_LEDS; i++)
    {
        if (i < ledCount)
        {
            // Progressive color: green -> yellow -> red
            if (i < Config::NUM_LEDS / 3)
            {
                rpmLeds.setPixelColor(i, rpmLeds.Color(0, 255, 0)); // Green
            }
            else if (i < 2 * Config::NUM_LEDS / 3)
            {
                rpmLeds.setPixelColor(i, rpmLeds.Color(255, 255, 0)); // Yellow
            }
            else
            {
                rpmLeds.setPixelColor(i, rpmLeds.Color(255, 0, 0)); // Red
            }
        }
        else
        {
            rpmLeds.setPixelColor(i, rpmLeds.Color(0, 0, 0)); // Off
        }
    }

    rpmLeds.show();
}

void Dashboard::logToSD(const char *message)
{
    size_t msgLen = strlen(message);

    // If buffer would overflow, flush it first
    if (sdBufferPos + msgLen + 2 > Config::SD_BUFFER_SIZE)
    {
        flushSDBuffer();
    }

    // Add message to buffer
    strcpy(sdBuffer + sdBufferPos, message);
    sdBufferPos += msgLen;
    sdBuffer[sdBufferPos++] = '\n';
    sdBuffer[sdBufferPos] = '\0';
}

void Dashboard::flushSDBuffer()
{
    if (sdBufferPos > 0 && logFile)
    {
        logFile.print(sdBuffer);
        logFile.flush();
        sdBufferPos = 0;
    }
}

// Reconnect logic
void Dashboard::publishMQTT()
{
    unsigned long currentMillis = millis();

    if (!mqttClient.connected())
    {
        if (WiFi.status() == WL_CONNECTED && (currentMillis - lastMqttAttempt > Config::MQTT_RETRY_INTERVAL))
        {
            if (Config::DEBUG_SERIAL)
                Serial.println("Attempting MQTT connection...");
            lastMqttAttempt = currentMillis;
            if (mqttClient.connect("tuiasi-dashboard"))
            {
                if (Config::DEBUG_SERIAL)
                    Serial.println("MQTT connected");
            }
            else
            {
                if (Config::DEBUG_SERIAL)
                    Serial.print("MQTT failed, rc=");
                if (Config::DEBUG_SERIAL)
                    Serial.println(mqttClient.state());
            }
        }
    }
    else
    {
        // If connected, publish
        mqttClient.publish("canbus/log", logLine);
    }
}

void Dashboard::formatLogLine(char *buffer, size_t size, uint32_t id, const uint8_t *data, uint8_t length)
{
    // Format: timestamp,id,data
    int offset = snprintf(buffer, size, "%.3f,%lX,",
                          dateTime + (millis() % 1000) / 1000.0, id);

    // Add data bytes
    for (int i = 0; i < length && offset < size - 3; i++)
    {
        offset += snprintf(buffer + offset, size - offset, "%02X", data[i]);
    }
}

void Dashboard::listSDFiles()
{
    if (!SD.begin(Config::SD_CS_PIN))
    {
        if (Config::DEBUG_SERIAL)
            Serial.println("SD Card not present for file listing.");
        return;
    }

    if (Config::DEBUG_SERIAL)
        Serial.println("Files on SD card:");
    File root = SD.open("/");
    if (!root)
    {
        if (Config::DEBUG_SERIAL)
            Serial.println("Failed to open root directory");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (!file.isDirectory())
        {
            if (Config::DEBUG_SERIAL)
                Serial.print("  ");
            if (Config::DEBUG_SERIAL)
                Serial.print(file.name());
            if (Config::DEBUG_SERIAL)
                Serial.print("  (");
            if (Config::DEBUG_SERIAL)
                Serial.print(file.size());
            if (Config::DEBUG_SERIAL)
                Serial.println(" bytes)");
        }
        file = root.openNextFile();
    }

    file.close();
    root.close();
}

// Lap Timing Geometry Functions

bool Dashboard::getIntersectionTime(const GPSPoint &prev, const GPSPoint &curr)
{
    if (!doIntersect(prev, curr))
    {
        return false;
    }

    double x1 = prev.lon, y1 = prev.lat;
    double x2 = curr.lon, y2 = curr.lat;
    double x3 = gateL.lon, y3 = gateL.lat;
    double x4 = gateR.lon, y4 = gateR.lat;

    double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (denom == 0)
    {
        return false;
    }

    double interLon = ((x1 * y2 - y1 * x2) * (x3 - x4) -
                       (x1 - x2) * (x3 * y4 - y3 * x4)) /
                      denom;
    double interLat = ((x1 * y2 - y1 * x2) * (y3 - y4) -
                       (y1 - y2) * (x3 * y4 - y3 * x4)) /
                      denom;

    double distToInter = hypot(interLon - prev.lon, interLat - prev.lat);
    double avgSpeed = (prev.speed + curr.speed) / 2.0;
    if (avgSpeed < 1)
    {
        avgSpeed = 1; // avoid division by 0
    }

    double ratio = distToInter / (avgSpeed / 3.6); // km/h → m/s
    currTime = prev.timestamp + ratio * (curr.timestamp - prev.timestamp);

    return true;
}

bool Dashboard::doIntersect(const GPSPoint &p1, const GPSPoint &q1)
{
    int o1 = orientation(p1, q1, gateL);
    int o2 = orientation(p1, q1, gateR);
    int o3 = orientation(gateL, gateR, p1);
    int o4 = orientation(gateL, gateR, q1);

    if (o1 != o2 && o3 != o4)
    {
        return true;
    }

    if (o1 == 0 && onSegment(p1, gateL, q1))
        return true;
    if (o2 == 0 && onSegment(p1, gateR, q1))
        return true;
    if (o3 == 0 && onSegment(gateL, p1, gateR))
        return true;
    if (o4 == 0 && onSegment(gateL, q1, gateR))
        return true;

    return false;
}

int Dashboard::orientation(const GPSPoint &p, const GPSPoint &q, const GPSPoint &r)
{
    double val = (q.lon - p.lon) * (r.lat - q.lat) - (q.lat - p.lat) * (r.lon - q.lon);
    if (val == 0.0)
    {
        return 0; // colinear
    }
    return (val > 0.0) ? 1 : 2; // clock or counterclockwise
}

bool Dashboard::onSegment(const GPSPoint &p, const GPSPoint &q, const GPSPoint &r)
{
    return q.lon <= fmax(p.lon, r.lon) && q.lon >= fmin(p.lon, r.lon) &&
           q.lat <= fmax(p.lat, r.lat) && q.lat >= fmin(p.lat, r.lat);
}

// Global dashboard instance
Dashboard dashboard;