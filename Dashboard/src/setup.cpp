#include "dashboard.h"

bool Dashboard::initialize()
{
    Serial.begin(115200);
    if (Config::DEBUG_SERIAL)
        Serial.println("Initializing dashboard...");
    if (!initializeDisplay())
    {
        if (Config::DEBUG_SERIAL)
            Serial.println("ERROR: Display initialization failed");
        return false; // Fatal
    }

    if (!initializeCAN())
    {
        if (Config::DEBUG_SERIAL)
            Serial.println("ERROR: CAN initialization failed");
        return false; // Fatal
    }

    display.setCursor(2, 38);
    display.print("CAN - done");

    startWiFi();


    display.setCursor(2, 43);
    display.print("WI-FI - done");

    if (!initializeSD())
    {
        if (Config::DEBUG_SERIAL)
            Serial.println("WARNING: SD card initialization failed");
        return false;
    }

    display.setCursor(2, 33);
    display.print("SD - done");

    // Setup MQTT (server only, doesn't connect yet)
    mqttClient.setServer(Config::MQTT_SERVER, 1883);

    // List SD files for debugging
    if (Config::DEBUG_SERIAL)
        listSDFiles();

    if (Config::DEBUG_SERIAL)
        Serial.println("Initialization complete. Main loop starting.");

    delay(300);
    return true;
}

bool Dashboard::initializeSD()
{
    spiSDCard.begin(Config::SD_SCK_PIN, Config::SD_MISO_PIN, Config::SD_MOSI_PIN, Config::SD_CS_PIN);
    if (!SD.begin(Config::SD_CS_PIN))
    {
        if (Config::DEBUG_SERIAL)
            Serial.println("WARNING: SD card initialization failed");
        return false;
    }
    if (Config::DEBUG_SERIAL)
        Serial.println("SD card initialization successful");
    return true;
}

bool Dashboard::initializeCAN()
{
    esp_err_t status = can_driver_install(&g_config, &t_config, &f_config);
    if (status != ESP_OK)
    {
        if (Config::DEBUG_SERIAL)
            Serial.println("ERROR: CAN driver installation failed");
        return false;
    }

    status = can_start();
    if (status != ESP_OK)
    {
        if (Config::DEBUG_SERIAL)
            Serial.println("ERROR: CAN start failed");
        return false;
    }

    if (Config::DEBUG_SERIAL)
        Serial.println("CAN initialized successfully");
    return true;
}

bool Dashboard::initializeDisplay()
{
    spiOled.begin(Config::OLED_CLK, -1, Config::OLED_MOSI, Config::OLED_CS);
    if (!display.begin(SSD1306_SWITCHCAPVCC))
    {
        if (Config::DEBUG_SERIAL)
            Serial.println("ERROR: Display initialization failed");
        return false;
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(2, 28);
    display.print("T.U.Iasi Racing 252");
    display.display();
    delay(300); // Show booting message

    if (Config::DEBUG_SERIAL)
        Serial.println("Display initialized successfully");
    return true;
}

void Dashboard::startWiFi()
{
    WiFi.mode(WIFI_STA); // Set station mode
    WiFi.begin();        // Start connection, do not wait
    if (Config::DEBUG_SERIAL)
        Serial.println("Attempting WiFi connection...");
    syncNTP();
    lastWifiAttempt = millis();
    ntpAttemptStart = millis(); // Start NTP timeout timer
}

bool Dashboard::syncNTP()
{
    if (Config::DEBUG_SERIAL)
        Serial.println("WiFi connected. Syncing NTP...");
    configTime(Config::GMT_OFFSET_SEC, Config::DAYLIGHT_OFFSET_SEC, Config::NTP_SERVER);

    if (!getLocalTime(&timeinfo, 5000))
    { // 5 second timeout
        if (Config::DEBUG_SERIAL)
            Serial.println("Failed to obtain NTP time");
        return false;
    }

    // NTP Succeeded!
    // 1. Set the internal time variable
    dateTime = mktime(&timeinfo);

    // 2. Update the external RTC module with the correct time
    DateTime ntpTime = DateTime(
        timeinfo.tm_year + 1900,
        timeinfo.tm_mon + 1,
        timeinfo.tm_mday,
        timeinfo.tm_hour,
        timeinfo.tm_min,
        timeinfo.tm_sec);
    rtc.adjust(ntpTime);

    if (Config::DEBUG_SERIAL)
        Serial.println("NTP time synchronized and RTC updated");
    timeIsSynced = true; // Set the global flag
    return true;
}

void Dashboard::createLogFile()
{
    // Get the time from the RTC (which may or may not be NTP-synced)
    DateTime now = rtc.now();

    // If time hasn't been synced yet, dateTime will be 0.
    // Set it from the RTC as a fallback.
    if (!timeIsSynced)
    {
        dateTime = now.unixtime();
        if (Config::DEBUG_SERIAL)
            Serial.println("Creating log file with RTC time (NTP not synced).");
    }
    else
    {
        if (Config::DEBUG_SERIAL)
            Serial.println("Creating log file with NTP-synced time.");
    }

    snprintf(filename, Config::FILENAME_SIZE,
             "/log_%04d-%02d-%02d_%02d-%02d-%02d.txt",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());

    logFile = SD.open(filename, FILE_WRITE);
    if (logFile)
    {
        if (Config::DEBUG_SERIAL)
            Serial.print("Logging data to: ");
        if (Config::DEBUG_SERIAL)
            Serial.println(filename);

        // Write header
        logFile.println("timestamp,id,data");
    }
    else
    {
        if (Config::DEBUG_SERIAL)
            Serial.println("Error opening log file");
    }
    logFileCreated = true; // Mark as created
}