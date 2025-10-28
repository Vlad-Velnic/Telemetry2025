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

    // RTC initialization removed

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
    display.setTextSize(1);
    display.setCursor(2, 28);
    display.print("T.U.Iasi Racing 252");
    display.display();
    //delay(100); // Show booting message

    if (Config::DEBUG_SERIAL)
        Serial.println("Display initialized successfully");
    return true;
}

void Dashboard::startWiFi()
{
    WiFi.mode(WIFI_STA); // Set station mode
    WiFi.begin("252","xyztuxOV");        // Start connection, do not wait
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
    configTime(Config::GMT_OFFSET_SEC, Config::DAYLIGHT_OFFSET_SEC, 
               Config::NTP_SERVER,
               Config::NTP_SERVER_SECONDARY,
               Config::NTP_SERVER_TERTIARY
               );

    if (!getLocalTime(&timeinfo, 5000))
    { // 5 second timeout
        if (Config::DEBUG_SERIAL)
            Serial.println("Failed to obtain NTP time");
        return false;
    }

    // NTP Succeeded!
    // 1. Set the internal time variable
    dateTime = mktime(&timeinfo);

    // 2. RTC update logic removed

    if (Config::DEBUG_SERIAL)
        Serial.println("NTP time synchronized");
    timeIsSynced = true; // Set the global flag
    return true;
}

void Dashboard::createLogFile()
{
    // Use the synced time (dateTime) to get current time components for file name
    struct tm now_tm;

    // Convert time_t (dateTime) to struct tm (now_tm)
    if (!localtime_r(&dateTime, &now_tm)) {
        if (Config::DEBUG_SERIAL)
            Serial.println("WARNING: Could not convert time_t to struct tm for log file name. Using default.");
        // Fallback to default/epoch time if conversion fails (unlikely after successful NTP sync)
        memset(&now_tm, 0, sizeof(now_tm));
        now_tm.tm_year = 100; // 2000
        now_tm.tm_mon = 0;    // Jan
        now_tm.tm_mday = 1;   // 1
    }
    
    if (timeIsSynced)
    {
        if (Config::DEBUG_SERIAL)
            Serial.println("Creating log file with NTP-synced time.");
    }
    else
    {
        if (Config::DEBUG_SERIAL)
            Serial.println("Creating log file with un-synced internal time.");
    }

    snprintf(filename, Config::FILENAME_SIZE,
             "/log_%04d-%02d-%02d_%02d-%02d-%02d.txt",
             now_tm.tm_year + 1900, now_tm.tm_mon + 1, now_tm.tm_mday,
             now_tm.tm_hour, now_tm.tm_min, now_tm.tm_sec);

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