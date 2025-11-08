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

    display.setCursor(2, 17);
    display.print("CAN - done");
    display.display();

    if (Config::DEBUG_SERIAL)
        Serial.println("Initializing Preferences...");
    preferences.begin(Config::PREF_NAMESPACE, false);
    logFileCounter = preferences.getUInt(Config::LOG_COUNT_KEY, 0); // Citim valoarea curentă
    if (Config::DEBUG_SERIAL)
        Serial.printf("Current log file counter in Flash: %u\n", logFileCounter);

    // RTC initialization removed

    startWiFi();
    display.setCursor(2, 24);
    unsigned long startCheck = millis();

    if (WiFi.status() != WL_CONNECTED)
    {
        // Conexiunea WiFi a eșuat rapid. Intrat în modul offline/fără sincronizare.
        NO_WIFI_MODE = true;
        timeIsSynced = false; // Confirmă că nu vom folosi NTP
        
        // Logica cerută: Incrementează și salvează contorul imediat
        logFileCounter++;
        preferences.putUInt(Config::LOG_COUNT_KEY, logFileCounter); 
        display.print("WI-FI - !error!");
        
        if (Config::DEBUG_SERIAL)
            Serial.printf("WiFi failed. Offline log counter incremented to: %u\n", logFileCounter);
    } 
    else 
    {
        // WiFi s-a conectat! Continuăm cu sincronizarea NTP (blocantă ~5s)
        NO_WIFI_MODE = false;
        if (syncNTP())
        {
            timeIsSynced = true; // Sincronizare NTP reușită
        }
        else
        {
            timeIsSynced = false; // NTP eșuat chiar și cu WiFi. Vom folosi oră nesincronizată.
            if (Config::DEBUG_SERIAL)
                Serial.println("WiFi connected but NTP failed.");
        }
        display.print("WI-FI - done");
    }

    display.display();

    if (!initializeSD())
    {
        if (Config::DEBUG_SERIAL)
            Serial.println("WARNING: SD card initialization failed");
        return false;
    }

    createLogFile();
    display.setCursor(2, 31);
    display.print("SD - done");
    display.display();

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
    display.setCursor(2, 10);
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
    WiFi.begin("Vlad's iPhone 14","coscos27");        // Start connection, do not wait
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
    // Do nothing if already created
    if (logFileCreated)
    {
        return;
    }

    // --- MODIFICARE: Logica denumirii fișierului ---
    if (NO_WIFI_MODE && !timeIsSynced)
    {
        // Calea offline: Log counter a fost deja incrementat în initialize()
        
        // Denumire fișier folosind contorul (e.g., log_001.txt)
        snprintf(filename, Config::FILENAME_SIZE,
                 "/log_%03u.txt",
                 logFileCounter);

        if (Config::DEBUG_SERIAL)
            Serial.printf("Creating log file using counter: %s\n", filename);
        
    }
    else
    {
        // Calea online (succes NTP) sau fallback-ul la ora de epoch (eșec NTP cu WiFi)
        // Folosește logica de denumire bazată pe timp (originală).

        struct tm now_tm;

        if (!localtime_r(&dateTime, &now_tm)) {
            if (Config::DEBUG_SERIAL)
                Serial.println("WARNING: Could not convert time_t to struct tm for log file name. Using default.");
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
        
        if (Config::DEBUG_SERIAL)
            Serial.printf("Creating log file using time: %s\n", filename);
    }
    // --- SFÂRȘIT MODIFICARE ---

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