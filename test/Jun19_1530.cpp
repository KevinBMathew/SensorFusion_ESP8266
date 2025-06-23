#include <Adafruit_GFX.h>
#include <Adafruit_SH110x.h>
#include <Adafruit_BME680.h>
#include <Adafruit_SHT4x.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <time.h>
#include <SD.h>
#include <SPI.h>
#include "DFRobot_STS3X.h"

// WiFi credentials - UPDATE THESE
#define WIFI_SSID "Galaxy S3"
#define WIFI_PASSWORD "kevinshotspot"

// NTP Configuration
#define MY_NTP_SERVER "pool.ntp.org"
#define MY_TZ "IST-5:30"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define TCA_ADDR 0x70
#define EEPROM_SIZE 56
#define CLEAR_PIN 0
#define NUM_SENSORS 5
#define READINGS_PER_MINUTE 12
#define SDA_PIN 4
#define SCL_PIN 5

// SD Card pin definitions
#define SD_CS_PIN 15
#define SD_MOSI_PIN 13
#define SD_MISO_PIN 12
#define SD_SCK_PIN 14

// Kalman Filter Class
class SimpleKalmanFilter {
private:
    float Q; // Process noise covariance
    float R; // Measurement noise covariance
    float P; // Estimation error covariance
    float K; // Kalman gain
    float X; // State estimate

public:
    SimpleKalmanFilter(float process_noise, float measurement_noise, float estimation_error) {
        Q = process_noise;
        R = measurement_noise;
        P = estimation_error;
        X = 0;
    }

    float update(float measurement) {
        // Prediction step
        P = P + Q;
        // Update step
        K = P / (P + R);
        X = X + K * (measurement - X);
        P = (1 - K) * P;
        return X;
    }

    void reset() {
        X = 0;
        P = 1.0;
    }

    float getState() {
        return X;
    }
};

// Create SH1106 display object
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

Adafruit_BME680 bme;
Adafruit_SHT4x sht4;
DFRobot_STS3X sts(&Wire, STS3X_I2C_ADDRESS_B);

// Kalman filter for sensor fusion
SimpleKalmanFilter kalmanFilter(0.05, 0.05, 10.0);

// SD Card variables
bool sdCardAvailable = false;
String csvFileName = "temp_data.csv";

// Time variables
time_t now;
tm timeinfo;
bool timeInitialized = false;
unsigned long lastNTPSync = 0;
const unsigned long NTP_SYNC_INTERVAL = 3600000; // Sync every hour

// Interrupt variables
volatile bool clearEEPROM = false;

// Timing variables
unsigned long lastReadTime = 0;
const unsigned long readInterval = 5000;
unsigned long lastMemCheck = 0;

// Buffers for averaging
float sensorReadings[NUM_SENSORS][READINGS_PER_MINUTE] = {0};
uint8_t readingIndex[NUM_SENSORS] = {0};

// Fused temperature tracking
float fusedMaxTemp = -INFINITY;
float fusedMinTemp = INFINITY;
float currentFusedTemp = 0;

// Error tracking
int consecutiveFailures = 0;
int bmeFailCount = 0;
int sht4FailCount = 0;
int sts35FailCount = 0;
float lastValidSHT45Reading = NAN;

struct SensorPort {
    uint8_t port;
    float (*readFunc)();
    const char* name;
    float maxTemp;
    float minTemp;
    int eepromAddr;
    float correction;
    float weight;
};

bool displayNeedsUpdate = true;

String getFormattedTime() {
    if (!timeInitialized) {
        return "No Time";
    }
    
    time(&now);
    localtime_r(&now, &timeinfo);
    
    char timeStr[20];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(timeStr);
}

void printCurrentTime() {
    Serial.print("Current time: ");
    Serial.println(getFormattedTime());
}

// WiFi and Time Functions
void initWiFi() {
    Serial.print("Connecting to WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(1000);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("WiFi connected! IP: ");
        Serial.println(WiFi.localIP());
        
        // Initialize NTP
        configTime(MY_TZ, MY_NTP_SERVER);
        Serial.println("NTP time sync initiated");
        
        // Wait for time to be set
        int timeAttempts = 0;
        while (!time(nullptr) && timeAttempts < 10) {
            delay(1000);
            Serial.print(".");
            timeAttempts++;
        }
        
        if (time(nullptr)) {
            timeInitialized = true;
            lastNTPSync = millis();
            Serial.println("\nTime synchronized with NTP server");
            printCurrentTime();
        }
    } else {
        Serial.println("\nWiFi connection failed! Continuing without time sync.");
    }
}

void checkNTPSync() {
    if (WiFi.status() == WL_CONNECTED && 
        timeInitialized && 
        (millis() - lastNTPSync > NTP_SYNC_INTERVAL)) {
        
        Serial.println("Performing periodic NTP sync...");
        configTime(MY_TZ, MY_NTP_SERVER);
        lastNTPSync = millis();
    }
}



String getTimestamp() {
    if (!timeInitialized) {
        return String(millis()); // Fallback to millis if no time
    }
    
    time(&now);
    localtime_r(&now, &timeinfo);
    
    char timestamp[20];
    strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &timeinfo);
    return String(timestamp);
}


// SD Card Functions
bool initSDCard() {
    Serial.println("Initializing SD card...");
    
    // Initialize SPI with custom pins
    SPI.begin();
    
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD card initialization failed!");
        return false;
    }
    
    Serial.println("SD card initialized successfully");
    
    // Check if CSV file exists, if not create header
    if (!SD.exists(csvFileName)) {
        File csvFile = SD.open(csvFileName, FILE_WRITE);
        if (csvFile) {
            // Write CSV header with timestamp
            csvFile.println("Timestamp,DateTime,BME680_Port0,BME680_Port1,STS35_Port4,SHT45_Port5,STS35_Port6,Fused_Temp");
            csvFile.close();
            Serial.println("CSV file created with header");
        } else {
            Serial.println("Failed to create CSV file");
            return false;
        }
    }
    
    return true;
}

void writeToCSV(float temps[], int validCount, int validIndices[], float fusedTemp) {
    if (!sdCardAvailable) return;
    
    File csvFile = SD.open(csvFileName, FILE_WRITE);
    if (!csvFile) {
        Serial.println("Failed to open CSV file for writing");
        return;
    }
    
    // Write timestamp
    csvFile.print(getTimestamp());
    csvFile.print(",");
    
    // Write formatted date/time
    csvFile.print(getFormattedTime());
    csvFile.print(",");
    
    // Initialize array to store all sensor values
    float allSensorValues[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) {
        allSensorValues[i] = NAN;
    }
    
    // Fill in the valid readings
    for (int i = 0; i < validCount; i++) {
        int sensorIndex = validIndices[i];
        allSensorValues[sensorIndex] = temps[i];
    }
    
    // Write all sensor values in order
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (!isnan(allSensorValues[i])) {
            csvFile.print(allSensorValues[i], 2);
        } else {
            csvFile.print("NaN");
        }
        csvFile.print(",");
    }
    
    // Write fused temperature
    if (!isnan(fusedTemp)) {
        csvFile.print(fusedTemp, 2);
    } else {
        csvFile.print("NaN");
    }
    
    csvFile.println(); // End line
    csvFile.close();
    
    Serial.printf("Data logged to CSV at %s: Fused=%.2f°C\n", 
                  getFormattedTime().c_str(), fusedTemp);
}

// I2C Multiplexer Reset Function
void resetMultiplexer() {
    Serial.println("Resetting I2C multiplexer...");
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(100);
    Wire.beginTransmission(TCA_ADDR);
    if (Wire.endTransmission() == 0) {
        Serial.println("Multiplexer reset successful");
    } else {
        Serial.println("Multiplexer reset failed - hardware issue");
    }
}

// I2C Bus Recovery Function
bool recoverI2CBus() {
    Serial.println("Starting I2C bus recovery...");
    pinMode(SCL_PIN, INPUT_PULLUP);
    if (digitalRead(SCL_PIN) == LOW) {
        Serial.println("SCL held low - cannot recover");
        return false;
    }

    pinMode(SDA_PIN, INPUT_PULLUP);
    if (digitalRead(SDA_PIN) == LOW) {
        Serial.println("SDA held low - attempting recovery");
        pinMode(SCL_PIN, OUTPUT_OPEN_DRAIN);
        for (int i = 0; i < 9; i++) {
            digitalWrite(SCL_PIN, LOW);
            delayMicroseconds(5);
            digitalWrite(SCL_PIN, HIGH);
            delayMicroseconds(5);
            if (digitalRead(SDA_PIN) == HIGH) {
                break;
            }
        }
        pinMode(SDA_PIN, OUTPUT_OPEN_DRAIN);
        digitalWrite(SDA_PIN, LOW);
        delayMicroseconds(5);
        digitalWrite(SCL_PIN, HIGH);
        delayMicroseconds(2);
        digitalWrite(SDA_PIN, HIGH);
        delayMicroseconds(2);
        pinMode(SDA_PIN, INPUT_PULLUP);
        pinMode(SCL_PIN, INPUT_PULLUP);
    }

    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100);
    resetMultiplexer();
    Serial.println("I2C bus recovery completed");
    return true;
}

bool testMultiplexerChannel(uint8_t channel) {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);
    return (Wire.endTransmission() == 0);
}

void checkMemory() {
    if (millis() - lastMemCheck > 60000) {
        uint32_t free = ESP.getFreeHeap();
        Serial.printf("Free heap: %u bytes\n", free);
        if (free < 10000) {
            Serial.println("WARNING: Low memory detected!");
        }
        lastMemCheck = millis();
    }
}

void IRAM_ATTR clearEEPROMISR() {
    clearEEPROM = true;
}

void selectPort(uint8_t port) {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << port);
    Wire.endTransmission();
    delay(50);
}

float readBME680() {
    if (!bme.begin(0x76)) {
        bmeFailCount++;
        if (bmeFailCount > 5) {
            Serial.println("BME680 multiple failures - attempting I2C recovery");
            recoverI2CBus();
            bmeFailCount = 0;
        }
        return NAN;
    }
    bmeFailCount = 0;
    return bme.readTemperature();
}

float readSHT45() {
    if (!sht4.begin()) {
        sht4FailCount++;
        if (sht4FailCount > 5) {
            Serial.println("SHT45 multiple failures - attempting I2C recovery");
            recoverI2CBus();
            sht4FailCount = 0;
        }
        return NAN;
    }

    sht4FailCount = 0;
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
    sensors_event_t humidity, temp;
    for (int retry = 0; retry < 3; retry++) {
        sht4.getEvent(&humidity, &temp);
        float temperature = temp.temperature;
        if (temperature != 0.0 && temperature > -40.0 && temperature < 85.0) {
            sht4FailCount = 0;
            lastValidSHT45Reading = temperature;
            return temperature;
        }
        if (temperature == 0.0 && !isnan(lastValidSHT45Reading)) {
            Serial.printf("SHT45 returned 0.0°C, using previous valid reading: %.2f°C\n", lastValidSHT45Reading);
            return lastValidSHT45Reading;
        }
        delay(60);
    }

    sht4FailCount++;
    if (!isnan(lastValidSHT45Reading)) {
        Serial.printf("SHT45 failed all retries, using previous valid reading: %.2f°C\n", lastValidSHT45Reading);
        return lastValidSHT45Reading;
    }
    return NAN;
}

float readSTS35() {
    int i = 0;
    while(sts.begin() != true) {
        Serial.println("STS35 init failed");
        delay(100);
        i++;
        if(i == 2) {
            sts35FailCount++;
            if (sts35FailCount > 5) {
                Serial.println("STS35 multiple failures - attempting I2C recovery");
                recoverI2CBus();
                sts35FailCount = 0;
            }
            return NAN;
        }
    }
    sts35FailCount = 0;
    sts.setFreq(sts.e10Hz);
    return sts.getTemperaturePeriodC();
}

SensorPort sensors[] = {
    {0, &readBME680, "0.BME680", -INFINITY, INFINITY, 0, -0.8, 0.2},
    {1, &readBME680, "1.BME680", -INFINITY, INFINITY, 8, -1.88, 0.15},
    {4, &readSTS35, "4.STS35", -INFINITY, INFINITY, 16, -0.31, 0.25},
    {5, &readSHT45, "5.SHT45", -INFINITY, INFINITY, 24, -0.06, 0.3},
    {6, &readSTS35, "6.STS35", -INFINITY, INFINITY, 32, -0.31, 0.1},
};

float fuseTemperatures(float temps[], int validCount, int validIndices[]) {
    if (validCount == 0) return NAN;

    float weightedSum = 0;
    float totalWeight = 0;
    for (int i = 0; i < validCount; i++) {
        int idx = validIndices[i];
        weightedSum += temps[i] * sensors[idx].weight;
        totalWeight += sensors[idx].weight;
    }

    float fusedTemp = weightedSum / totalWeight;
    return kalmanFilter.update(fusedTemp);
}

void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    
    // Display current time on first line
    display.setCursor(0, 0);
    if (timeInitialized) {
        String timeStr = getFormattedTime();
        // Show only time part (HH:MM:SS) to fit on display
        int spaceIndex = timeStr.indexOf(' ');
        if (spaceIndex > 0) {
            display.print(timeStr.substring(spaceIndex + 1));
        } else {
            display.print("Time Error");
        }
    } else {
        display.print("No Time Sync");
    }
    
    // Display sensor extremes in rows with 2 decimal places
    for(int i = 0; i < NUM_SENSORS; i++) {
        int x = 0;
        int y = 12 + (i * 10); // Adjusted for time display
        display.setCursor(x, y);
        if (sensors[i].maxTemp != -INFINITY && sensors[i].minTemp != INFINITY) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%s:%.2f/%.2f",
                    sensors[i].name,
                    sensors[i].maxTemp,
                    sensors[i].minTemp);
            display.print(buffer);
        } else {
            display.printf("%s: --/--", sensors[i].name);
        }
    }

    // Display fused temperature on last line
    display.setCursor(0, 62);
    if (fusedMaxTemp != -INFINITY && fusedMinTemp != INFINITY) {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "Fused:%.2f/%.2f",
                fusedMaxTemp, fusedMinTemp);
        display.print(buffer);
    } else {
        display.print("Fused: --/--");
    }

    display.display();
    displayNeedsUpdate = false;
}

void clearAllEEPROM() {
    Serial.println("Clearing EEPROM...");
    for (int i = 0; i < EEPROM_SIZE; i++) {
        EEPROM.write(i, 0);
    }
    EEPROM.commit();
    for (auto& sensor : sensors) {
        sensor.maxTemp = -INFINITY;
        sensor.minTemp = INFINITY;
    }

    fusedMaxTemp = -INFINITY;
    fusedMinTemp = INFINITY;
    kalmanFilter.reset();
    for (int i = 0; i < NUM_SENSORS; i++) {
        readingIndex[i] = 0;
        for (int j = 0; j < READINGS_PER_MINUTE; j++) {
            sensorReadings[i][j] = 0;
        }
    }
    consecutiveFailures = 0;
    bmeFailCount = 0;
    sht4FailCount = 0;
    sts35FailCount = 0;
    displayNeedsUpdate = true;
    updateDisplay();
    Serial.println("EEPROM cleared successfully!");
}

void setup() {
    Serial.begin(115200);
    ESP.wdtEnable(8000);
    
    // Initialize WiFi and NTP first
    initWiFi();
    
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100);
    resetMultiplexer();

    display.begin(0x3C);
    display.clearDisplay();
    display.display();

    EEPROM.begin(EEPROM_SIZE);

    pinMode(CLEAR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CLEAR_PIN), clearEEPROMISR, FALLING);

    // Initialize SD card
    sdCardAvailable = initSDCard();
    if (sdCardAvailable) {
        Serial.println("SD card ready for data logging");
    } else {
        Serial.println("Continuing without SD card logging");
    }

    // Load stored values from EEPROM
    for (auto& sensor : sensors) {
        EEPROM.get(sensor.eepromAddr, sensor.maxTemp);
        EEPROM.get(sensor.eepromAddr + 4, sensor.minTemp);
        if (isnan(sensor.maxTemp)) sensor.maxTemp = -INFINITY;
        if (isnan(sensor.minTemp)) sensor.minTemp = INFINITY;
    }

    // Load fused temperature extremes
    EEPROM.get(48, fusedMaxTemp);
    EEPROM.get(52, fusedMinTemp);
    if (isnan(fusedMaxTemp)) fusedMaxTemp = -INFINITY;
    if (isnan(fusedMinTemp)) fusedMinTemp = INFINITY;

    for (int i = 0; i < NUM_SENSORS; i++) {
        readingIndex[i] = 0;
        for (int j = 0; j < READINGS_PER_MINUTE; j++) {
            sensorReadings[i][j] = 0;
        }
    }

    updateDisplay();
    Serial.println("Press and release the FLASH button to clear EEPROM");
    Serial.println("Serial commands: 'r' - show extremes, 's' - SD card status, 't' - show time");
}

bool updateEEPROM(SensorPort &sensor, float temp) {
    bool updated = false;
    if (temp > sensor.maxTemp) {
        sensor.maxTemp = temp;
        EEPROM.put(sensor.eepromAddr, temp);
        updated = true;
        Serial.printf("New MAX extreme for %s: %.2f°C\n", sensor.name, temp);
    }

    if (temp < sensor.minTemp) {
        sensor.minTemp = temp;
        EEPROM.put(sensor.eepromAddr + 4, temp);
        updated = true;
        Serial.printf("New MIN extreme for %s: %.2f°C\n", sensor.name, temp);
    }

    if (updated) {
        EEPROM.commit();
        displayNeedsUpdate = true;
    }

    return updated;
}

bool updateFusedEEPROM(float temp) {
    bool updated = false;
    if (temp > fusedMaxTemp) {
        fusedMaxTemp = temp;
        EEPROM.put(48, temp);
        updated = true;
        Serial.printf("New MAX extreme for Fused: %.2f°C\n", temp);
    }

    if (temp < fusedMinTemp) {
        fusedMinTemp = temp;
        EEPROM.put(52, temp);
        updated = true;
        Serial.printf("New MIN extreme for Fused: %.2f°C\n", temp);
    }

    if (updated) {
        EEPROM.commit();
        displayNeedsUpdate = true;
    }

    return updated;
}

void printExtremes() {
    Serial.println("\nStored Extremes:");
    for (auto& sensor : sensors) {
        Serial.printf("[%s] Max: %.2f°C Min: %.2f°C\n",
                     sensor.name, sensor.maxTemp, sensor.minTemp);
    }
    Serial.printf("[Fused] Max: %.2f°C Min: %.2f°C\n", fusedMaxTemp, fusedMinTemp);
    Serial.println();
}

void loop() {
    ESP.wdtFeed();
    unsigned long currentTime = millis();
    checkMemory();
    checkNTPSync(); // Check for periodic NTP sync

    if (clearEEPROM) {
        clearEEPROM = false;
        clearAllEEPROM();
        delay(500);
    }

    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'r') {
            printExtremes();
        } else if (cmd == 's') {
            // Print SD card status
            if (sdCardAvailable) {
                File csvFile = SD.open(csvFileName);
                if (csvFile) {
                    Serial.printf("CSV file size: %d bytes\n", csvFile.size());
                    csvFile.close();
                }
            } else {
                Serial.println("SD card not available");
            }
        } else if (cmd == 't') {
            printCurrentTime();
        }
    }

    if (currentTime - lastReadTime >= readInterval) {
        lastReadTime = currentTime;
        int currentFailures = 0;
        float validTemps[NUM_SENSORS];
        int validIndices[NUM_SENSORS];
        int validCount = 0;

        for (int i = 0; i < NUM_SENSORS; i++) {
            if (!testMultiplexerChannel(sensors[i].port)) {
                Serial.printf("Multiplexer channel %d failed - attempting reset\n", sensors[i].port);
                resetMultiplexer();
                delay(100);
            }

            selectPort(sensors[i].port);
            delay(10);
            float rawTemp = sensors[i].readFunc();
            if (!isnan(rawTemp) && rawTemp != 0.0) {
                float correctedTemp = rawTemp + sensors[i].correction;
                sensorReadings[i][readingIndex[i]] = correctedTemp;
                readingIndex[i]++;
                Serial.printf("[Port %d] %s: %.2f°C (raw: %.2f°C, correction: %.2f°C) - Reading %d/12\n",
                             sensors[i].port, sensors[i].name, correctedTemp, rawTemp,
                             sensors[i].correction, readingIndex[i]);

                if (readingIndex[i] == READINGS_PER_MINUTE) {
                    float sum = 0;
                    for (int j = 0; j < READINGS_PER_MINUTE; j++) {
                        sum += sensorReadings[i][j];
                    }
                    float avg = sum / READINGS_PER_MINUTE;
                    Serial.printf("*** 1-MINUTE AVERAGE for %s: %.2f°C ***\n", sensors[i].name, avg);
                    updateEEPROM(sensors[i], avg);
                    // Store for sensor fusion
                    validTemps[validCount] = avg;
                    validIndices[validCount] = i;
                    validCount++;
                    readingIndex[i] = 0;
                }
                consecutiveFailures = 0;
            } else {
                Serial.printf("[Port %d] %s: Error reading\n", sensors[i].port, sensors[i].name);
                currentFailures++;
            }
        }

        // Perform sensor fusion if we have valid readings
        if (validCount > 0) {
            float fusedTemp = fuseTemperatures(validTemps, validCount, validIndices);
            if (!isnan(fusedTemp)) {
                currentFusedTemp = fusedTemp;
                updateFusedEEPROM(fusedTemp);
                Serial.printf("*** KALMAN FILTERED FUSED TEMPERATURE: %.2f°C ***\n", fusedTemp);
                
                // Write to CSV file with timestamp
                writeToCSV(validTemps, validCount, validIndices, fusedTemp);
            }
        }

        if (currentFailures > 0) {
            consecutiveFailures++;
            if (consecutiveFailures >= 3) {
                Serial.printf("Multiple sensor failures detected (%d consecutive cycles)\n", consecutiveFailures);
                if (recoverI2CBus()) {
                    consecutiveFailures = 0;
                    Serial.println("I2C bus recovery completed - resuming normal operation");
                } else {
                    Serial.println("I2C bus recovery failed - system may need reset");
                }
            }
        }

        if (displayNeedsUpdate) {
            updateDisplay();
            Serial.println("Display updated with new extremes");
        }
    }
}
