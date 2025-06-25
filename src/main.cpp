#include <Adafruit_BME680.h>
#include <Adafruit_SHT4x.h>
#include <Adafruit_SH110X.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <cmath>
#include <time.h>
#include <Wire.h>
#include "DFRobot_STS3X.h"

// WiFi credentials - UPDATE THESE
#define WIFI_SSID "Galaxy S3"
#define WIFI_PASSWORD "kevinshotspot"

// NTP Configuration
#define MY_NTP_SERVER "pool.ntp.org"
#define MY_TZ "IST-5:30"

// Hardware Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define TCA_ADDR 0x70
#define CLEAR_PIN 0
#define SDA_PIN 4
#define SCL_PIN 5

// SD Card pin definitions
#define SD_CS_PIN 15
#define SD_MOSI_PIN 13
#define SD_MISO_PIN 12
#define SD_SCK_PIN 14

// System Configuration
#define EEPROM_SIZE 64
#define NUM_SENSORS 6
#define READINGS_PER_MINUTE 12 // 12 readings * 5s interval = 60s

// --- Kalman Filter and Anomaly Detection Parameters ---
#define WINDOW_SIZE 5
const float DROP_THRESHOLD = 1.5;
const float PROCESS_VAR = 0.01;
const float MEAS_VAR_BME = 0.25 * 0.25;   // 0.0625
const float MEAS_VAR_STS35 = 0.05 * 0.05; // 0.0025
const float MEAS_VAR_SHT45 = 0.05 * 0.05; // 0.0025

// --- Weighted Kalman Filter with Anomaly Detection Class ---
class WeightedKFAnomaly {
private:
    // Filter state
    float x; // State (fused temperature)
    float P; // State variance
    float Q; // Process variance

    // Sensor-specific parameters
    float R_diag[NUM_SENSORS];
    float weights[NUM_SENSORS];

    // Anomaly detection state
    float drop_threshold;
    float sensor_history[NUM_SENSORS][WINDOW_SIZE];
    uint8_t history_index[NUM_SENSORS];
    uint8_t history_count[NUM_SENSORS];
    float prev_valid[NUM_SENSORS];
    bool first_step;

public:
    // Constructor
    WeightedKFAnomaly(float q_val, const float r_vals[], const float w_vals[], float threshold) {
        Q = q_val;
        drop_threshold = threshold;
        memcpy(R_diag, r_vals, sizeof(R_diag));
        memcpy(weights, w_vals, sizeof(weights));
        reset();
    }

    // Reset the filter to its initial state
    void reset() {
        x = 25.0; // Initial temperature estimate
        P = 10.0; // Initial variance
        first_step = true;
        for (int i = 0; i < NUM_SENSORS; i++) {
            prev_valid[i] = 25.0; // Initialize with a reasonable default
            history_index[i] = 0;
            history_count[i] = 0;
            for (int j = 0; j < WINDOW_SIZE; j++) {
                sensor_history[i][j] = 0.0;
            }
        }
    }

private:
    // Internal function to detect and correct anomalies
    void detect_anomalies(const float z[], float z_corrected[]) {
        if (first_step) {
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (!isnan(z[i])) {
                    prev_valid[i] = z[i];
                }
                z_corrected[i] = z[i]; // Use initial reading regardless
            }
            first_step = false;
            return;
        }

        for (int i = 0; i < NUM_SENSORS; i++) {
            if (isnan(z[i])) {
                z_corrected[i] = prev_valid[i]; // Use last good value if current is invalid
                continue;
            }

            float expected_temp;
            if (history_count[i] >= 2) {
                float sum = 0;
                for (int j = 0; j < history_count[i]; j++) {
                    sum += sensor_history[i][j];
                }
                expected_temp = sum / history_count[i];
            } else {
                expected_temp = prev_valid[i];
            }

            float temp_drop = expected_temp - z[i];
            if (temp_drop > drop_threshold) {
                z_corrected[i] = prev_valid[i];
                Serial.printf("ANOMALY Sensor %d: Drop of %.1fC detected. Using %.1fC instead of %.1fC\n", i, temp_drop, prev_valid[i], z[i]);
            } else {
                z_corrected[i] = z[i];
                prev_valid[i] = z[i];

                sensor_history[i][history_index[i]] = z[i];
                history_index[i] = (history_index[i] + 1) % WINDOW_SIZE;
                if (history_count[i] < WINDOW_SIZE) {
                    history_count[i]++;
                }
            }
        }
    }

public:
    // Main filter processing step
    float step(float z[]) {
        float z_corrected[NUM_SENSORS];
        detect_anomalies(z, z_corrected);

        // Prediction
        P += Q;

        // Weighted Update (translates the NumPy vector math)
        float h_dot_h = 0;
        float w2_dot_r = 0;
        for (int i = 0; i < NUM_SENSORS; i++) {
            h_dot_h += weights[i] * weights[i];
            w2_dot_r += weights[i] * weights[i] * R_diag[i];
        }
        float S = P * h_dot_h + w2_dot_r;
        if (S == 0) return x;

        float K[NUM_SENSORS];
        for (int i = 0; i < NUM_SENSORS; i++) {
            K[i] = (P * weights[i]) / S;
        }

        float y[NUM_SENSORS];
        for (int i = 0; i < NUM_SENSORS; i++) {
            y[i] = z_corrected[i] - x;
        }

        float update_val = 0;
        for (int i = 0; i < NUM_SENSORS; i++) {
            update_val += K[i] * (weights[i] * y[i]);
        }
        x += update_val;

        float k_dot_h = 0;
        for (int i = 0; i < NUM_SENSORS; i++) {
            k_dot_h += K[i] * weights[i];
        }
        P -= k_dot_h * P;

        return x;
    }
};

// --- Global object definitions ---
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_BME680 bme;
Adafruit_SHT4x sht4;
DFRobot_STS3X sts(&Wire, STS3X_I2C_ADDRESS_B);

// --- Kalman filter parameters and instance ---
const float R_DIAG[NUM_SENSORS] = {
    MEAS_VAR_BME, MEAS_VAR_BME, MEAS_VAR_STS35,
    MEAS_VAR_SHT45, MEAS_VAR_STS35, MEAS_VAR_SHT45
};
const float SENSOR_WEIGHTS[NUM_SENSORS] = {0.1, 0.1, 0.2, 0.2, 0.2, 0.2};
WeightedKFAnomaly kalmanFilter(PROCESS_VAR, R_DIAG, SENSOR_WEIGHTS, DROP_THRESHOLD);

// --- Minute Averaging Buffers ---
float sensorReadings[NUM_SENSORS][READINGS_PER_MINUTE];
uint8_t readingIndex = 0;

// SD Card and Time variables
bool sdCardAvailable = false;
String csvFileName = "temp_data_25Jun_overnight.csv";
time_t now;
tm timeinfo;
bool timeInitialized = false;

// System state variables
volatile bool clearEEPROM = false;
unsigned long lastReadTime = 0;
const unsigned long readInterval = 5000;
unsigned long lastMemCheck = 0;
float fusedMaxTemp = -INFINITY;
float fusedMinTemp = INFINITY;
float currentFusedTemp = 0;
int consecutiveFailures = 0;
int bmeFailCount = 0;
int sht4FailCount = 0;
int sts35FailCount = 0;
float lastValidSHT45Reading[2] = {NAN, NAN};
bool displayNeedsUpdate = true;

// Sensor definition struct
struct SensorPort {
    uint8_t port;
    float (*readFunc)();
    const char* name;
    float maxTemp;
    float minTemp;
    int eepromAddr;
    float correction;
};

// Forward declarations for sensor reading functions
float readBME680();
float readSHT45Port5();
float readSHT45Port7();
float readSTS35();

SensorPort sensors[] = {
    {0, &readBME680, "0.BME680", -INFINITY, INFINITY, 0, -1.31},
    {1, &readBME680, "1.BME680", -INFINITY, INFINITY, 8, -2.18},
    {4, &readSTS35, "4.STS35", -INFINITY, INFINITY, 16, -0.93},
    {5, &readSHT45Port5, "5.SHT45", -INFINITY, INFINITY, 24, -0.36},
    {6, &readSTS35, "6.STS35", -INFINITY, INFINITY, 32, -0.91},
    {7, &readSHT45Port7, "7.SHT45", -INFINITY, INFINITY, 40, -0.5}
};

// --- Time and WiFi Functions ---
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
        // Initialize NTP and wait for sync
        configTime(MY_TZ, MY_NTP_SERVER);
        Serial.println("NTP time sync initiated");
        // Wait for time to be set with longer timeout
        int timeAttempts = 0;
        while (!time(nullptr) && timeAttempts < 30) {
            delay(1000);
            Serial.print(".");
            timeAttempts++;
        }

        if (time(nullptr)) {
            timeInitialized = true;
            Serial.println("\nTime synchronized with NTP server");
            printCurrentTime();
            // Add delay to ensure complete sync before disconnecting
            delay(15000); // 15 second delay for complete sync
            // Disconnect WiFi after successful time sync
            Serial.println("Disconnecting WiFi...");
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            Serial.println("WiFi turned off");
            printCurrentTime();
        } else {
            Serial.println("\nTime sync failed!");
        }
    } else {
        Serial.println("\nWiFi connection failed! Continuing without time sync.");
    }
}


String getTimestamp() {
    if (!timeInitialized) {
        return String(millis());
    }
    time(&now);
    localtime_r(&now, &timeinfo);
    char timestamp[20];
    strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &timeinfo);
    return String(timestamp);
}

// --- SD Card and I2C Functions ---
bool initSDCard() {
    Serial.println("Initializing SD card...");
    SPI.begin();
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD card initialization failed!");
        return false;
    }
    Serial.println("SD card initialized successfully");
    if (!SD.exists(csvFileName)) {
        File csvFile = SD.open(csvFileName, FILE_WRITE);
        if (csvFile) {
            csvFile.println("Timestamp,DateTime,BME680_Port0,BME680_Port1,STS35_Port4,SHT45_Port5,STS35_Port6,SHT45_Port7,Fused_Temp");
            csvFile.close();
            Serial.println("CSV file created with header");
        } else {
            Serial.println("Failed to create CSV file");
            return false;
        }
    }
    return true;
}

void writeToCSV(float allTemps[], float fusedTemp) {
    if (!sdCardAvailable) return;
    File csvFile = SD.open(csvFileName, FILE_WRITE);
    if (!csvFile) {
        Serial.println("Failed to open CSV file for writing");
        return;
    }

    csvFile.print(getTimestamp());
    csvFile.print(",");
    csvFile.print(getFormattedTime());
    csvFile.print(",");

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (!isnan(allTemps[i])) {
            csvFile.print(allTemps[i], 2);
        } else {
            csvFile.print("NaN");
        }
        csvFile.print(",");
    }

    if (!isnan(fusedTemp)) {
        csvFile.print(fusedTemp, 2);
    } else {
        csvFile.print("NaN");
    }
    csvFile.println();
    csvFile.close();

    Serial.printf("Data logged to CSV: Fused=%.2f°C\n", fusedTemp);
}

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
            if (digitalRead(SDA_PIN) == HIGH) break;
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

// --- Sensor Reading Functions ---
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

float readSHT45Internal(int sensorIndex) {
    if (!sht4.begin()) {
        sht4FailCount++;
        if (sht4FailCount > 5) {
            Serial.println("SHT45 multiple failures - attempting I2C recovery");
            recoverI2CBus();
            sht4FailCount = 0;
        }
        return NAN;
    }
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
    sensors_event_t humidity, temp;
    for (int retry = 0; retry < 3; retry++) {
        sht4.getEvent(&humidity, &temp);
        float temperature = temp.temperature;
        if (temperature != 0.0 && temperature > -40.0 && temperature < 85.0) {
            sht4FailCount = 0;
            lastValidSHT45Reading[sensorIndex] = temperature;
            return temperature;
        }
    }
    if (!isnan(lastValidSHT45Reading[sensorIndex])) {
        Serial.printf("SHT45 failed all retries, using previous valid reading: %.2f°C\n", lastValidSHT45Reading[sensorIndex]);
        return lastValidSHT45Reading[sensorIndex];
    }
    return NAN;
}

float readSHT45Port5() { return readSHT45Internal(0); }
float readSHT45Port7() { return readSHT45Internal(1); }

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

// --- Display and EEPROM Functions ---
void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    if (timeInitialized) {
        String timeStr = getFormattedTime();
        int spaceIndex = timeStr.indexOf(' ');
        if (spaceIndex > 0) {
            display.print(timeStr.substring(spaceIndex + 1));
        } else {
            display.print("Time Error");
        }
    } else {
        display.print("No Time Sync");
    }

    for(int i = 0; i < NUM_SENSORS; i++) {
        int x = 0;
        int y = 10 + (i * 8);
        display.setCursor(x, y);
        if (sensors[i].maxTemp != -INFINITY && sensors[i].minTemp != INFINITY) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%s:%.1f/%.1f",
                sensors[i].name, sensors[i].maxTemp, sensors[i].minTemp);
            display.print(buffer);
        } else {
            display.printf("%s: --/--", sensors[i].name);
        }
    }

    display.setCursor(0, 58);
    if (fusedMaxTemp != -INFINITY && fusedMinTemp != INFINITY) {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "Fused:%.2f/%.2f", fusedMaxTemp, fusedMinTemp);
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
    
    // Clear averaging buffers
    readingIndex = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        for (int j = 0; j < READINGS_PER_MINUTE; j++) {
            sensorReadings[i][j] = 0.0f;
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

// --- Setup Function ---
void setup() {
    Serial.begin(115200);
    ESP.wdtEnable(8000);
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
    
    sdCardAvailable = initSDCard();
    if (sdCardAvailable) {
        Serial.println("SD card ready for data logging");
    } else {
        Serial.println("Continuing without SD card logging");
    }

    // Initialize averaging buffer
    for (int i = 0; i < NUM_SENSORS; i++) {
        for (int j = 0; j < READINGS_PER_MINUTE; j++) {
            sensorReadings[i][j] = 0.0f;
        }
    }

    for (auto& sensor : sensors) {
        EEPROM.get(sensor.eepromAddr, sensor.maxTemp);
        EEPROM.get(sensor.eepromAddr + 4, sensor.minTemp);
        if (isnan(sensor.maxTemp)) sensor.maxTemp = -INFINITY;
        if (isnan(sensor.minTemp)) sensor.minTemp = INFINITY;
    }

    EEPROM.get(48, fusedMaxTemp);
    EEPROM.get(52, fusedMinTemp);
    if (isnan(fusedMaxTemp)) fusedMaxTemp = -INFINITY;
    if (isnan(fusedMinTemp)) fusedMinTemp = INFINITY;
    
    updateDisplay();
    Serial.println("Press and release the FLASH button to clear EEPROM");
    Serial.println("Serial commands: 'r' - show extremes, 's' - SD card status, 't' - show time");
}

// --- Main Loop with Minute Averaging ---
void loop() {
    ESP.wdtFeed();
    unsigned long currentTime = millis();
    checkMemory();

    if (clearEEPROM) {
        clearEEPROM = false;
        clearAllEEPROM();
        delay(500);
    }

    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'r') { printExtremes(); }
        else if (cmd == 's') {
            if (sdCardAvailable) {
                File csvFile = SD.open(csvFileName);
                if (csvFile) {
                    Serial.printf("CSV file size: %u bytes\n", csvFile.size());
                    csvFile.close();
                }
            } else {
                Serial.println("SD card not available");
            }
        }
        else if (cmd == 't') { printCurrentTime(); }
    }

    if (currentTime - lastReadTime >= readInterval) {
        lastReadTime = currentTime;

        Serial.println("----------------------------------------");
        Serial.printf("Collecting readings %d/%d for minute average...\n", readingIndex + 1, READINGS_PER_MINUTE);

        int currentCycleFailures = 0;
        for (int i = 0; i < NUM_SENSORS; i++) {
            selectPort(sensors[i].port);
            float rawTemp = sensors[i].readFunc();
            if (!isnan(rawTemp)) {
                float correctedTemp = rawTemp + sensors[i].correction;
                sensorReadings[i][readingIndex] = correctedTemp; // Store reading in buffer
                Serial.printf("  [Port %d] %s: %.2f°C\n", sensors[i].port, sensors[i].name, correctedTemp);
            } else {
                sensorReadings[i][readingIndex] = NAN; // Store NAN if read fails
                Serial.printf("  [Port %d] %s: Error reading\n", sensors[i].port, sensors[i].name);
                currentCycleFailures++;
            }
        }
        
        if (currentCycleFailures == NUM_SENSORS) {
            consecutiveFailures++;
        } else {
            consecutiveFailures = 0;
        }

        readingIndex++; // Increment the index for the next reading cycle

        // --- Minute-End Processing: Triggered when buffer is full ---
        if (readingIndex >= READINGS_PER_MINUTE) {
            Serial.println("\n*** One minute elapsed. Calculating averages and fusing data... ***");

            // Step 1: Calculate the average for each sensor over the last minute
            float minuteAverages[NUM_SENSORS];
            for (int i = 0; i < NUM_SENSORS; i++) {
                float sum = 0;
                int validReadingCount = 0;
                for (int j = 0; j < READINGS_PER_MINUTE; j++) {
                    if (!isnan(sensorReadings[i][j])) {
                        sum += sensorReadings[i][j];
                        validReadingCount++;
                    }
                }
                if (validReadingCount > 0) {
                    minuteAverages[i] = sum / validReadingCount;
                    Serial.printf("  - Avg for %s: %.2f°C (%d readings)\n", sensors[i].name, minuteAverages[i], validReadingCount);
                } else {
                    minuteAverages[i] = NAN; // No valid readings this minute
                    Serial.printf("  - Avg for %s: FAILED (0 readings)\n", sensors[i].name);
                }
            }

            // Step 2: Fuse the averaged readings using the Kalman filter
            float fusedTemp = kalmanFilter.step(minuteAverages);
            currentFusedTemp = fusedTemp;
            Serial.printf("\n*** MINUTE FUSED TEMPERATURE: %.2f°C ***\n", fusedTemp);

            // Step 3: Update EEPROM extremes and log to SD card
            updateFusedEEPROM(fusedTemp);
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (!isnan(minuteAverages[i])) {
                    updateEEPROM(sensors[i], minuteAverages[i]);
                }
            }
            writeToCSV(minuteAverages, fusedTemp); // Log the AVERAGES, not the raw readings

            // Step 4: Reset the index to start collecting for the next minute
            readingIndex = 0;
            Serial.println("\nBuffer reset. Starting new 1-minute cycle.");
        }

        if (consecutiveFailures >= 3) {
            if (recoverI2CBus()) consecutiveFailures = 0;
        }

        if (displayNeedsUpdate) {
            updateDisplay();
        }
    }
}
