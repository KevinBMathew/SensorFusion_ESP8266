#include <ESP8266WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <time.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_BME680.h>
#include <Adafruit_SHT4x.h>
#include "DFRobot_STS3X.h"
#include <QuickMedianLib.h>
#include <Arduino.h>

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
#define READINGS_PER_INTERVAL 12 // 12 readings * 5s interval = 60s

// --- Kalman Filter and Anomaly Detection Parameters ---
#define WINDOW_SIZE 5
const float DROP_THRESHOLD = 1.5;
const float PROCESS_VAR = 0.01;
const float MEAS_VAR_BME = 0.25 * 0.25; // 0.0625
const float MEAS_VAR_STS35 = 0.05 * 0.05; // 0.0025
const float MEAS_VAR_SHT45 = 0.05 * 0.05; // 0.0025

// --- Fuzzy Logic Functions ---
float m_close(float d) {
    // Full weight ≤0.1 °C, fade-out to 0.5 °C
    if (d <= 0.1) {
        return 1.0;
    }
    if (d <= 0.5) {
        return (0.5 - d) / 0.4;
    }
    return 0.0;
}

float m_moderate(float d) {
    // Bell between 0.1-0.8 °C, peak around 0.5 °C
    if (0.1 < d && d <= 0.5) {
        return (d - 0.1) / 0.4;
    }
    if (0.5 < d && d <= 0.8) {
        return (0.8 - d) / 0.3;
    }
    return 0.0;
}

float dyn_weight(float dev) {
    // High / Medium / Outlier mapped to 1.0 / 0.3 / 0
    return 1.0 * m_close(dev) + 0.3 * m_moderate(dev);
}

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

// --- Combined Kalman-Fuzzy Filter Class ---
class CombinedKalmanFuzzy {
private:
    WeightedKFAnomaly kf;
    float sensor_weights[NUM_SENSORS];
    float history[10];
    uint8_t history_index;
    uint8_t history_count;
    const float kalman_weight = 0.3;

public:
    CombinedKalmanFuzzy(float q_val, const float r_vals[], const float w_vals[], float threshold) 
        : kf(q_val, r_vals, w_vals, threshold) {
        memcpy(sensor_weights, w_vals, sizeof(sensor_weights));
        history_index = 0;
        history_count = 0;
        for (int i = 0; i < 10; i++) {
            history[i] = 0.0;
        }
    }

    void reset() {
        kf.reset();
        history_index = 0;
        history_count = 0;
        for (int i = 0; i < 10; i++) {
            history[i] = 0.0;
        }
    }

    float step(float z[]) {
        // Step 1: Get Kalman filter estimate
        float kalman_estimate = kf.step(z);
        
        // Step 2: Create augmented measurement vector (z + kalman_estimate)
        float augmented_measurements[NUM_SENSORS + 1];
        for (int i = 0; i < NUM_SENSORS; i++) {
            augmented_measurements[i] = z[i];
        }
        augmented_measurements[NUM_SENSORS] = kalman_estimate;
        
        // Step 3: Calculate median of augmented measurements
        float median_val = QuickMedian<float>::GetMedian(augmented_measurements, NUM_SENSORS + 1);
        
        // Step 4: Calculate deviations from median
        float devs[NUM_SENSORS + 1];
        for (int i = 0; i <= NUM_SENSORS; i++) {
            devs[i] = fabs(augmented_measurements[i] - median_val);
        }
        
        // Step 5: Create augmented weights (sensor weights + kalman weight)
        float augmented_weights[NUM_SENSORS + 1];
        for (int i = 0; i < NUM_SENSORS; i++) {
            augmented_weights[i] = sensor_weights[i];
        }
        augmented_weights[NUM_SENSORS] = kalman_weight;
        
        // Step 6: Apply fuzzy logic weighting
        float fuzzy_weights[NUM_SENSORS + 1];
        for (int i = 0; i <= NUM_SENSORS; i++) {
            fuzzy_weights[i] = dyn_weight(devs[i]);
        }
        
        // Step 7: Calculate final weights (fuzzy * base weights)
        float final_weights[NUM_SENSORS + 1];
        float weight_sum = 0;
        for (int i = 0; i <= NUM_SENSORS; i++) {
            final_weights[i] = fuzzy_weights[i] * augmented_weights[i];
            weight_sum += final_weights[i];
        }
        
        // Step 8: Normalize weights
        if (weight_sum > 0) {
            for (int i = 0; i <= NUM_SENSORS; i++) {
                final_weights[i] /= weight_sum;
            }
        }
        
        // Step 9: Calculate combined estimate
        float combined_estimate = 0;
        for (int i = 0; i <= NUM_SENSORS; i++) {
            combined_estimate += final_weights[i] * augmented_measurements[i];
        }
        
        // Step 10: Store in history
        history[history_index] = combined_estimate;
        history_index = (history_index + 1) % 10;
        if (history_count < 10) {
            history_count++;
        }
        
        // Debug output
        Serial.printf("Combined Filter - Kalman: %.2f°C, Median: %.2f°C, Final: %.2f°C\n", 
                     kalman_estimate, median_val, combined_estimate);
        
        return combined_estimate;
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

const float SENSOR_WEIGHTS[NUM_SENSORS] = {0.1, 0.1, 0.18, 0.18, 0.22, 0.22};

// Replace the pure Kalman filter with combined approach
CombinedKalmanFuzzy combinedFilter(PROCESS_VAR, R_DIAG, SENSOR_WEIGHTS, DROP_THRESHOLD);

// --- 30 Second Averaging Buffers ---
float sensorReadings[NUM_SENSORS][READINGS_PER_INTERVAL];
uint8_t readingIndex = 0;

// SD Card and Time variables
bool sdCardAvailable = false;
String csvFileName = "temp_data_4Jul_on.csv";
time_t now;
tm timeinfo;
bool timeInitialized = false;

// System state variables
volatile bool clearEEPROM = false;
unsigned long lastReadTime = 0;
const unsigned long readInterval = 2500;
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
    {0, &readBME680, "0.BME680", -INFINITY, INFINITY, 0, -1.95},
    {1, &readBME680, "1.BME680", -INFINITY, INFINITY, 8, -2.95},
    {4, &readSTS35, "4.STS35", -INFINITY, INFINITY, 16, -0.95},
    {5, &readSHT45Port5, "5.SHT45", -INFINITY, INFINITY, 24, -0.34},
    {6, &readSTS35, "6.STS35", -INFINITY, INFINITY, 32, -0.87},
    {7, &readSHT45Port7, "7.SHT45", -INFINITY, INFINITY, 40, -0.45}
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

bool setTimeFromCSVPlusMinutes(String csvLine, int minutesToAdd) {
    if (csvLine.length() == 0) {
        Serial.println("Empty CSV line, cannot extract time");
        return false;
    }
    
    // Find the second comma (after timestamp and datetime)
    int firstComma = csvLine.indexOf(',');
    if (firstComma == -1) return false;
    int secondComma = csvLine.indexOf(',', firstComma + 1);
    if (secondComma == -1) return false;
    
    // Extract the datetime string (YYYY-MM-DD HH:MM:SS format)
    String datetimeStr = csvLine.substring(firstComma + 1, secondComma);
    Serial.println("Extracted datetime: " + datetimeStr);
    
    // Parse the datetime string: "YYYY-MM-DD HH:MM:SS"
    if (datetimeStr.length() != 19) return false;
    
    int year = datetimeStr.substring(0, 4).toInt();
    int month = datetimeStr.substring(5, 7).toInt();
    int day = datetimeStr.substring(8, 10).toInt();
    int hour = datetimeStr.substring(11, 13).toInt();
    int minute = datetimeStr.substring(14, 16).toInt();
    int second = datetimeStr.substring(17, 19).toInt();
    
    // Convert to time_t
    tm timeStruct = {};
    timeStruct.tm_year = year - 1900;
    timeStruct.tm_mon = month - 1;
    timeStruct.tm_mday = day;
    timeStruct.tm_hour = hour;
    timeStruct.tm_min = minute;
    timeStruct.tm_sec = second;
    
    time_t lastTime = mktime(&timeStruct);
    if (lastTime == -1) {
        Serial.println("Failed to convert time");
        return false;
    }
    
    // Add the specified minutes (convert to seconds)
    time_t newTime = lastTime + (minutesToAdd * 60);
    
    // Set the system time
    timeval tv = { newTime, 0 };
    settimeofday(&tv, NULL);
    timeInitialized = true;
    
    Serial.printf("Time set from CSV + %d minutes: ", minutesToAdd);
    printCurrentTime();
    return true;
}

String readLastCSVLine() {
    if (!sdCardAvailable) {
        Serial.println("SD card not available for reading last line");
        return "";
    }
    
    File csvFile = SD.open(csvFileName, FILE_READ);
    if (!csvFile) {
        Serial.println("Failed to open CSV file for reading last line");
        return "";
    }
    
    size_t fileSize = csvFile.size();
    if (fileSize <= 100) { // Skip if file is too small (just header)
        csvFile.close();
        return "";
    }
    
    String lastLine = "";
    char ch;
    int pos = fileSize - 1;
    
    // Skip any trailing newlines at the end
    csvFile.seek(pos);
    while (pos >= 0) {
        ch = csvFile.read();
        if (ch != '\n' && ch != '\r') {
            break;
        }
        pos--;
        if (pos >= 0) csvFile.seek(pos);
    }
    
    // Read backwards until we find a newline or reach beginning
    while (pos >= 0) {
        csvFile.seek(pos);
        ch = csvFile.read();
        if (ch == '\n' || ch == '\r') {
            break;
        }
        lastLine = String(ch) + lastLine;
        pos--;
    }
    
    csvFile.close();
    Serial.println("Last CSV line: " + lastLine);
    return lastLine;
}

void attemptCSVTimeFallback() {
    Serial.println("Attempting to set time from last CSV entry + 2 minutes...");
    // Ensure SD card is initialized
    if (!sdCardAvailable) {
        Serial.println("SD card not available for time fallback");
        return;
    }
    
    String lastLine = readLastCSVLine();
    if (lastLine.length() > 0) {
        if (setTimeFromCSVPlusMinutes(lastLine, 2)) {
            Serial.println("Successfully set time from CSV file + 2 minutes");
        } else {
            Serial.println("Failed to parse time from CSV file");
        }
    } else {
        Serial.println("Could not read last line from CSV file");
    }
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
            delay(15000);
            Serial.println("Disconnecting WiFi...");
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            Serial.println("WiFi turned off");
            printCurrentTime();
        } else {
            Serial.println("\nTime sync failed!");
            // Fall back to CSV time method
            attemptCSVTimeFallback();
        }
    } else {
        Serial.println("\nWiFi connection failed!");
        // Fall back to CSV time method
        attemptCSVTimeFallback();
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
            csvFile.println("Timestamp,DateTime,BME680_Port0,BME680_Port1,STS35_Port4,SHT45_Port5,STS35_Port6,SHT45_Port7,Combined_Fused_Temp");
            csvFile.close();
            Serial.println("CSV file created with header (Combined Fusion)");
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
    Serial.printf("Data logged to CSV: Combined Fused=%.2f°C\n", fusedTemp);
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
    /*if (!sht4.begin()) {
        sht4FailCount++;
        if (sht4FailCount > 5) {
            Serial.println("SHT45 multiple failures - attempting I2C recovery");
            recoverI2CBus();
            sht4FailCount = 0;
        }
        return NAN;
    }
    
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);*/
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
    /*int i = 0;
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
    sts.setFreq(sts.e10Hz);*/
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
        snprintf(buffer, sizeof(buffer), "CombF:%.2f/%.2f", fusedMaxTemp, fusedMinTemp);
        display.print(buffer);
    } else {
        display.print("CombF: --/--");
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
    combinedFilter.reset();
    
    // Clear averaging buffers
    readingIndex = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        for (int j = 0; j < READINGS_PER_INTERVAL; j++) {
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
        Serial.printf("New MAX extreme for Combined Fused: %.2f°C\n", temp);
    }
    
    if (temp < fusedMinTemp) {
        fusedMinTemp = temp;
        EEPROM.put(52, temp);
        updated = true;
        Serial.printf("New MIN extreme for Combined Fused: %.2f°C\n", temp);
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
    Serial.printf("[Combined Fused] Max: %.2f°C Min: %.2f°C\n", fusedMaxTemp, fusedMinTemp);
    Serial.println();
}

void initSensors()
{
    for(int i=2;i<NUM_SENSORS;i++)
    {
        selectPort(sensors[i].port);
        if(i<2)
        {
            while(!bme.begin())
            {
                delay(100);
                Serial.println("BME not initialised, retrying!");
            }
            Serial.println("BME initialised!");
        }
        else if(i==2||i==4)
        {
            while(sts.begin() != true)
            {
            Serial.println("STS35 init failed, retrying");
            delay(100);
            }
            sts.setFreq(sts.e10Hz);
            Serial.println("STS35 initialised!");
        }
        else
        {
            while(sht4.begin()!=true)
            {
                Serial.println("SHT45 init failed");
                delay(100);
            }
            sht4.setPrecision(SHT4X_HIGH_PRECISION);
            sht4.setHeater(SHT4X_NO_HEATER);
            Serial.println("SHT45 initialised!");
        }
    }
}

// --- Setup Function ---
void setup() {
    Serial.begin(115200);
    ESP.wdtEnable(8000);
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100);
    resetMultiplexer();
    initSensors();
    sdCardAvailable = initSDCard();
    if (sdCardAvailable) {
        Serial.println("SD card ready for data logging");
    } else {
        Serial.println("Continuing without SD card logging");
    }
    
    initWiFi();
        
    display.begin(0x3C);
    display.clearDisplay();
    display.display();
    
    EEPROM.begin(EEPROM_SIZE);
    pinMode(CLEAR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CLEAR_PIN), clearEEPROMISR, FALLING);
    
    // Initialize averaging buffer
    for (int i = 0; i < NUM_SENSORS; i++) {
        for (int j = 0; j < READINGS_PER_INTERVAL; j++) {
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
    Serial.println("*** COMBINED KALMAN-FUZZY FILTER ACTIVE ***");
}

// --- Main Loop with 30 Second Averaging ---
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
        Serial.printf("Collecting readings %d/%d for interval average...\n", readingIndex + 1, READINGS_PER_INTERVAL);
        
        if(readingIndex==0) {
            display.setContrast(127);
        } else {
            display.setContrast(31);
        }
        
        int currentCycleFailures = 0;
        for (int i = 0; i < NUM_SENSORS; i++) {
            selectPort(sensors[i].port);
            float rawTemp = sensors[i].readFunc();
            
            if (!isnan(rawTemp)) {
                float correctedTemp = rawTemp + sensors[i].correction;
                sensorReadings[i][readingIndex] = correctedTemp; // Store reading in buffer
                Serial.printf(" [Port %d] %s: %.2f°C\n", sensors[i].port, sensors[i].name, correctedTemp);
            } else {
                sensorReadings[i][readingIndex] = NAN; // Store NAN if read fails
                Serial.printf(" [Port %d] %s: Error reading\n", sensors[i].port, sensors[i].name);
                currentCycleFailures++;
            }
        }
        
        if (currentCycleFailures == NUM_SENSORS) {
            consecutiveFailures++;
        } else {
            consecutiveFailures = 0;
        }
        
        readingIndex++; // Increment the index for the next reading cycle
        
        // --- 30 Second-End Processing: Triggered when buffer is full ---
        if (readingIndex >= READINGS_PER_INTERVAL) {
            Serial.println("\n*** One interval elapsed. Calculating averages and fusing data with COMBINED KALMAN-FUZZY... ***");
            
            // Step 1: Calculate the average for each sensor over the last interval
            float intervalAverages[NUM_SENSORS];
            for (int i = 0; i < NUM_SENSORS; i++) {
                float sum = 0;
                int validReadingCount = 0;
                for (int j = 0; j < READINGS_PER_INTERVAL; j++) {
                    if (!isnan(sensorReadings[i][j])) {
                        sum += sensorReadings[i][j];
                        validReadingCount++;
                    }
                }
                
                if (validReadingCount > 0) {
                    intervalAverages[i] = sum / validReadingCount;
                    Serial.printf(" - Avg for %s: %.2f°C (%d readings)\n", sensors[i].name, intervalAverages[i], validReadingCount);
                } else {
                    intervalAverages[i] = NAN; // No valid readings this cycle
                    Serial.printf(" - Avg for %s: FAILED (0 readings)\n", sensors[i].name);
                }
            }
            
            // Step 2: Fuse the averaged readings using the COMBINED Kalman-Fuzzy filter
            float fusedTemp = combinedFilter.step(intervalAverages);
            currentFusedTemp = fusedTemp;
            Serial.printf("\n*** MINUTE COMBINED FUSED TEMPERATURE: %.2f°C ***\n", fusedTemp);
            
            // Step 3: Update EEPROM extremes and log to SD card
            updateFusedEEPROM(fusedTemp);
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (!isnan(intervalAverages[i])) {
                    updateEEPROM(sensors[i], intervalAverages[i]);
                }
            }
            
            writeToCSV(intervalAverages, fusedTemp); // Log the AVERAGES, not the raw readings
            
            // Step 4: Reset the index to start collecting for the next 30 second
            readingIndex = 0;
            Serial.println("\nBuffer reset. Starting new 30 second cycle.");
        }
        
        if (consecutiveFailures >= 3) {
            if (recoverI2CBus()) consecutiveFailures = 0;
        }
        
        if (displayNeedsUpdate) {
            updateDisplay();
        }
    }
}
