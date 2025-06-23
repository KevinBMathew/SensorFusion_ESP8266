#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_BME680.h>
#include <Adafruit_SHT4x.h>
#include <EEPROM.h>
#include <ESP.h>
#include "DFRobot_STS3X.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define TCA_ADDR 0x70
#define EEPROM_SIZE 48 // 6 sensors * 8 bytes (4 for max, 4 for min)
#define CLEAR_PIN 0 // GPIO0 (Flash button) - use internal pull-up
#define NUM_SENSORS 6
#define READINGS_PER_MINUTE 12
#define SDA_PIN 4
#define SCL_PIN 5

// Create SH1106 display object
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_BME680 bme;
Adafruit_SHT4x sht4;
DFRobot_STS3X sts(&Wire, STS3X_I2C_ADDRESS_B);

// Interrupt variables
volatile bool clearEEPROM = false;

// Timing variables
unsigned long lastReadTime = 0;
const unsigned long readInterval = 5000; // 5 seconds in milliseconds
unsigned long lastMemCheck = 0;

// Buffers for averaging
float sensorReadings[NUM_SENSORS][READINGS_PER_MINUTE] = {0};
uint8_t readingIndex[NUM_SENSORS] = {0};

// Error tracking
int consecutiveFailures = 0;
int bmeFailCount = 0;
int sht4FailCount = 0;
int sts35FailCount = 0;

struct SensorPort {
    uint8_t port;
    float (*readFunc)();
    const char* name;
    float maxTemp;
    float minTemp;
    int eepromAddr;
    float correction;
};

bool displayNeedsUpdate = true; // Flag to track if display needs updating

// I2C Multiplexer Reset Function
void resetMultiplexer() {
    Serial.println("Resetting I2C multiplexer...");
    
    // Disable all channels
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(0x00);  // Disable all channels
    Wire.endTransmission();
    delay(100);
    
    // Test multiplexer communication
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
    
    // Check if SCL is held low - cannot recover from this
    pinMode(SCL_PIN, INPUT_PULLUP);
    if (digitalRead(SCL_PIN) == LOW) {
        Serial.println("SCL held low - cannot recover");
        return false;
    }
    
    // Recovery procedure for SDA held low
    pinMode(SDA_PIN, INPUT_PULLUP);
    if (digitalRead(SDA_PIN) == LOW) {
        Serial.println("SDA held low - attempting recovery");
        
        // Clock out any remaining bits
        pinMode(SCL_PIN, OUTPUT_OPEN_DRAIN);
        for (int i = 0; i < 9; i++) {
            digitalWrite(SCL_PIN, LOW);
            delayMicroseconds(5);
            digitalWrite(SCL_PIN, HIGH);
            delayMicroseconds(5);
            
            // Check if SDA is released
            if (digitalRead(SDA_PIN) == HIGH) {
                break;
            }
        }
        
        // Generate STOP condition
        pinMode(SDA_PIN, OUTPUT_OPEN_DRAIN);
        digitalWrite(SDA_PIN, LOW);
        delayMicroseconds(5);
        digitalWrite(SCL_PIN, HIGH);
        delayMicroseconds(2);
        digitalWrite(SDA_PIN, HIGH);
        delayMicroseconds(2);
        
        // Return pins to input mode
        pinMode(SDA_PIN, INPUT_PULLUP);
        pinMode(SCL_PIN, INPUT_PULLUP);
    }
    
    // Reinitialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100);
    
    // Reset multiplexer after I2C recovery
    resetMultiplexer();
    
    Serial.println("I2C bus recovery completed");
    return true;
}

// Test multiplexer channel functionality
bool testMultiplexerChannel(uint8_t channel) {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);
    return (Wire.endTransmission() == 0);
}

// Memory monitoring function
void checkMemory() {
    if (millis() - lastMemCheck > 60000) { // Check every minute
        uint32_t free = ESP.getFreeHeap();
        Serial.printf("Free heap: %u bytes\n", free);
        
        // Warning if memory is getting low
        if (free < 10000) {
            Serial.println("WARNING: Low memory detected!");
        }
        
        lastMemCheck = millis();
    }
}

// Fixed interrupt service routine for ESP8266
void IRAM_ATTR clearEEPROMISR() {
    clearEEPROM = true;
}

void selectPort(uint8_t port) {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << port);
    Wire.endTransmission();
    delay(50); // Increased delay for better stability
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
    bmeFailCount = 0; // Reset on success
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
    
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
    sensors_event_t humidity, temp;
    
    // Add retry logic with improved error checking
    for (int retry = 0; retry < 3; retry++) {
        sht4.getEvent(&humidity, &temp);
        float temperature = temp.temperature;
        
        // Check for valid reading (not 0.0 and within reasonable range)
        if (temperature != 0.0 && temperature > -40.0 && temperature < 85.0) {
            sht4FailCount = 0; // Reset on success
            return temperature;
        }
        delay(50); // Short delay before retry
    }
    
    sht4FailCount++;
    return NAN; // Return NAN if all retries fail
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
            return NAN; // Return NAN instead of arbitrary value
        }
    }
    sts35FailCount = 0; // Reset on success
    sts.setFreq(sts.e10Hz);
    return sts.getTemperaturePeriodC();
}

SensorPort sensors[] = {
    {0, &readBME680, "0.BME680", -INFINITY, INFINITY, 0, 0},
    {1, &readBME680, "1.BME680", -INFINITY, INFINITY, 8, 0},
    {4, &readSTS35, "4.STS35", -INFINITY, INFINITY, 16, 0},
    {5, &readSHT45, "5.SHT45", -INFINITY, INFINITY, 24, 0},
    {6, &readSTS35, "6.STS35", -INFINITY, INFINITY, 32, 0},
    // Removed problematic SHT45 from port 3 - will be handled separately if needed
};

void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    
    // Display sensor extremes in rows with 2 decimal places
    for(int i = 0; i < NUM_SENSORS - 1; i++) { // -1 because we removed one sensor
        int x = 0;
        int y = 2 + (i * 10);
        display.setCursor(x, y);
        
        // Display sensor name and max/min values with 2 decimal places
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
    
    display.display();
    displayNeedsUpdate = false; // Reset the flag
}

void clearAllEEPROM() {
    Serial.println("Clearing EEPROM...");
    // Clear all EEPROM data
    for (int i = 0; i < EEPROM_SIZE; i++) {
        EEPROM.write(i, 0);
    }
    EEPROM.commit();
    
    // Reset all sensor extremes to default values
    for (auto& sensor : sensors) {
        sensor.maxTemp = -INFINITY;
        sensor.minTemp = INFINITY;
    }
    
    // Clear reading buffers
    for (int i = 0; i < NUM_SENSORS; i++) {
        readingIndex[i] = 0;
        for (int j = 0; j < READINGS_PER_MINUTE; j++) {
            sensorReadings[i][j] = 0;
        }
    }
    
    // Reset error counters
    consecutiveFailures = 0;
    bmeFailCount = 0;
    sht4FailCount = 0;
    sts35FailCount = 0;
    
    // Update display to show cleared values
    displayNeedsUpdate = true;
    updateDisplay();
    Serial.println("EEPROM cleared successfully!");
}

void setup() {
    Serial.begin(115200);
    
    // Initialize watchdog timer
    ESP.wdtEnable(8000); // 8 second watchdog
    
    Wire.begin(SDA_PIN, SCL_PIN); // SDA=GPIO4, SCL=GPIO5
    delay(100);
    
    // Reset multiplexer on startup
    resetMultiplexer();
    
    // Initialize display
    display.begin(0x3C);
    display.clearDisplay();
    display.display();
    
    // Initialize EEPROM
    EEPROM.begin(EEPROM_SIZE);
    
    // Configure interrupt pin for EEPROM clear (using GPIO0 - Flash button)
    pinMode(CLEAR_PIN, INPUT_PULLUP); // Use internal pull-up resistor
    attachInterrupt(digitalPinToInterrupt(CLEAR_PIN), clearEEPROMISR, FALLING);
    
    // Load stored values from EEPROM
    for (auto& sensor : sensors) {
        EEPROM.get(sensor.eepromAddr, sensor.maxTemp);
        EEPROM.get(sensor.eepromAddr + 4, sensor.minTemp);
        
        // Initialize with default values if EEPROM is empty
        if (isnan(sensor.maxTemp)) sensor.maxTemp = -INFINITY;
        if (isnan(sensor.minTemp)) sensor.minTemp = INFINITY;
    }
    
    // Initialize reading buffers
    for (int i = 0; i < NUM_SENSORS; i++) {
        readingIndex[i] = 0;
        for (int j = 0; j < READINGS_PER_MINUTE; j++) {
            sensorReadings[i][j] = 0;
        }
    }
    
    // Initial display update
    updateDisplay();
    Serial.println("System initialized with stability improvements");
    Serial.println("- Watchdog timer enabled (8 seconds)");
    Serial.println("- I2C bus recovery implemented");
    Serial.println("- Multiplexer reset on startup");
    Serial.println("- Memory monitoring active");
    Serial.println("- Enhanced error handling active");
    Serial.println("Press and release the FLASH button to clear EEPROM");
    Serial.println("Sensor readings will be averaged over 1 minute (12 readings at 5-second intervals)");
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
        displayNeedsUpdate = true; // Set flag to update display
    }
    
    return updated;
}

void printExtremes() {
    Serial.println("\nStored Extremes:");
    for (auto& sensor : sensors) {
        Serial.printf("[%s] Max: %.2f°C Min: %.2f°C\n",
                     sensor.name, sensor.maxTemp, sensor.minTemp);
    }
    Serial.println();
}

void loop() {
    // Feed watchdog timer at start of each loop
    ESP.wdtFeed();
    
    unsigned long currentTime = millis();
    
    // Check memory status
    checkMemory();
    
    // Check for EEPROM clear interrupt
    if (clearEEPROM) {
        clearEEPROM = false; // Clear the flag
        clearAllEEPROM();
        delay(500); // Debounce delay
    }
    
    // Check for serial input
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'r') {
            printExtremes();
        }
    }
    
    // Take readings every 5 seconds
    if (currentTime - lastReadTime >= readInterval) {
        lastReadTime = currentTime;
        
        int currentFailures = 0;
        
        for (int i = 0; i < NUM_SENSORS - 1; i++) { // -1 because we removed problematic sensor
            // Test multiplexer channel before attempting to read
            if (!testMultiplexerChannel(sensors[i].port)) {
                Serial.printf("Multiplexer channel %d failed - attempting reset\n", sensors[i].port);
                resetMultiplexer();
                delay(100);
            }
            
            selectPort(sensors[i].port);
            delay(10);
            float rawTemp = sensors[i].readFunc();
            
            if (!isnan(rawTemp)) {
                // Apply correction factor
                float correctedTemp = rawTemp + sensors[i].correction;
                
                // Store reading in buffer
                sensorReadings[i][readingIndex[i]] = correctedTemp;
                readingIndex[i]++;
                
                Serial.printf("[Port %d] %s: %.2f°C (raw: %.2f°C, correction: %.2f°C) - Reading %d/12\n",
                             sensors[i].port, sensors[i].name, correctedTemp, rawTemp,
                             sensors[i].correction, readingIndex[i]);
                
                // If buffer is full (12 readings), calculate average
                if (readingIndex[i] == READINGS_PER_MINUTE) {
                    float sum = 0;
                    for (int j = 0; j < READINGS_PER_MINUTE; j++) {
                        sum += sensorReadings[i][j];
                    }
                    float avg = sum / READINGS_PER_MINUTE;
                    
                    Serial.printf("*** 1-MINUTE AVERAGE for %s: %.2f°C ***\n", sensors[i].name, avg);
                    
                    // Update extremes with the average
                    updateEEPROM(sensors[i], avg);
                    
                    // Reset buffer index for next minute
                    readingIndex[i] = 0;
                }
                
                // Reset consecutive failures on successful read
                consecutiveFailures = 0;
                
            } else {
                Serial.printf("[Port %d] %s: Error reading\n", sensors[i].port, sensors[i].name);
                currentFailures++;
            }
        }
        
        // Check if we need I2C bus recovery
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
        
        // Only update display if new extremes were found
        if (displayNeedsUpdate) {
            updateDisplay();
            Serial.println("Display updated with new extremes");
        }
    }
}
