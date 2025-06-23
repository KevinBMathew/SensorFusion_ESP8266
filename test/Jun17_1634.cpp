#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_BME680.h>
#include <Adafruit_SHT4x.h>
#include <EEPROM.h>
#include "DFRobot_STS3X.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Address line pins (replacing TCA multiplexer)
#define A0_PIN 13  // D7 (GPIO13)
#define A1_PIN 12  // D6 (GPIO12) 
#define A2_PIN 14  // D5 (GPIO14)

#define EEPROM_SIZE 48 // 6 sensors * 8 bytes (4 for max, 4 for min)
#define CLEAR_PIN 0 // GPIO0 (Flash button) - use internal pull-up
#define NUM_SENSORS 6
#define READINGS_PER_MINUTE 12

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

// Buffers for averaging
float sensorReadings[NUM_SENSORS][READINGS_PER_MINUTE] = {0};
uint8_t readingIndex[NUM_SENSORS] = {0};

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

// Interrupt Service Routine - must have ICACHE_RAM_ATTR for ESP8266
void ICACHE_RAM_ATTR clearEEPROMISR() {
  clearEEPROM = true;
}

void selectPort(uint8_t port) {
  // Set address lines based on port number (binary)
  digitalWrite(A0_PIN, (port & 0x01) ? HIGH : LOW);  // LSB
  digitalWrite(A1_PIN, (port & 0x02) ? HIGH : LOW);  // Bit 1
  digitalWrite(A2_PIN, (port & 0x04) ? HIGH : LOW);  // MSB
  delay(10); // Small delay for address line settling
}

float readBME680() {
  if (!bme.begin(0x76)) return NAN;
  return bme.readTemperature();
}

float readSHT45() {
  if (!sht4.begin()) return NAN;
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);
  sensors_event_t humidity, temp;
  
  // Add retry logic
  for (int retry = 0; retry < 3; retry++) {
    sht4.getEvent(&humidity, &temp);
    float temperature = temp.temperature;
    
    // Check for valid reading (not 0.0 and within reasonable range)
    if (temperature != 0.0 && temperature > -40.0 && temperature < 85.0) {
      return temperature;
    }
    delay(50); // Short delay before retry
  }
  return NAN; // Return NAN if all retries fail
}

float readSTS35() {
  int i = 0;
  while(sts.begin() != true) {
    Serial.println("STS35 init failed");
    delay(100);
    i++;
    if(i == 2)
      return NAN; // Return NAN instead of arbitrary value
  }
  sts.setFreq(sts.e10Hz);
  return sts.getTemperaturePeriodC();
}

SensorPort sensors[] = {
  {0, &readBME680, "0.BME680", -INFINITY, INFINITY, 0, 0},//-2.15
  {1, &readBME680, "1.BME680", -INFINITY, INFINITY, 8, 0},//-3.08
  {4, &readSTS35, "4.STS35", -INFINITY, INFINITY, 16, 0},//-0.4
  {5, &readSHT45, "5.SHT45", -INFINITY, INFINITY, 24, 0},//-0.25
  {6, &readSTS35, "6.STS35", -INFINITY, INFINITY, 32, 0},//-0.4
  {3, &readSHT45, "3.SHT45", -INFINITY, INFINITY, 40, 0} //-0.05
};

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  
  // Display sensor extremes in 6 rows with 2 decimal places
  for(int i = 0; i < 6; i++) {
    int x = 0;
    int y = 2 + (i * 10);
    display.setCursor(x, y);
    
    // Display sensor name and max/min values with 2 decimal places
    if (sensors[i].maxTemp != -INFINITY && sensors[i].minTemp != INFINITY) {
      // Use printf-style formatting for precise decimal control
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
  
  // Update display to show cleared values
  displayNeedsUpdate = true;
  updateDisplay();
  Serial.println("EEPROM cleared successfully!");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(4, 5); // SDA=GPIO4, SCL=GPIO5
  
  // Configure address line pins as outputs
  pinMode(A0_PIN, OUTPUT);
  pinMode(A1_PIN, OUTPUT);
  pinMode(A2_PIN, OUTPUT);
  
  // Initialize all address lines to LOW
  digitalWrite(A0_PIN, LOW);
  digitalWrite(A1_PIN, LOW);
  digitalWrite(A2_PIN, LOW);
  
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
  Serial.println("Display initialized with stored extremes");
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
  unsigned long currentTime = millis();
  
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
    
    for (int i = 0; i < NUM_SENSORS; i++) {
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
      } else {
        Serial.printf("[Port %d] %s: Error reading\n", sensors[i].port, sensors[i].name);
      }
    }
    
    // Only update display if new extremes were found
    if (displayNeedsUpdate) {
      updateDisplay();
      Serial.println("Display updated with new extremes");
    }
  }
}
