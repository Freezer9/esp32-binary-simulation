#include <Arduino.h>
#include <CRC8.h>
#include <math.h>
#include <Preferences.h>    // For non-volatile storage on ESP32
#include <ESP32Time.h>      // For RTC functions on ESP32

#define HEADER 0xABCD
#define TEAM_NUMBER 632419

CRC8 crc8;
Preferences preferences;  // Global Preferences object
ESP32Time rtc;            // ESP32Time instance

// Default time: 6 March 2025, 08:00:00 UTC
const uint32_t DEFAULT_EPOCH = 1757241600;  

unsigned long lastTelemetryMillis = 0;
unsigned long missionStartTime = 0;
unsigned long lastUpdateMillis = 0;
const unsigned long telemetryInterval = 1000; // 1-second telemetry interval

// Global state variables
bool telemetryEnabled = false, missionActive = false;
int satelliteStatus = 0, packetNumber = 1;
float velocity = 0.0, descentRate = 0.0, acceleration = 0.0;
float altitude1 = 0.0, altitude2 = 0.0;
uint32_t pressure1 = 101325, pressure2 = 101325; // Default sea-level pressure (Pa)

float max(float a, float b) {
    return (a > b) ? a : b;
}

float randomFloat(float minVal, float maxVal) {
    return minVal + (random(1000) / 1000.0) * (maxVal - minVal);
}

float calculatePressure(float altitude) {
    return 101325.0 * exp(-altitude / 8435.0);
}

float calculateAltitude(uint32_t pressure) {
    return 8435.0 * log(101325.0 / pressure);
}

uint8_t calculateCRC8(uint8_t *data, size_t length) {
    crc8.restart();
    for (size_t i = 0; i < length; i++) {
        crc8.add(data[i]);
    }
    return crc8.calc();
}

struct TelemetryData {
    uint32_t packetNumber;
    uint8_t satelliteStatus;
    uint8_t errorCode;
    uint32_t missionTime;
    uint32_t pressure1;
    uint32_t pressure2;
    int16_t descentRate; // m/s * 100
    int16_t temp;        // °C * 100
    uint8_t batteryVoltage; // V * 10
    float gps1Latitude;
    float gps1Longitude;
    int16_t gps1Altitude; // m * 10
    int16_t pitch; // degrees * 100
    int16_t roll;  // degrees * 100
    int16_t yaw;   // degrees * 100
    uint8_t lnln[4];
    int16_t iotS1Data; // * 100
    int16_t iotS2Data; // * 100
    uint32_t teamNumber;
} __attribute__((packed));

// Load RTC time from preferences; if not saved, use default time.
void loadTimeFromPreferences() {
    if (preferences.isKey("epoch")) {
        long savedEpoch = preferences.getLong("epoch", DEFAULT_EPOCH);
        rtc.setTime(savedEpoch);
    } else {
        rtc.setTime(DEFAULT_EPOCH);
        preferences.putLong("epoch", DEFAULT_EPOCH);
    }
}

// Save the current RTC epoch to preferences.
void saveTimeToPreferences() {
    preferences.putLong("epoch", rtc.getEpoch());
}

// Process serial commands. This function now handles two types:
// 1. SDT,HH,MM,SS,DD,MM,YYYY  -> Set DateTime command using ESP32Time
// 2. CMD,... (existing commands)
void processSerialCommand(String msg) {
    msg.trim();
    // Process SDT command to set datetime
    if (msg.startsWith("SDT,")) {
        int firstComma = msg.indexOf(',');
        if (firstComma < 0) {
            Serial.println("Invalid SDT command format. Expected: SDT,HH,MM,SS,DD,MM,YYYY");
            return;
        }
        String params = msg.substring(firstComma + 1);
        int tokenCount = 0;
        int tokens[6]; // HH, MM, SS, DD, MM, YYYY
        while (params.length() > 0 && tokenCount < 6) {
            int commaIndex = params.indexOf(',');
            String token;
            if (commaIndex == -1) {
                token = params;
                params = "";
            } else {
                token = params.substring(0, commaIndex);
                params = params.substring(commaIndex + 1);
            }
            token.trim();
            tokens[tokenCount++] = token.toInt();
        }
        if (tokenCount < 6) {
            Serial.println("Invalid SDT command. Not enough parameters.");
            return;
        }
        // tokens: [0]=HH, [1]=MM, [2]=SS, [3]=DD, [4]=MM, [5]=YYYY
        rtc.setTime(tokens[0], tokens[1], tokens[2], tokens[3], tokens[4], tokens[5]);
        saveTimeToPreferences();
        Serial.print("RTC time set to: ");
        Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));
    }
    // Process CMD commands as before.
    else if (msg.startsWith("CMD,")) {
        String parts[4];
        int partIndex = 0;
        int lastIndex = 0;
        int commaIndex = msg.indexOf(',');
        while (commaIndex >= 0 && partIndex < 4) {
            parts[partIndex++] = msg.substring(lastIndex, commaIndex);
            lastIndex = commaIndex + 1;
            commaIndex = msg.indexOf(',', lastIndex);
        }
        parts[partIndex] = msg.substring(lastIndex);
        if (parts[0] == "CMD" && parts[1] == String(TEAM_NUMBER)) {
            String commandType = parts[2];
            if (commandType == "TELEM") {
                if (parts[3] == "ON") {
                    telemetryEnabled = true;
                    preferences.putBool("telemetry", telemetryEnabled); // Save state
                    Serial.println("Telemetry enabled.");
                } else if (parts[3] == "OFF") {
                    telemetryEnabled = false;
                    preferences.putBool("telemetry", telemetryEnabled); // Save state
                    Serial.println("Telemetry disabled.");
                }
            } else if (commandType == "FLY") {
                missionActive = true;
                missionStartTime = millis();
                lastUpdateMillis = missionStartTime;
                satelliteStatus = 1; // Ascent
                velocity = 0.0;
                acceleration = 400.0; // Initial acceleration (m/s^2)
                Serial.println("Mission started!");
            } else if (commandType == "CAL") {
                packetNumber = 1;
                missionActive = false;
                satelliteStatus = 0; // Ready
                velocity = 0.0;
                acceleration = 0.0;
                altitude1 = 0.0;
                altitude2 = 0.0;
                pressure1 = 101325;
                pressure2 = 101325;
                descentRate = 0.0;
                Serial.println("Calibration complete.");
            } else {
                Serial.println("Error: Unknown command.");
            }
        } else {
            Serial.println("Error: Invalid team number!");
        }
    } else {
        Serial.println("Error: Unknown command format.");
    }
}

void simulateFlight() {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateMillis) / 1000.0; // Time step in seconds
    lastUpdateMillis = currentTime;

    switch (satelliteStatus) {
        case 1: // Ascent Phase
            velocity += acceleration * dt;
            altitude1 += velocity * dt;
            altitude2 += velocity * dt;
            pressure1 = calculatePressure(altitude1);
            pressure2 = calculatePressure(altitude2);
            if (altitude1 > 500) {
                acceleration -= 100 * dt; // Reduce acceleration after 500 m
                acceleration = max(acceleration, 0);
            }
            if (altitude1 >= 1000) {
                satelliteStatus = 2; // Transition to Descent Phase 1
                velocity = 0.0;
                acceleration = 0.0;
                descentRate = 15.0; // Model Satellite Descent
                missionStartTime = millis(); // Reset phase timer
            }
            break;
        case 2: // Descent Phase 1 (15 m/s for 30s)
            altitude1 -= descentRate * dt;
            altitude2 -= descentRate * dt;
            altitude1 = max(altitude1, 0);
            altitude2 = max(altitude2, 0);
            pressure1 = calculatePressure(altitude1);
            pressure2 = calculatePressure(altitude2);
            if ((millis() - missionStartTime) / 1000.0 >= 30) {
                satelliteStatus = 3; // Release Phase
                descentRate = 15.0;  // Continue at 15 m/s
                missionStartTime = millis();
            }
            break;
        case 3: // Release Phase (15 m/s for 10s)
            altitude1 -= descentRate * dt;
            altitude2 -= descentRate * dt;
            altitude1 = max(altitude1, 0);
            altitude2 = max(altitude2, 0);
            pressure1 = calculatePressure(altitude1);
            pressure2 = calculatePressure(altitude2);
            if ((millis() - missionStartTime) / 1000.0 >= 10) {
                satelliteStatus = 4; // Science Payload & Container Descent
                descentRate = 6.0;   // Science Payload at 6 m/s
                missionStartTime = millis();
            }
            break;
        case 4: // Science Payload (6 m/s) and Container Descent (10 m/s)
            altitude1 -= 6.0 * dt;  // Science Payload
            altitude2 -= 10.0 * dt; // Container
            altitude1 = max(altitude1, 0);
            altitude2 = max(altitude2, 0);
            pressure1 = calculatePressure(altitude1);
            pressure2 = calculatePressure(altitude2);
            if (altitude1 <= 0 && altitude2 <= 0) {
                satelliteStatus = 5; // Recovery
                descentRate = 0.0;
                missionActive = false;
            }
            break;
        case 5: // Recovery
            altitude1 = 0;
            altitude2 = 0;
            pressure1 = 101325;
            pressure2 = 101325;
            descentRate = 0.0;
            break;
    }
}

void sendTelemetry() {
    TelemetryData data = {
        .packetNumber = (uint32_t)(packetNumber++),
        .satelliteStatus = (uint8_t)(satelliteStatus),
        .errorCode = 0b000000,
        .missionTime = (uint32_t)((millis() - missionStartTime) / 1000),
        .pressure1 = pressure1,
        .pressure2 = pressure2,
        .descentRate = (int16_t)(descentRate / 100),
        .temp = (int16_t)(randomFloat(20, 30) * 100), // 20-30°C
        .batteryVoltage = (uint8_t)(randomFloat(11.8, 12.2) * 10), // 11.8-12.2V
        .gps1Latitude = randomFloat(-90, 90),
        .gps1Longitude = randomFloat(-180, 180),
        .gps1Altitude = (int16_t)(altitude1 * 10),
        .pitch = (int16_t)(randomFloat(-90, 90) * 100),
        .roll = (int16_t)(randomFloat(-180, 180) * 100),
        .yaw = (int16_t)(randomFloat(-180, 180) * 100),
        .lnln = {'0', 'N', '0', 'N'},
        .iotS1Data = (int16_t)(randomFloat(15, 25) * 100),
        .iotS2Data = (int16_t)(randomFloat(15, 25) * 100),
        .teamNumber = TEAM_NUMBER
    };

    uint8_t buffer[sizeof(data) + 4];
    uint16_t header = HEADER;
    uint8_t packetLength = sizeof(data);
    uint8_t *ptr = buffer;

    memcpy(ptr, &header, 2);
    ptr += 2;
    memcpy(ptr, &packetLength, 1);
    ptr += 1;
    memcpy(ptr, &data, sizeof(data));
    ptr += sizeof(data);

    uint8_t checksum = calculateCRC8((uint8_t *)&data, sizeof(data));
    memcpy(ptr, &checksum, 1);

    Serial.write(buffer, sizeof(buffer));
}

void setup() {
    Serial.begin(115200);
    // Initialize Preferences for non-volatile storage
    preferences.begin("storage", false);
    // Load saved telemetry state and packet number
    telemetryEnabled = preferences.getBool("telemetry", false);
    packetNumber = preferences.getUInt("packetNumber", 1);
    Serial.print("Telemetry state loaded: ");
    Serial.println(telemetryEnabled ? "ON" : "OFF");

    // Remove NTP configuration; use saved RTC time instead.
    loadTimeFromPreferences();

    randomSeed(analogRead(0)); // Seed random number generator
    Serial.println("Serial Telemetry System Ready");
    Serial.print("RTC Time: ");
    Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));
}

void loop() {
    if (Serial.available()) {
        String received = Serial.readStringUntil('\n');
        received.trim();
        processSerialCommand(received);
    }

    if (missionActive) {
        simulateFlight();
    }

    if (telemetryEnabled && millis() - lastTelemetryMillis >= telemetryInterval) {
        sendTelemetry();
        lastTelemetryMillis = millis();
    }
}
