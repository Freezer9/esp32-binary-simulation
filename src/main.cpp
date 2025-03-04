#include <Arduino.h>
#include <CRC8.h>

#define HEADER 0xABCD
#define TEAM_NUMBER 632419
CRC8 crc8;

unsigned long lastTelemetryMillis = 0;
unsigned long missionStartTime = 0;
unsigned long lastUpdateMillis = 0;
const unsigned long telemetryInterval = 1000;

bool telemetryEnabled = false, missionActive = false;
int satelliteStatus = 0, packetNumber = 1;
float velocity = 0.0, descentRate = 0.0, acceleration = 0.0;

uint32_t pressure1 = 101325, pressure2 = 101320; // Default sea-level pressure
float altitude1 = 0.0, altitude2 = 0.0; // Altitude in meters

float randomFloat(float min, float max) {
    return min + (random(1000) / 1000.0) * (max - min);
}

float calculateAltitude(uint32_t pressure) {
    return 44330.0 * (1.0 - pow((float)pressure / 101325.0, 0.190263));
}

uint8_t calculateCRC8(uint8_t *data, size_t length) {
    crc8.restart();  // Reset CRC sebelum digunakan
    for (size_t i = 0; i < length; i++) {
        crc8.add(data[i]);  // Tambahkan byte ke CRC
    }
    return crc8.calc();  // Dapatkan hasil CRC8
}


struct TelemetryData {
    uint32_t packetNumber;
    uint8_t satelliteStatus;
    uint8_t errorCode;
    uint32_t missionTime;
    uint32_t pressure1;
    uint32_t pressure2;
    int16_t descentRate;
    int16_t temp;  // Dikalikan 100
    uint8_t batteryVoltage; // Dikalikan 10
    float gps1Latitude;
    float gps1Longitude;
    int16_t gps1Altitude; // Dikalikan 10
    int16_t pitch; // Dikalikan 100
    int16_t roll; // Dikalikan 100
    int16_t yaw; // Dikalikan 100
    uint8_t lnln[4];
    int16_t iotS1Data; // Dikalikan 100
    int16_t iotS2Data; // Dikalikan 100
    uint32_t teamNumber;
} __attribute__((packed));

void processSerialCommand(String msg) {
    if (msg.startsWith("CMD,")) {
        String parts[4];
        int partIndex = 0;
        int lastIndex = 0;
        int commaIndex = msg.indexOf(',');

        while (commaIndex >= 0 && partIndex < 3) {
            parts[partIndex++] = msg.substring(lastIndex, commaIndex);
            lastIndex = commaIndex + 1;
            commaIndex = msg.indexOf(',', lastIndex);
        }
        parts[partIndex++] = msg.substring(lastIndex);

        if (parts[0] == "CMD" && partIndex >= 3) {
            String commandType = parts[2];

            if (parts[1] != String(TEAM_NUMBER) ) {
                Serial.println("Error: Invalid team number!");
                return;
            }

            if (commandType == "TELEM" && (parts[3] == "ON" || parts[3] == "OFF")) {

                if (parts[3] == "ON") {
                    telemetryEnabled = true;
                    Serial.println("Telemetry enabled.");
                } else {
                    telemetryEnabled = false;
                    Serial.println("Telemetry disabled.");
                }

            } else if (commandType == "FLY") {
                missionActive = true;
                missionStartTime = millis();
                lastUpdateMillis = missionStartTime;
                satelliteStatus = 1;
                velocity = 0.0;
                acceleration = 400.0;
                Serial.println("Mission started!");             
            } else if (commandType == "CAL") {
                packetNumber = packetNumber;
                missionActive = false;
                satelliteStatus = 0;
                velocity = 0.0;
                acceleration = 0.0;
                Serial.println("Calibration complete.");
            } else {
                Serial.println("Error: Unknown command.");
            }
        }
    }
}

void simulateFlight() {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateMillis) / 1000.0;
    lastUpdateMillis = currentTime;

    altitude1 = calculateAltitude(pressure1);
    altitude2 = calculateAltitude(pressure2);

    switch (satelliteStatus) {
        case 1: 
            velocity += acceleration * dt;
            pressure1 -= 50;
            pressure2 -= 50;
            if (altitude1 >= 1000) satelliteStatus = 2;
            break;
        case 2:
            pressure1 += 50;
            pressure2 += 50;
            descentRate = 15;
            if (altitude1 <= 800) satelliteStatus = 3;
            break;
        case 3:
            pressure1 += 70;
            pressure2 += 70;
            descentRate = 20;
            if (altitude1 <= 500) satelliteStatus = 4;
            break;
        case 4:
            pressure1 += 40;
            pressure2 += 40;
            descentRate = 7;
            if (altitude1 <= 0) {
                altitude1 = 0;
                satelliteStatus = 5;
            }
            break;
        case 5:
            pressure2 += 50;
            descentRate = 10;
            if (altitude2 <= 0) {
                altitude1 = 0;
                altitude2 = 0;
                descentRate = 0;
                missionActive = false;
            }
            break;
    }
}

void sendTelemetry() {
    time_t now;
    time(&now);

    TelemetryData data = {
        .packetNumber = (uint32_t)(packetNumber++),
        .satelliteStatus = (uint8_t)(satelliteStatus),
        .errorCode = 0b000000,
        .missionTime = (uint32_t)now,
        .pressure1 = pressure1,
        .pressure2 = pressure2,
        .descentRate = (int16_t) descentRate,
        .temp = (int16_t) (randomFloat(20, 30) * 100), // 25.00°C (x100)
        .batteryVoltage = (uint8_t)(randomFloat(11.8, 12.2) * 10), // 12.0V (x10)
        .gps1Latitude = -7.2575,
        .gps1Longitude = 112.7521,
        .gps1Altitude = (int16_t)(randomFloat(0, 100) * 10), // 10.0m (x10)
        .pitch = (int16_t)(randomFloat(-30, 30) * 100), // 15.00° (x100)
        .roll = (int16_t)(randomFloat(-30, 30) * 100), // -5.00° (x100)
        .yaw = (int16_t)(randomFloat(-30, 30) * 100), // 30.00° (x100)
        .lnln = {6, 1, 8, 1},
        .iotS1Data = (int16_t)(randomFloat(0, 20) * 100), // 12.34 (x100)
        .iotS2Data =  (int16_t)(randomFloat(-10, 10) * 100), // -5.67 (x100)
        .teamNumber = TEAM_NUMBER
    };

    uint8_t buffer[sizeof(data) + 4];
    uint16_t header = HEADER;
    uint8_t packetLength = sizeof(data);
    uint8_t *ptr = buffer;

    // Menyalin header
    memcpy(ptr, &header, 2);
    ptr += 2;

    // Menyalin panjang paket
    memcpy(ptr, &packetLength, 1);
    ptr += 1;

    // Menyalin data telemetri
    memcpy(ptr, &data, sizeof(data));
    ptr += sizeof(data);

    // Menghitung checksum
    uint8_t checksum = calculateCRC8((uint8_t *)&data, sizeof(data));
    memcpy(ptr, &checksum, 1);

    // Mengirim data melalui Serial
    Serial.write(buffer, sizeof(buffer));
}

void setup() {
    Serial.begin(115200);
    configTime(7 * 3600, 0, "pool.ntp.org");
    Serial.println("Serial Telemetry System Ready");
}

void loop() {
    if (Serial.available()) {
        String received = Serial.readStringUntil('\n');
        received.trim();
        processSerialCommand(received);
    }
    
    if (missionActive) simulateFlight();
    
    
    if (telemetryEnabled && millis() - lastTelemetryMillis >= telemetryInterval) {
        sendTelemetry();
        lastTelemetryMillis = millis();
    }
}
