#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <TinyGPSPlus.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

// =====================================================
// 1. PIN CONFIGURATION
// =====================================================

// DS18B20 temperature sensor
constexpr uint8_t TEMP_PIN = 4;

// ADXL345 accelerometer - I2C
constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t ADXL345_ADDRESS = 0x53;

// NEO-6M GPS - UART2
constexpr uint8_t GPS_RX_PIN = 16;  // ESP32 RX <- GPS TX
constexpr uint8_t GPS_TX_PIN = 17;  // ESP32 TX -> GPS RX
constexpr uint32_t GPS_BAUD = 9600;

// =====================================================
// 2. PRODUCT TEMPERATURE LIMITS
// =====================================================

constexpr char PRODUCT_ID[] = "other_medicine";
constexpr float TEMP_MIN_C = 28.0f;
constexpr float TEMP_MAX_C = 30.0f;

// =====================================================
// 3. TIMING AND THRESHOLD SETTINGS
// =====================================================

constexpr uint32_t ACCEL_INTERVAL_MS = 10;
constexpr uint32_t DASHBOARD_INTERVAL_MS = 1000;
constexpr uint32_t TEMP_INTERVAL_MS = 2000;
constexpr uint32_t TEMP_CONVERSION_MS = 750;
constexpr uint32_t GPS_MAX_AGE_MS = 5000;
constexpr uint32_t ALERT_COOLDOWN_MS = 500;

constexpr float GRAVITY_MPS2 = 9.80665f;
constexpr float GRAVITY_FILTER_ALPHA = 0.90f;
constexpr float IMPACT_THRESHOLD_G = 1.5f;

// =====================================================
// 4. SENSOR OBJECTS
// =====================================================

Adafruit_ADXL345_Unified accel =
    Adafruit_ADXL345_Unified(12345);

TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);

OneWire oneWire(TEMP_PIN);
DallasTemperature tempSensors(&oneWire);

// =====================================================
// 5. SENSOR STATUS VARIABLES
// =====================================================

bool accelReady = false;

int tempSensorCount = 0;
bool tempConversionPending = false;
bool tempReadingAvailable = false;
bool tempDisconnected = false;
bool tempWarning85 = false;

float temperatureC = NAN;

// Accelerometer gravity estimate
bool gravityInitialized = false;
float gravityX = 0.0f;
float gravityY = 0.0f;
float gravityZ = 0.0f;

// Impact statistics
float currentShockG = 0.0f;
float peakShockG = 0.0f;
uint32_t shockCount = 0;
uint32_t lastShockTime = 0;

// Independent scheduling timers
uint32_t lastAccelSampleMs = 0;
uint32_t lastDashboardMs = 0;
uint32_t lastTempRequestMs = 0;
uint32_t tempConversionStartedAt = 0;

// =====================================================
// 6. GPS HELPER
// =====================================================

bool hasRecentGpsFix() {
    return gps.location.isValid() &&
           gps.location.age() <= GPS_MAX_AGE_MS;
}

// =====================================================
// 7. TEMPERATURE INITIALIZATION
// =====================================================

void initializeTemperatureSensor() {
    tempSensors.begin();

    // 12-bit resolution; maximum conversion time ~750 ms.
    tempSensors.setResolution(12);

    // Do not wait synchronously for conversion.
    tempSensors.setWaitForConversion(false);

    tempSensorCount = tempSensors.getDeviceCount();

    Serial.println("\n[DS18B20 INITIALIZATION]");
    Serial.printf("Sensors found: %d\n", tempSensorCount);

    DeviceAddress address;

    for (int i = 0; i < tempSensorCount; i++) {
        if (tempSensors.getAddress(address, i)) {
            Serial.printf("Sensor %d address: ", i);

            for (uint8_t j = 0; j < 8; j++) {
                Serial.printf("%02X", address[j]);
            }

            Serial.println();
        } else {
            Serial.printf(
                "Could not read address for sensor %d\n", i
            );
        }
    }

    if (tempSensorCount == 0) {
        Serial.println("WARNING: No DS18B20 detected.");
        Serial.println(
            "Check GPIO 4, power, GND and 4.7k pull-up resistor."
        );
    }

    // Make the first conversion eligible immediately.
    lastTempRequestMs = millis() - TEMP_INTERVAL_MS;
}

// =====================================================
// 8. START TEMPERATURE CONVERSION
// =====================================================

void startTemperatureConversion(uint32_t now) {
    tempSensors.requestTemperatures();

    tempConversionStartedAt = now;
    lastTempRequestMs = now;
    tempConversionPending = true;
}

// =====================================================
// 9. TEMPERATURE UPDATE - NON-BLOCKING
// =====================================================

void updateTemperature(uint32_t now) {

    // Check whether the previous conversion has finished.
    if (tempConversionPending &&
        (uint32_t)(now - tempConversionStartedAt)
            >= TEMP_CONVERSION_MS) {

        float reading = tempSensors.getTempCByIndex(0);
        tempConversionPending = false;

        // Reject disconnected, out-of-range and power-on
        // default readings instead of reporting them as normal.
        if (reading == DEVICE_DISCONNECTED_C ||
            reading <= -127.0f ||
            reading > 125.0f) {

            tempDisconnected = true;
            tempReadingAvailable = false;
            tempWarning85 = false;

        } else if (fabsf(reading - 85.0f) < 0.01f) {

            // 85 C may be a DS18B20 power-on default reading.
            tempWarning85 = true;
            tempReadingAvailable = false;
            tempDisconnected = false;

        } else {

            temperatureC = reading;
            tempDisconnected = false;
            tempReadingAvailable = true;
            tempWarning85 = false;
        }
    }

    // Start a new conversion on its own schedule.
    if (!tempConversionPending &&
        (uint32_t)(now - lastTempRequestMs)
            >= TEMP_INTERVAL_MS) {

        startTemperatureConversion(now);
    }
}

// =====================================================
// 10. ADXL345 IMPACT DETECTION
// =====================================================

void updateAccelerometer(uint32_t now) {

    if (!accelReady) {
        return;
    }

    if ((uint32_t)(now - lastAccelSampleMs)
        < ACCEL_INTERVAL_MS) {
        return;
    }

    lastAccelSampleMs = now;

    sensors_event_t event;
    accel.getEvent(&event);

    // ADXL345 acceleration is reported in m/s^2.
    float rawX = event.acceleration.x;
    float rawY = event.acceleration.y;
    float rawZ = event.acceleration.z;

    // Initialize gravity estimate from the first sample.
    if (!gravityInitialized) {
        gravityX = rawX;
        gravityY = rawY;
        gravityZ = rawZ;
        gravityInitialized = true;
        return;
    }

    // Low-pass filter estimates the slowly changing gravity.
    gravityX =
        GRAVITY_FILTER_ALPHA * gravityX +
        (1.0f - GRAVITY_FILTER_ALPHA) * rawX;

    gravityY =
        GRAVITY_FILTER_ALPHA * gravityY +
        (1.0f - GRAVITY_FILTER_ALPHA) * rawY;

    gravityZ =
        GRAVITY_FILTER_ALPHA * gravityZ +
        (1.0f - GRAVITY_FILTER_ALPHA) * rawZ;

    // Subtract gravity estimate to isolate dynamic acceleration.
    float dynamicX = rawX - gravityX;
    float dynamicY = rawY - gravityY;
    float dynamicZ = rawZ - gravityZ;

    float dynamicMagnitude = sqrtf(
        dynamicX * dynamicX +
        dynamicY * dynamicY +
        dynamicZ * dynamicZ
    );

    // Convert dynamic acceleration to g.
    currentShockG = dynamicMagnitude / GRAVITY_MPS2;

    if (currentShockG > peakShockG) {
        peakShockG = currentShockG;
    }

    // Impact detection works regardless of GPS availability.
    if (currentShockG >= IMPACT_THRESHOLD_G &&
        (uint32_t)(now - lastShockTime)
            >= ALERT_COOLDOWN_MS) {

        shockCount++;
        lastShockTime = now;

        Serial.println("\n!!! IMPACT ALERT !!!");
        Serial.printf(
            "Dynamic acceleration: %.2f g\n", currentShockG
        );
        Serial.printf(
            "Total detected impacts: %lu\n",
            (unsigned long)shockCount
        );

        if (hasRecentGpsFix()) {
            Serial.printf(
                "Latitude: %.6f\n", gps.location.lat()
            );
            Serial.printf(
                "Longitude: %.6f\n", gps.location.lng()
            );
        } else {
            Serial.println(
                "GPS position unavailable or outdated."
            );
        }
    }
}

// =====================================================
// 11. DASHBOARD
// =====================================================

void printDashboard() {
    Serial.print("\033[2J\033[H");

    Serial.println("========================================");
    Serial.println("        COLDGUARDIAN MONITOR");
    Serial.println("========================================");

    // ---------- TEMPERATURE ----------
    Serial.println("\n[TEMPERATURE - DS18B20]");
    Serial.printf("PRODUCT     : %s\n", PRODUCT_ID);
    Serial.printf(
        "TEMP LIMITS : %.2f C to %.2f C\n",
        TEMP_MIN_C, TEMP_MAX_C
    );

    if (tempSensorCount == 0) {
        Serial.println("STATUS      : NO SENSOR FOUND AT STARTUP");

    } else if (tempDisconnected) {
        Serial.println("STATUS      : DISCONNECTED / READ ERROR");

    } else if (tempWarning85) {
        Serial.println("STATUS      : WARNING - CHECK 85 C READING");

    } else if (tempReadingAvailable) {
        Serial.printf("TEMPERATURE : %.2f C\n", temperatureC);

        if (temperatureC < TEMP_MIN_C) {
            Serial.println(
                "STATUS      : ALERT - TEMPERATURE TOO LOW"
            );
        } else if (temperatureC > TEMP_MAX_C) {
            Serial.println(
                "STATUS      : ALERT - TEMPERATURE TOO HIGH"
            );
        } else {
            Serial.println(
                "STATUS      : NORMAL - WITHIN LIMITS"
            );
        }

    } else if (tempConversionPending) {
        Serial.println("STATUS      : CONVERSION IN PROGRESS");

    } else {
        Serial.println("STATUS      : WAITING FOR FIRST READING");
    }

    // ---------- GPS ----------
    Serial.println("\n[GPS - NEO-6M]");

    if (hasRecentGpsFix()) {
        Serial.println("GPS STATUS  : FIX VALID");
    } else if (gps.charsProcessed() > 0) {
        Serial.println("GPS STATUS  : SEARCHING FOR FIX");
    } else {
        Serial.println("GPS STATUS  : NO DATA RECEIVED");
    }

    Serial.printf(
        "SATELLITES  : %u\n",
        gps.satellites.isValid()
            ? (unsigned int)gps.satellites.value()
            : 0
    );

    if (hasRecentGpsFix()) {
        Serial.printf(
            "LATITUDE    : %.6f\n", gps.location.lat()
        );
        Serial.printf(
            "LONGITUDE   : %.6f\n", gps.location.lng()
        );
    } else {
        Serial.println("LATITUDE    : --");
        Serial.println("LONGITUDE   : --");
    }

    if (gps.speed.isValid()) {
        Serial.printf(
            "GPS SPEED   : %.2f km/h", gps.speed.kmph()
        );
        Serial.println();
    } else {
        Serial.println("GPS SPEED   : --");
    }

    if (gps.altitude.isValid()) {
        Serial.printf(
            "ALTITUDE    : %.2f m\n", gps.altitude.meters()
        );
    } else {
        Serial.println("ALTITUDE    : --");
    }

    // ---------- ADXL345 ----------
    Serial.println("\n[IMPACT - ADXL345]");

    if (accelReady) {
        Serial.println("STATUS      : CONNECTED");
        Serial.printf(
            "CURRENT     : %.2f g\n", currentShockG
        );
        Serial.printf(
            "PEAK        : %.2f g\n", peakShockG
        );
        Serial.printf(
            "THRESHOLD   : %.2f g\n", IMPACT_THRESHOLD_G
        );
        Serial.printf(
            "IMPACT COUNT: %lu\n",
            (unsigned long)shockCount
        );
    } else {
        Serial.println("STATUS      : SENSOR UNAVAILABLE");
    }

}

// =====================================================
// 12. SETUP - RUNS ONCE
// =====================================================

void setup() {
    Serial.begin(115200);

    Serial.println("\nColdGuardian starting...");
    delay(200);

    // GPS initializes independently.
    GPS_Serial.begin(
        GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN
    );
    Serial.println("GPS UART initialized.");

    // Accelerometer initializes independently.
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    accelReady = accel.begin(ADXL345_ADDRESS);

    if (accelReady) {
        accel.setRange(ADXL345_RANGE_16_G);
        Serial.println("ADXL345 initialized.");
    } else {
        Serial.println(
            "WARNING: ADXL345 not detected; GPS and temperature "
            "will continue."
        );
    }

    // Temperature initializes independently.
    initializeTemperatureSensor();

    Serial.println("\nSensor initialization complete.");
    Serial.println("Starting monitoring...\n");
}

// =====================================================
// 13. LOOP - RUNS REPEATEDLY
// =====================================================

void loop() {
    uint32_t now = millis();

    // 1. Process GPS bytes as quickly as possible.
    while (GPS_Serial.available() > 0) {
        gps.encode((char)GPS_Serial.read());
    }

    // 2. Update temperature conversion state.
    updateTemperature(now);

    // 3. Sample accelerometer on its own schedule.
    updateAccelerometer(now);

    // 4. Refresh dashboard independently.
    if ((uint32_t)(now - lastDashboardMs)
        >= DASHBOARD_INTERVAL_MS) {

        lastDashboardMs = now;
        printDashboard();
    }
}