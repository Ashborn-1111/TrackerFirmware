#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <TinyGPSPlus.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include <math.h>
#include <time.h>
#include <esp_system.h>

#include "secrets.h"  // Create include/secrets.h from the provided example.

// =====================================================
// 1. HARDWARE PIN CONFIGURATION (existing working pins)
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
// 2. TIMING / FIRMWARE SETTINGS
// =====================================================

constexpr char FIRMWARE_VERSION[] = "0.2.0-mqtt";
constexpr char HARDWARE_MODEL[] = "esp32-devkit-v1";

constexpr uint32_t ACCEL_INTERVAL_MS = 10;
constexpr uint32_t DASHBOARD_INTERVAL_MS = 1000;
constexpr uint32_t TEMP_INTERVAL_MS = 2000;
constexpr uint32_t TEMP_CONVERSION_MS = 750;
constexpr uint32_t GPS_MAX_AGE_MS = 5000;

constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 10000;
constexpr uint32_t MQTT_RETRY_INTERVAL_MS = 5000;
constexpr uint32_t REGISTRATION_RETRY_INTERVAL_MS = 10000;
constexpr uint32_t MQTT_KEEPALIVE_MS = 30000;
constexpr uint32_t MQTT_CONNECT_TIMEOUT_MS = 2500;

constexpr uint8_t OUTBOX_CAPACITY = 12;
constexpr float GRAVITY_MPS2 = 9.80665f;
constexpr float GRAVITY_FILTER_ALPHA = 0.90f;
constexpr uint32_t MAX_SHOCK_EVENT_DURATION_MS = 2000;

// =====================================================
// 3. SENSOR OBJECTS
// =====================================================

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);
OneWire oneWire(TEMP_PIN);
DallasTemperature tempSensors(&oneWire);

// =====================================================
// 4. MQTT / NETWORK OBJECTS
// =====================================================

WiFiClient mqttTransport;
MqttClient mqttClient(mqttTransport);

const String deviceId = String(COLDGUARDIAN_DEVICE_ID);
const String messageTopic = String("coldchain/") + deviceId + "/messages";
const String ackTopic = String("coldchain/") + deviceId + "/registration/ack";
const String configTopic = String("coldchain/") + deviceId + "/config";
const String mqttClientId = String("coldguardian-") + deviceId;

bool ntpConfigured = false;
bool registrationAcknowledged = false;
bool registrationQueued = false;
bool configReceived = false;
bool deviceAssigned = false;
bool monitoringEnabled = false;
uint32_t configVersion = 0;
uint32_t nextMessageSeq = 0;
uint32_t reportingIntervalMs = 15000;
uint32_t lastWifiRetryMs = 0;
uint32_t lastMqttAttemptMs = 0;
uint32_t lastRegistrationAttemptMs = 0;
uint32_t lastTelemetryMs = 0;

float configuredMinTemperatureC = NAN;
float configuredMaxTemperatureC = NAN;
float configuredShockThresholdG = NAN;
String configuredProductSku;
String configuredProductName;
String bootId;

// Small RAM outbox. QoS 1 messages remain queued until PUBACK succeeds.
enum class OutboundKind : uint8_t {
    Registration,
    Telemetry,
    Event
};

struct PendingMessage {
    String body;
    OutboundKind kind = OutboundKind::Event;
    uint32_t seq = 0;
    uint32_t telemetryShockCountSnapshot = 0;
    float telemetryPeakSnapshotG = 0.0f;
};

PendingMessage outbox[OUTBOX_CAPACITY];
uint8_t outboxHead = 0;
uint8_t outboxCount = 0;

// =====================================================
// 5. SENSOR STATUS / LIVE VALUES (existing sensor logic)
// =====================================================

bool accelReady = false;
bool accelReadingAvailable = false;

int tempSensorCount = 0;
bool tempConversionPending = false;
bool tempReadingAvailable = false;
bool tempDisconnected = false;
bool tempWarning85 = false;
float temperatureC = NAN;

bool gravityInitialized = false;
float gravityX = 0.0f;
float gravityY = 0.0f;
float gravityZ = 0.0f;

float currentShockG = 0.0f;
float peakShockG = 0.0f;               // Peak since boot, for dashboard only.
float peakShockSinceLastReportG = 0.0f;
uint32_t shockCountTotal = 0;
uint32_t shockCountSinceLastReport = 0;

bool temperatureBreachActive = false;
bool shockEventActive = false;
bool shockNeedsRearm = false;
uint32_t shockEventStartedAtMs = 0;
float shockEventPeakG = 0.0f;

uint32_t lastAccelSampleMs = 0;
uint32_t lastDashboardMs = 0;
uint32_t lastTempRequestMs = 0;
uint32_t tempConversionStartedAt = 0;

// =====================================================
// 6. CLOCK / COMMON HELPERS
// =====================================================

bool getUtcTimestamp(char *buffer, size_t bufferSize) {
    const time_t now = time(nullptr);
    // Do not send 1970-era timestamps: the backend expects ISO-8601 UTC.
    if (now < 1700000000) {
        return false;
    }

    struct tm utcTime;
    if (gmtime_r(&now, &utcTime) == nullptr) {
        return false;
    }

    return strftime(buffer, bufferSize, "%Y-%m-%dT%H:%M:%SZ", &utcTime) > 0;
}

bool hasRecentGpsFix() {
    return gps.location.isValid() && gps.location.age() <= GPS_MAX_AGE_MS;
}

bool canMonitorDevice() {
    // Continue local detection while MQTT/Wi-Fi is down, using the last valid config.
    return configReceived && deviceAssigned && monitoringEnabled &&
           isfinite(configuredMinTemperatureC) &&
           isfinite(configuredMaxTemperatureC) &&
           isfinite(configuredShockThresholdG) &&
           configuredMinTemperatureC <= configuredMaxTemperatureC &&
           configuredShockThresholdG > 0.0f;
}

bool canPublishTelemetry() {
    return mqttClient.connected() && registrationAcknowledged && canMonitorDevice();
}

// =====================================================
// 7. MQTT OUTBOX / ENVELOPE
// =====================================================

bool enqueueEnvelope(const char *messageType,
                    const JsonDocument &payloadDocument,
                    OutboundKind kind) {
    if (outboxCount >= OUTBOX_CAPACITY) {
        Serial.println("WARNING: MQTT outbox full; message was not queued.");
        return false;
    }

    char timestamp[32];
    if (!getUtcTimestamp(timestamp, sizeof(timestamp))) {
        Serial.println("Waiting for NTP clock sync; message not queued yet.");
        return false;
    }

    StaticJsonDocument<1536> envelope;
    envelope["schema_version"] = 1;
    envelope["message_type"] = messageType;
    envelope["device_id"] = deviceId;
    envelope["firmware_version"] = FIRMWARE_VERSION;
    envelope["boot_id"] = bootId;
    envelope["message_seq"] = nextMessageSeq;
    envelope["config_version"] = configVersion;
    envelope["observed_at"] = timestamp;
    envelope["uptime_ms"] = millis();
    envelope["payload"].set(payloadDocument.as<JsonObjectConst>());

    PendingMessage &slot = outbox[(outboxHead + outboxCount) % OUTBOX_CAPACITY];
    slot.body = "";
    slot.body.reserve(measureJson(envelope) + 1);
    serializeJson(envelope, slot.body);
    slot.kind = kind;
    slot.seq = nextMessageSeq;
    slot.telemetryShockCountSnapshot = 0;
    slot.telemetryPeakSnapshotG = 0.0f;

    if (kind == OutboundKind::Telemetry) {
        slot.telemetryShockCountSnapshot = shockCountSinceLastReport;
        slot.telemetryPeakSnapshotG = peakShockSinceLastReportG;
    }

    ++outboxCount;
    ++nextMessageSeq;
    if (kind == OutboundKind::Registration) {
        registrationQueued = true;
    }
    return true;
}

void flushOneQueuedMessage() {
    if (!mqttClient.connected() || outboxCount == 0) {
        return;
    }

    PendingMessage &message = outbox[outboxHead];

    // ArduinoMqttClient supports QoS 1. endMessage waits for PUBACK.
    // If it fails, keep the exact same body/sequence at the queue head.
    if (!mqttClient.beginMessage(messageTopic, false, 1)) {
        Serial.println("MQTT beginMessage failed; retaining queued message.");
        return;
    }

    const size_t written = mqttClient.print(message.body);
    if (written != message.body.length()) {
        Serial.println("MQTT payload write incomplete; retaining queued message.");
        mqttClient.stop();
        return;
    }

    if (!mqttClient.endMessage()) {
        Serial.println("MQTT PUBACK not received; retaining same message for retry.");
        return;
    }

    if (message.kind == OutboundKind::Registration) {
        registrationQueued = false;
    } else if (message.kind == OutboundKind::Telemetry) {
        if (shockCountSinceLastReport >= message.telemetryShockCountSnapshot) {
            shockCountSinceLastReport -= message.telemetryShockCountSnapshot;
        } else {
            shockCountSinceLastReport = 0;
        }
        if (peakShockSinceLastReportG <= message.telemetryPeakSnapshotG + 0.0001f) {
            peakShockSinceLastReportG = accelReadingAvailable ? currentShockG : 0.0f;
        }
    }

    Serial.printf("MQTT delivered: seq=%lu, type=%s\n",
                  (unsigned long)message.seq,
                  message.kind == OutboundKind::Registration ? "register" :
                  message.kind == OutboundKind::Telemetry ? "telemetry" : "event");

    message.body = "";
    outboxHead = (outboxHead + 1) % OUTBOX_CAPACITY;
    --outboxCount;
}

// =====================================================
// 8. REGISTRATION / CONFIG MESSAGE HANDLING
// =====================================================

void publishRegistration() {
    StaticJsonDocument<256> payload;
    payload["hardware_model"] = HARDWARE_MODEL;
    JsonObject capabilities = payload.createNestedObject("capabilities");
    capabilities["temperature"] = true;
    capabilities["accelerometer"] = true;
    capabilities["gnss"] = true;
    capabilities["epaper"] = false;

    if (enqueueEnvelope("register", payload, OutboundKind::Registration)) {
        lastRegistrationAttemptMs = millis();
        Serial.println("Queued device registration.");
    }
}

void handleConfigMessage(const String &body) {
    StaticJsonDocument<1536> config;
    const DeserializationError error = deserializeJson(config, body);
    if (error) {
        Serial.printf("Ignoring malformed device config: %s\n", error.c_str());
        return;
    }

    if (String(config["message_type"] | "") != "device_config") {
        return;
    }
    if (String(config["device_id"] | "") != deviceId) {
        Serial.println("Ignoring config for another device ID.");
        return;
    }
    if (!config["config_version"].is<uint32_t>()) {
        Serial.println("Ignoring config without a valid config_version.");
        return;
    }

    const uint32_t incomingVersion = config["config_version"].as<uint32_t>();
    if (configReceived && incomingVersion < configVersion) {
        Serial.println("Ignoring stale device config.");
        return;
    }

    const bool incomingAssigned = config["assignment"]["assigned"] | false;
    const bool incomingMonitoring = config["monitoring"]["enabled"] | false;
    float newMin = NAN;
    float newMax = NAN;
    float newShock = NAN;
    uint32_t newReportIntervalMs = 15000;

    if (incomingAssigned && incomingMonitoring) {
        JsonVariantConst minValue = config["monitoring"]["temperature"]["min_temperature_c"];
        JsonVariantConst maxValue = config["monitoring"]["temperature"]["max_temperature_c"];
        JsonVariantConst shockValue = config["monitoring"]["shock"]["threshold_g"];

        if (minValue.isNull() || maxValue.isNull() || shockValue.isNull()) {
            Serial.println("Rejecting assigned config with missing thresholds.");
            return;
        }

        newMin = minValue.as<float>();
        newMax = maxValue.as<float>();
        newShock = shockValue.as<float>();
        if (!isfinite(newMin) || !isfinite(newMax) || !isfinite(newShock) ||
            newMin > newMax || newShock <= 0.0f) {
            Serial.println("Rejecting assigned config with invalid thresholds.");
            return;
        }

        uint32_t seconds = config["monitoring"]["reporting_interval_seconds"] | 15;
        if (seconds == 0 || seconds > 3600) {
            Serial.println("Rejecting config with invalid reporting interval.");
            return;
        }
        newReportIntervalMs = seconds * 1000UL;
    }

    configVersion = incomingVersion;
    configReceived = true;
    deviceAssigned = incomingAssigned;
    monitoringEnabled = incomingAssigned && incomingMonitoring;
    configuredMinTemperatureC = newMin;
    configuredMaxTemperatureC = newMax;
    configuredShockThresholdG = newShock;
    reportingIntervalMs = newReportIntervalMs;
    configuredProductSku = config["package"]["product_sku"] | "";
    configuredProductName = config["package"]["product_name"] | "";

    // A new package config is a new monitoring context.
    temperatureBreachActive = false;
    shockEventActive = false;
    shockNeedsRearm = false;
    lastTelemetryMs = millis() - reportingIntervalMs; // report promptly

    Serial.printf("Applied config v%lu: assigned=%s, monitoring=%s\n",
                  (unsigned long)configVersion,
                  deviceAssigned ? "true" : "false",
                  monitoringEnabled ? "true" : "false");
    if (monitoringEnabled) {
        Serial.printf("Temperature limits: %.2f..%.2f C; shock threshold: %.2f g\n",
                      configuredMinTemperatureC, configuredMaxTemperatureC,
                      configuredShockThresholdG);
    }
}

void onMqttMessage(int messageSize) {
    const String topic = mqttClient.messageTopic();
    String body;
    body.reserve(messageSize + 1);
    while (mqttClient.available()) {
        body += static_cast<char>(mqttClient.read());
    }

    if (topic == ackTopic) {
        StaticJsonDocument<512> ack;
        if (deserializeJson(ack, body)) {
            Serial.println("Malformed registration acknowledgement.");
            return;
        }
        if (String(ack["device_id"] | "") != deviceId) {
            return;
        }

        const String status = ack["status"] | "";
        if (status == "registered" || status == "already_registered") {
            registrationAcknowledged = true;
            Serial.printf("Registration acknowledged: %s\n", status.c_str());
        } else {
            registrationAcknowledged = false;
            Serial.printf("Registration rejected: %s\n",
                          String(ack["reason"] | "unknown reason").c_str());
        }
    } else if (topic == configTopic) {
        handleConfigMessage(body);
    }
}

// =====================================================
// 9. WIFI / MQTT RECONNECT LOOP
// =====================================================

void beginWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiRetryMs = millis();
    Serial.printf("Connecting to Wi-Fi SSID '%s'...\n", WIFI_SSID);
}

void serviceNetwork(uint32_t now) {
    if (WiFi.status() != WL_CONNECTED) {
        if (mqttClient.connected()) {
            mqttClient.stop();
        }
        registrationAcknowledged = false;
        if ((uint32_t)(now - lastWifiRetryMs) >= WIFI_RETRY_INTERVAL_MS) {
            lastWifiRetryMs = now;
            Serial.println("Wi-Fi disconnected; retrying connection.");
            WiFi.reconnect();
        }
        return;
    }

    if (!ntpConfigured) {
        // UTC timestamps are required by the backend message contract.
        configTime(0, 0, "pool.ntp.org", "time.google.com");
        ntpConfigured = true;
        Serial.println("NTP time synchronization requested.");
    }

    if (!mqttClient.connected()) {
        if ((uint32_t)(now - lastMqttAttemptMs) < MQTT_RETRY_INTERVAL_MS) {
            return;
        }
        lastMqttAttemptMs = now;

        mqttClient.stop();
        mqttClient.setId(mqttClientId);
        mqttClient.setKeepAliveInterval(MQTT_KEEPALIVE_MS);
        mqttClient.setConnectionTimeout(MQTT_CONNECT_TIMEOUT_MS);
        if (MQTT_USERNAME[0] != '\0') {
            mqttClient.setUsernamePassword(MQTT_USERNAME, MQTT_PASSWORD);
        }

        Serial.printf("Connecting MQTT to %s:%u...\n",
                      MQTT_BROKER_HOST, (unsigned int)MQTT_BROKER_PORT);
        if (!mqttClient.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT)) {
            Serial.printf("MQTT connection failed (error %d).\n",
                          mqttClient.connectError());
            return;
        }

        registrationAcknowledged = false;
        if (!mqttClient.subscribe(ackTopic, 1) || !mqttClient.subscribe(configTopic, 1)) {
            Serial.println("MQTT subscription failed; disconnecting to retry.");
            mqttClient.stop();
            return;
        }
        Serial.println("MQTT connected; subscribed to registration ACK and config.");
    }

    mqttClient.poll();

    char timestamp[32];
    const bool clockReady = getUtcTimestamp(timestamp, sizeof(timestamp));
    if (clockReady && !registrationAcknowledged && !registrationQueued &&
        (lastRegistrationAttemptMs == 0 ||
         (uint32_t)(now - lastRegistrationAttemptMs) >= REGISTRATION_RETRY_INTERVAL_MS)) {
        publishRegistration();
    }
}

// =====================================================
// 10. DS18B20 TEMPERATURE INITIALIZATION / UPDATE
// =====================================================

void initializeTemperatureSensor() {
    tempSensors.begin();
    tempSensors.setResolution(12);
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
            Serial.printf("Could not read address for sensor %d\n", i);
        }
    }

    if (tempSensorCount == 0) {
        Serial.println("WARNING: No DS18B20 detected. Check GPIO 4, power, GND and 4.7k pull-up.");
    }
    lastTempRequestMs = millis() - TEMP_INTERVAL_MS;
}

void startTemperatureConversion(uint32_t now) {
    tempSensors.requestTemperatures();
    tempConversionStartedAt = now;
    lastTempRequestMs = now;
    tempConversionPending = true;
}

void updateTemperature(uint32_t now) {
    if (tempConversionPending &&
        (uint32_t)(now - tempConversionStartedAt) >= TEMP_CONVERSION_MS) {
        const float reading = tempSensors.getTempCByIndex(0);
        tempConversionPending = false;

        if (reading == DEVICE_DISCONNECTED_C || reading <= -127.0f || reading > 125.0f) {
            tempDisconnected = true;
            tempReadingAvailable = false;
            tempWarning85 = false;
        } else if (fabsf(reading - 85.0f) < 0.01f) {
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

    if (!tempConversionPending &&
        (uint32_t)(now - lastTempRequestMs) >= TEMP_INTERVAL_MS) {
        startTemperatureConversion(now);
    }
}

// =====================================================
// 11. DEVICE-SIDE TEMPERATURE BREACH DETECTION
// =====================================================

void evaluateTemperatureBreach() {
    if (!canMonitorDevice() || !tempReadingAvailable || !isfinite(temperatureC)) {
        return;
    }

    const bool belowMinimum = temperatureC < configuredMinTemperatureC;
    const bool aboveMaximum = temperatureC > configuredMaxTemperatureC;
    if (!belowMinimum && !aboveMaximum) {
        temperatureBreachActive = false;
        return;
    }

    // Only emit an event on entering a breach, not on every loop/sample.
    if (temperatureBreachActive) {
        return;
    }

    StaticJsonDocument<384> payload;
    payload["type"] = "temperature_breach";
    payload["temperature_c"] = temperatureC;
    payload["breach_direction"] = belowMinimum ? "below_minimum" : "above_maximum";
    payload["min_temperature_c"] = configuredMinTemperatureC;
    payload["max_temperature_c"] = configuredMaxTemperatureC;

    if (enqueueEnvelope("event", payload, OutboundKind::Event)) {
        temperatureBreachActive = true;
        Serial.printf("Queued temperature breach: %.2f C (%s).\n",
                      temperatureC, belowMinimum ? "below minimum" : "above maximum");
    }
}

// =====================================================
// 12. ADXL345 SAMPLING / SHOCK EVENT DETECTION
// =====================================================

void finishShockEvent(uint32_t now) {
    if (!shockEventActive) {
        return;
    }

    const uint32_t durationMs = now - shockEventStartedAtMs;
    StaticJsonDocument<384> payload;
    payload["type"] = "shock_detected";
    payload["peak_acceleration_magnitude_g"] = shockEventPeakG;
    payload["threshold_g"] = configuredShockThresholdG;
    payload["duration_ms"] = durationMs;

    if (enqueueEnvelope("event", payload, OutboundKind::Event)) {
        ++shockCountTotal;
        ++shockCountSinceLastReport;
        Serial.printf("Queued shock event: peak=%.2f g, duration=%lu ms\n",
                      shockEventPeakG, (unsigned long)durationMs);
    } else {
        Serial.println("WARNING: Could not queue shock event.");
    }

    shockEventActive = false;
    shockNeedsRearm = currentShockG >= configuredShockThresholdG * 0.5f;
}

void updateAccelerometer(uint32_t now) {
    if (!accelReady || (uint32_t)(now - lastAccelSampleMs) < ACCEL_INTERVAL_MS) {
        return;
    }
    lastAccelSampleMs = now;

    sensors_event_t event;
    accel.getEvent(&event);
    const float rawX = event.acceleration.x;
    const float rawY = event.acceleration.y;
    const float rawZ = event.acceleration.z;

    if (!gravityInitialized) {
        gravityX = rawX;
        gravityY = rawY;
        gravityZ = rawZ;
        gravityInitialized = true;
        return;
    }

    gravityX = GRAVITY_FILTER_ALPHA * gravityX + (1.0f - GRAVITY_FILTER_ALPHA) * rawX;
    gravityY = GRAVITY_FILTER_ALPHA * gravityY + (1.0f - GRAVITY_FILTER_ALPHA) * rawY;
    gravityZ = GRAVITY_FILTER_ALPHA * gravityZ + (1.0f - GRAVITY_FILTER_ALPHA) * rawZ;

    const float dynamicX = rawX - gravityX;
    const float dynamicY = rawY - gravityY;
    const float dynamicZ = rawZ - gravityZ;
    const float dynamicMagnitude = sqrtf(
        dynamicX * dynamicX + dynamicY * dynamicY + dynamicZ * dynamicZ);

    currentShockG = dynamicMagnitude / GRAVITY_MPS2;
    accelReadingAvailable = true;
    if (currentShockG > peakShockG) peakShockG = currentShockG;
    if (currentShockG > peakShockSinceLastReportG) {
        peakShockSinceLastReportG = currentShockG;
    }

    if (!canMonitorDevice()) {
        shockEventActive = false;
        shockNeedsRearm = false;
        return;
    }

    if (shockNeedsRearm && currentShockG < configuredShockThresholdG * 0.5f) {
        shockNeedsRearm = false;
    }

    if (!shockEventActive && !shockNeedsRearm &&
        currentShockG >= configuredShockThresholdG) {
        shockEventActive = true;
        shockEventStartedAtMs = now;
        shockEventPeakG = currentShockG;
    } else if (shockEventActive) {
        if (currentShockG > shockEventPeakG) shockEventPeakG = currentShockG;

        if (currentShockG < configuredShockThresholdG * 0.5f ||
            (uint32_t)(now - shockEventStartedAtMs) >= MAX_SHOCK_EVENT_DURATION_MS) {
            finishShockEvent(now);
        }
    }
}

// =====================================================
// 13. PERIODIC TELEMETRY
// =====================================================

void queueTelemetryIfDue(uint32_t now) {
    if (!canPublishTelemetry()) {
        return;
    }
    if ((uint32_t)(now - lastTelemetryMs) < reportingIntervalMs) {
        return;
    }

    // Send alerts/registration first; don't fill the outbox with old telemetry.
    if (outboxCount > 0) {
        lastTelemetryMs = now;
        return;
    }

    StaticJsonDocument<768> payload;

    JsonObject temperature = payload.createNestedObject("temperature");
    const bool temperatureValid = tempReadingAvailable && isfinite(temperatureC);
    temperature["valid"] = temperatureValid;
    if (temperatureValid) temperature["celsius"] = temperatureC;
    else temperature["celsius"] = nullptr;

    JsonObject motion = payload.createNestedObject("motion");
    motion["valid"] = accelReady && accelReadingAvailable;
    if (accelReady && accelReadingAvailable) {
        motion["peak_acceleration_magnitude_g"] = peakShockSinceLastReportG;
    } else {
        motion["peak_acceleration_magnitude_g"] = nullptr;
    }
    motion["shock_count_since_last_report"] = shockCountSinceLastReport;

    JsonObject location = payload.createNestedObject("location");
    const bool gpsFixValid = hasRecentGpsFix();
    location["fix_valid"] = gpsFixValid;
    if (gpsFixValid) {
        location["latitude"] = gps.location.lat();
        location["longitude"] = gps.location.lng();
    } else {
        location["latitude"] = nullptr;
        location["longitude"] = nullptr;
    }
    // TinyGPSPlus HDOP is not a meter value, so don't mislabel it as accuracy_m.
    location["accuracy_m"] = nullptr;
    location["satellites_used"] = gps.satellites.isValid()
        ? gps.satellites.value() : 0;

    if (enqueueEnvelope("telemetry", payload, OutboundKind::Telemetry)) {
        lastTelemetryMs = now;
    }
}

// =====================================================
// 14. DASHBOARD (same sensor fields + network/config state)
// =====================================================

void printDashboard() {
    Serial.print("\033[2J\033[H");
    Serial.println("========================================");
    Serial.println("        COLDGUARDIAN MONITOR");
    Serial.println("========================================");

    Serial.println("\n[NETWORK / MQTT]");
    Serial.printf("WIFI        : %s\n", WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("WIFI IP     : %s\n", WiFi.localIP().toString().c_str());
    }
    Serial.printf("MQTT        : %s\n", mqttClient.connected() ? "CONNECTED" : "DISCONNECTED");
    Serial.printf("REGISTERED  : %s\n", registrationAcknowledged ? "YES" : "NO");
    Serial.printf("CONFIG      : %s (version %lu)\n", configReceived ? "RECEIVED" : "WAITING",
                  (unsigned long)configVersion);
    Serial.printf("ASSIGNED    : %s\n", deviceAssigned ? "YES" : "NO");
    Serial.printf("MONITORING  : %s\n", canMonitorDevice() ? "ENABLED" : "DISABLED");
    Serial.printf("MQTT QUEUE  : %u\n", (unsigned int)outboxCount);

    Serial.println("\n[TEMPERATURE - DS18B20]");
    Serial.printf("PRODUCT SKU : %s\n", configuredProductSku.length() ? configuredProductSku.c_str() : "UNASSIGNED");
    if (canMonitorDevice()) {
        Serial.printf("PRODUCT     : %s\n", configuredProductName.c_str());
        Serial.printf("TEMP LIMITS : %.2f C to %.2f C\n",
                      configuredMinTemperatureC, configuredMaxTemperatureC);
    } else {
        Serial.println("TEMP LIMITS : WAITING FOR ASSIGNED BACKEND CONFIG");
    }

    if (tempSensorCount == 0) {
        Serial.println("STATUS      : NO SENSOR FOUND AT STARTUP");
    } else if (tempDisconnected) {
        Serial.println("STATUS      : DISCONNECTED / READ ERROR");
    } else if (tempWarning85) {
        Serial.println("STATUS      : WARNING - CHECK 85 C READING");
    } else if (tempReadingAvailable) {
        Serial.printf("TEMPERATURE : %.2f C\n", temperatureC);
        if (!canMonitorDevice()) {
            Serial.println("STATUS      : READING OK; NO ACTIVE PACKAGE CONFIG");
        } else if (temperatureC < configuredMinTemperatureC) {
            Serial.println("STATUS      : ALERT - TEMPERATURE TOO LOW");
        } else if (temperatureC > configuredMaxTemperatureC) {
            Serial.println("STATUS      : ALERT - TEMPERATURE TOO HIGH");
        } else {
            Serial.println("STATUS      : NORMAL - WITHIN LIMITS");
        }
    } else if (tempConversionPending) {
        Serial.println("STATUS      : CONVERSION IN PROGRESS");
    } else {
        Serial.println("STATUS      : WAITING FOR FIRST READING");
    }

    Serial.println("\n[GPS - NEO-6M]");
    if (hasRecentGpsFix()) {
        Serial.println("GPS STATUS  : FIX VALID");
    } else if (gps.charsProcessed() > 0) {
        Serial.println("GPS STATUS  : SEARCHING FOR FIX");
    } else {
        Serial.println("GPS STATUS  : NO DATA RECEIVED");
    }
    Serial.printf("SATELLITES  : %u\n", gps.satellites.isValid()
        ? (unsigned int)gps.satellites.value() : 0);
    if (hasRecentGpsFix()) {
        Serial.printf("LATITUDE    : %.6f\n", gps.location.lat());
        Serial.printf("LONGITUDE   : %.6f\n", gps.location.lng());
    } else {
        Serial.println("LATITUDE    : --");
        Serial.println("LONGITUDE   : --");
    }
    if (gps.speed.isValid()) Serial.printf("GPS SPEED   : %.2f km/h\n", gps.speed.kmph());
    else Serial.println("GPS SPEED   : --");
    if (gps.altitude.isValid()) Serial.printf("ALTITUDE    : %.2f m\n", gps.altitude.meters());
    else Serial.println("ALTITUDE    : --");

    Serial.println("\n[IMPACT - ADXL345]");
    if (accelReady) {
        Serial.println("STATUS      : CONNECTED");
        Serial.printf("CURRENT     : %.2f g\n", currentShockG);
        Serial.printf("PEAK BOOT   : %.2f g\n", peakShockG);
        if (canMonitorDevice()) {
            Serial.printf("THRESHOLD   : %.2f g\n", configuredShockThresholdG);
        } else {
            Serial.println("THRESHOLD   : WAITING FOR BACKEND CONFIG");
        }
        Serial.printf("IMPACT COUNT: %lu\n", (unsigned long)shockCountTotal);
    } else {
        Serial.println("STATUS      : SENSOR UNAVAILABLE");
    }
}

// =====================================================
// 15. SETUP
// =====================================================

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\nColdGuardian firmware + MQTT starting...");

    char bootIdBuffer[9];
    snprintf(bootIdBuffer, sizeof(bootIdBuffer), "%08lx",
             static_cast<unsigned long>(esp_random()));
    bootId = bootIdBuffer;
    Serial.printf("Device ID: %s | boot_id: %s\n", deviceId.c_str(), bootId.c_str());

    GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("GPS UART initialized.");

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    accelReady = accel.begin(ADXL345_ADDRESS);
    if (accelReady) {
        accel.setRange(ADXL345_RANGE_16_G);
        Serial.println("ADXL345 initialized.");
    } else {
        Serial.println("WARNING: ADXL345 not detected; GPS and temperature continue.");
    }

    initializeTemperatureSensor();

    mqttClient.onMessage(onMqttMessage);
    beginWifi();

    Serial.println("Sensor initialization complete. Monitoring loop started.");
    Serial.println("Temperature/shock limits will come from the assigned package config.");
}

// =====================================================
// 16. MAIN LOOP
// =====================================================

void loop() {
    uint32_t now = millis();

    // Keep network checks and MQTT polling in the main loop.
    serviceNetwork(now);

    // Process GPS bytes as quickly as possible.
    while (GPS_Serial.available() > 0) {
        gps.encode(static_cast<char>(GPS_Serial.read()));
    }

    // Keep sensor schedules independent, as in the original working code.
    updateTemperature(now);
    updateAccelerometer(now);
    evaluateTemperatureBreach();
    queueTelemetryIfDue(now);

    // Send at most one QoS 1 message per loop; unacknowledged messages stay queued.
    flushOneQueuedMessage();

    if ((uint32_t)(now - lastDashboardMs) >= DASHBOARD_INTERVAL_MS) {
        lastDashboardMs = now;
        printDashboard();
    }
}
