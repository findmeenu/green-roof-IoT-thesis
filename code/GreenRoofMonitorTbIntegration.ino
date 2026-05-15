#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ─────────────────────────────────────────────
// WIFI CONFIG
// ─────────────────────────────────────────────
const char* WIFI_SSID     = "Tele2_9c6ee9";
const char* WIFI_PASSWORD = "dtmhvwwm";

// ─────────────────────────────────────────────
// THINGSBOARD CONFIG
// ─────────────────────────────────────────────
const char* TB_SERVER = "eu.thingsboard.cloud";
const int   TB_PORT   = 1883;
const char* TB_TOKEN  = "EeJJUtIBncsHLNC4wAgj";

// ─────────────────────────────────────────────
// LOCAL MQTT BROKER FOR TASMOTA
// This should be your laptop IP running Mosquitto
// ─────────────────────────────────────────────
const char* LOCAL_BROKER = "192.168.0.88";
const int   LOCAL_BROKER_PORT = 1883;

// Tasmota topic
// Must match Tasmota MQTT Topic field
const char* TASMOTA_TOPIC = "cmnd/plug1/POWER";

// ─────────────────────────────────────────────
// SENSOR CONFIG
// ─────────────────────────────────────────────
const int MOISTURE_PIN = 4;

const int MOISTURE_RAW_DRY = 5500;
const int MOISTURE_RAW_WET = 10;

// ─────────────────────────────────────────────
// TIMING
// ─────────────────────────────────────────────
const unsigned long TELEMETRY_INTERVAL_MS = 5000;

// ─────────────────────────────────────────────
// MQTT TOPICS
// ─────────────────────────────────────────────
const char* TOPIC_TELEMETRY = "v1/devices/me/telemetry";
const char* TOPIC_RPC_SUB   = "v1/devices/me/rpc/request/+";

// ─────────────────────────────────────────────
// GLOBALS
// ─────────────────────────────────────────────
WiFiClient tbWifiClient;
PubSubClient tbMqtt(tbWifiClient);

WiFiClient localWifiClient;
PubSubClient localMqtt(localWifiClient);

unsigned long lastTelemetryMs = 0;
unsigned long messageId = 0;

bool pumpState = false;

// ─────────────────────────────────────────────
// FORWARD DECLARATIONS
// ─────────────────────────────────────────────
void connectWiFi();
void reconnectThingsBoard();
void connectLocalMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void sendTelemetry(const char* eventType, const char* source,
                   float moisturePct, int rawVal);
float readMoisturePercent(int* rawOut);
void controlTasmotaPlug(bool on);
String extractRequestId(const char* topic);

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== ESP32 ThingsBoard Logger + Tasmota Controller ===");

  pinMode(MOISTURE_PIN, INPUT);

  connectWiFi();

  tbMqtt.setServer(TB_SERVER, TB_PORT);
  tbMqtt.setCallback(mqttCallback);
  tbMqtt.setBufferSize(512);

  localMqtt.setServer(LOCAL_BROKER, LOCAL_BROKER_PORT);

  reconnectThingsBoard();
  connectLocalMQTT();

  Serial.println("=== Setup complete ===");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  // Add this at the top of loop()
if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Lost connection, reconnecting...");
    connectWiFi();
}
  if (!tbMqtt.connected()) {
    reconnectThingsBoard();
  }

  if (!localMqtt.connected()) {
    connectLocalMQTT();
  }

  tbMqtt.loop();
  localMqtt.loop();

  unsigned long now = millis();

  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;

    int rawVal = 0;
    float moisturePct = readMoisturePercent(&rawVal);

    Serial.printf("[SENSOR] raw=%d moisture=%.1f%% pump=%s\n",
                  rawVal, moisturePct, pumpState ? "ON" : "OFF");

    sendTelemetry("sensor_reading", "esp32_logger",
                  moisturePct, rawVal);
  }
}

// ============================================================
// THINGSBOARD RPC CALLBACK
// Button should send:
// {"method":"setPump","params":{"pump":true}}
// {"method":"setPump","params":{"pump":false}}
// ============================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("[RPC] Topic: ");
  Serial.println(topic);

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload, length);

  if (err) {
    Serial.print("[RPC] JSON parse failed: ");
    Serial.println(err.c_str());
    return;
  }

  const char* method = doc["method"] | "";

  Serial.print("[RPC] Method: ");
  Serial.println(method);

  if (strcmp(method, "setPump") == 0) {
    bool pumpCmd = doc["params"]["pump"] | false;

    Serial.printf("[RPC] Pump command received: %s\n",
                  pumpCmd ? "ON" : "OFF");

    controlTasmotaPlug(pumpCmd);
    pumpState = pumpCmd;

    int rawVal = 0;
    float moisturePct = readMoisturePercent(&rawVal);

    sendTelemetry("manual_pump_control", "thingsboard_button",
                  moisturePct, rawVal);

    String requestId = extractRequestId(topic);
    String responseTopic = "v1/devices/me/rpc/response/" + requestId;

    StaticJsonDocument<128> response;
    response["status"] = "ok";
    response["pump_state"] = pumpState ? "ON" : "OFF";

    char responseBuffer[128];
    serializeJson(response, responseBuffer);

    tbMqtt.publish(responseTopic.c_str(), responseBuffer);

    Serial.println("[RPC] Response sent to ThingsBoard");
  }
}

// ============================================================
// SEND TELEMETRY TO THINGSBOARD
// ============================================================
void sendTelemetry(const char* eventType, const char* source,
                   float moisturePct, int rawVal) {
  StaticJsonDocument<512> doc;

  messageId++;

  doc["event_type"] = eventType;
  doc["source"] = source;
  doc["raw_moisture_value"] = rawVal;
  doc["moisture_percent"] = round(moisturePct * 10.0) / 10.0;
  doc["pump_state"] = pumpState ? 1 : 0;
  doc["pump_state_text"] = pumpState ? "ON" : "OFF";
  doc["message_id"] = messageId;
  doc["timestamp_ms"] = millis();
  doc["min_free_heap"] = ESP.getFreeHeap();

  char buffer[512];
  serializeJson(doc, buffer);

  bool ok = tbMqtt.publish(TOPIC_TELEMETRY, buffer);

  Serial.print("[TB] Telemetry publish: ");
  Serial.println(ok ? "OK" : "FAIL");
}

// ============================================================
// CONTROL TASMOTA PLUG
// ============================================================
void controlTasmotaPlug(bool on) {
  if (!localMqtt.connected()) {
    connectLocalMQTT();
  }

  const char* payload = on ? "ON" : "OFF";

  bool ok = localMqtt.publish(TASMOTA_TOPIC, payload, true);

  Serial.printf("[TASMOTA] Publish %s to %s → %s\n",
                payload, TASMOTA_TOPIC, ok ? "OK" : "FAIL");
}

// ============================================================
// READ SOIL MOISTURE
// ============================================================
float readMoisturePercent(int* rawOut) {
  int raw = analogRead(MOISTURE_PIN);
  *rawOut = raw;

  raw = constrain(raw, MOISTURE_RAW_WET, MOISTURE_RAW_DRY);

  float pct = map(raw, MOISTURE_RAW_DRY, MOISTURE_RAW_WET, 0, 100);
  pct = constrain(pct, 0.0f, 100.0f);

  return pct;
}

// ============================================================
// WIFI CONNECTION
// ============================================================
void connectWiFi() {
  Serial.print("[WiFi] Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("[WiFi] Connected");
  Serial.print("[WiFi] ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

// ============================================================
// CONNECT TO THINGSBOARD
// ============================================================
void reconnectThingsBoard() {
  while (!tbMqtt.connected()) {
    Serial.print("[TB] Connecting to ThingsBoard... ");

    if (tbMqtt.connect("ESP32_GreenRoof_Logger", TB_TOKEN, nullptr)) {
      Serial.println("connected");

      tbMqtt.subscribe(TOPIC_RPC_SUB);
      Serial.println("[TB] Subscribed to RPC topic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(tbMqtt.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

// ============================================================
// CONNECT TO LOCAL MQTT BROKER
// ============================================================
void connectLocalMQTT() {
  if (localMqtt.connected()) return;

  Serial.print("[LocalMQTT] Connecting to broker... ");

  if (localMqtt.connect("ESP32_Tasmota_Controller")) {
    Serial.println("connected");
  } else {
    Serial.print("failed, rc=");
    Serial.println(localMqtt.state());
  }
}

// ============================================================
// EXTRACT RPC REQUEST ID
// ============================================================
String extractRequestId(const char* topic) {
  String t(topic);
  int lastSlash = t.lastIndexOf('/');
  return lastSlash >= 0 ? t.substring(lastSlash + 1) : "0";
}