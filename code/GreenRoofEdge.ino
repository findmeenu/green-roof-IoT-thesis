#include <WiFi.h>
#include <PubSubClient.h>
#define TEST_MODE false   // run scenario tests
// #define TEST_MODE false  // run real experiment
// ============================================================
//  GreenRoofMonitor_EDGE.ino
//  EDGE ARCHITECTURE ONLY
//
//  ESP32 reads soil moisture
//  ESP32 makes local rule-based decision
//  ESP32 controls pump via Tasmota MQTT and/or relay
//  ESP32 prints clean CSV rows to Serial
// ============================================================

// ─────────────────────────────────────────────
// USER CONFIGURATION
// ─────────────────────────────────────────────
const char* WIFI_SSID       = "";
const char* WIFI_PASSWORD   = "";
// Local MQTT broker for Tasmota i.e. your laptop IP
const char* LOCAL_BROKER       = "";


// ----------------------
// For Personal hotspot setup
// // --------------------------------------------
// const char* WIFI_SSID       = "Meenu Gupta";
// const char* WIFI_PASSWORD   = "d0wckk4bdnzig";

// // Local MQTT broker for Tasmota
// // At cmd C:\Users\prash>arp -a; result: Interface: 172.20.10.5 --- 0x2. so this local broker is correct.
// const char* LOCAL_BROKER       = "172.20.10.5";



const int   LOCAL_BROKER_PORT  = 1883;

// Tasmota topic
const char* TASMOTA_TOPIC      = "cmnd/plug1/POWER";

// ─────────────────────────────────────────────
// PIN DEFINITIONS
// ─────────────────────────────────────────────
const int MOISTURE_PIN = 4;
const int RELAY_PIN    = 26;

// ─────────────────────────────────────────────
// ACTUATOR SELECTION
// ─────────────────────────────────────────────
const bool USE_RELAY   = false;
const bool USE_TASMOTA = true;

// ─────────────────────────────────────────────
// CONTROL THRESHOLDS
// ─────────────────────────────────────────────
const float DRY_THRESHOLD = 40.0;   // below this → pump ON
const float WET_THRESHOLD = 60.0;   // above this → pump OFF

// ─────────────────────────────────────────────
// SENSOR CALIBRATION
// Change these after testing your sensor
// ─────────────────────────────────────────────
const int MOISTURE_RAW_DRY = 6500;
const int MOISTURE_RAW_WET = 2150;

// ─────────────────────────────────────────────
// TIMING
// ─────────────────────────────────────────────
const unsigned long TELEMETRY_INTERVAL_MS = 5000;

// ─────────────────────────────────────────────
// GLOBAL STATE
// ─────────────────────────────────────────────
WiFiClient localWifiClient;
PubSubClient localMqtt(localWifiClient);

unsigned long lastTelemetryMs = 0;
unsigned long cycle_id = 0;

bool pumpState = false;

// --------------------------------------
// struct for testing
// --------------------------------------
struct CycleResult {
  int reading_received;
  int data_loss;
  int mqtt_messages_count;
  int command_ack_received;
  int command_loss;
  bool finalPumpState;
  String decision;
};

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void connectWiFi();
void connectLocalMQTT();
void runEdgeCycle();
float readMoisturePercent(int* rawOut);
bool setPumpEdge(bool on, int* mqtt_messages_count);
void printCsvHeader();
void printCsvRow(
  unsigned long cycle_id,
  int rawVal,
  float moisturePct,
  unsigned long end_to_end_latency_ms,
  int mqtt_messages_count,
  int reading_received,
  int command_ack_received,
  int data_loss,
  int command_loss,
  String decision,
  uint32_t min_free_heap
);

// ============================================================
// SETUP
// Initializes the system: starts Serial logging, sets up pins, connects WiFi and MQTT so ESP32 is ready to operate.
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== GreenRoofMonitor EDGE starting ===");

  pinMode(MOISTURE_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, LOW);

  connectWiFi();

  localMqtt.setServer(LOCAL_BROKER, LOCAL_BROKER_PORT);
  connectLocalMQTT();

  Serial.println("=== Setup complete ===");

  printCsvHeader();

  if (TEST_MODE) {
  runAllTests();
}
}

// ============================================================
// LOOP
// Every few seconds → runs one full control cycle (read → decide → act → log)
// ============================================================
void loop() {
  if (TEST_MODE) {
  return;
}
  if (!localMqtt.connected()) {
    connectLocalMQTT();
  }

  localMqtt.loop();

  unsigned long now = millis();

  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    runEdgeCycle();
  }
}

// ============================================================
// EDGE CONTROL CYCLE
// 1. Start cycle
// 2. Read soil sensor
// 3. Convert raw value to moisture %
// 4. Decide pump ON/OFF
// 5. Send command to Tasmota/relay
// 6. Calculate latency
// 7. Print one CSV row
// ============================================================
  void runEdgeCycle() {
    cycle_id++;

    //Cycle starts here to calculate end to end latency
    unsigned long cycleStart = millis(); //millis() = current time right now. measured as milliseconds since ESP32 started (boot/reset)

    int mqtt_messages_count = 0;

    int reading_received = 1;
  //   command_ack_received = 1 → command was successfully sent to Tasmota (MQTT publish OK)
  //   command_ack_received = 0 → command failed to send (MQTT publish failed)
  //   command_ack_received = -1 → when soil sensor reading is not detected
    int command_ack_received = -1;

    int data_loss = 0;
    int command_loss = -1;  //command_loss = did pump command fail? Problem Tasmota does not reply / MQTT publish fails. So command_loss = 1, command_ack_received = 0 & pump state does not change

    String decision = "NO_ACTION";

    // 1. Read sensor
    int rawVal = 0;
    float moisturePct = readMoisturePercent(&rawVal);

    // Basic sensor validity check
    // For ESP32, analogRead is normally 0–4095.
    // If your board gives larger values, adjust this condition.
    // if reading recd. and data loss 0 0 then sensor not working.
    if (rawVal < 2001 || rawVal > 6700) { 
      reading_received = 0;   //means sensor failed.
      data_loss = 0;  //why here....data loss.......as it is sensor not working properly so measurement loss. So, in edge no data loss does not make sense as we are not sending telemetry data to anywhere.
      decision = "NO_ACTION";
    }

    // 2. Edge decision
    bool newPumpState = pumpState;

    if (reading_received == 1) {
      if (moisturePct < DRY_THRESHOLD) {
        newPumpState = true;
        decision = "PUMP_ON";
      }
      else if (moisturePct > WET_THRESHOLD) {
        newPumpState = false;
        decision = "PUMP_OFF";
      }
      // If current desired state == previous actual state→ no need to send command to Tasmota
      else {
        newPumpState = pumpState;
        decision = pumpState ? "KEEP_ON" : "KEEP_OFF";
      }

      // 3. Edge actuation
      // commandOk = true  → MQTT command to Tasmota was published OK
      // commandOk = false → MQTT command to Tasmota failed
    if (newPumpState != pumpState) {
        bool commandOk = setPumpEdge(newPumpState, &mqtt_messages_count);

      if (!commandOk) {
        command_ack_received = 0;
        command_loss = 1;
    }

     if (commandOk) {
        command_ack_received = 1;
        command_loss = 0;
        pumpState = newPumpState;
    }
  }
  else {
    command_ack_received = -1;
    command_loss = -1;
    decision = pumpState ? "KEEP_ON" : "KEEP_OFF";  // ADD THIS
    
    }
  }
    unsigned long cycleEnd = millis();

    unsigned long end_to_end_latency_ms = cycleEnd - cycleStart;
    uint32_t min_free_heap = ESP.getMinFreeHeap();

    // 4. Print clean CSV row
    printCsvRow(
      cycle_id,
      rawVal,
      moisturePct,
      end_to_end_latency_ms,
      mqtt_messages_count,
      reading_received,
      command_ack_received,
      data_loss,
      command_loss,
      decision,
      min_free_heap
    );
}

// ============================================================
// SENSOR READING
// Maps raw ADC value to 0–100% moisture
// ============================================================
float readMoisturePercent(int* rawOut) {
  int raw = analogRead(MOISTURE_PIN);
  *rawOut = raw;

  int constrainedRaw = constrain(raw, MOISTURE_RAW_WET, MOISTURE_RAW_DRY);

  float pct = map(constrainedRaw, MOISTURE_RAW_DRY, MOISTURE_RAW_WET, 0, 100);
  pct = constrain(pct, 0.0f, 100.0f);

  return pct;
}

// ============================================================
// PUMP CONTROL — EDGE ONLY
// Controls relay and/or Tasmota plug
// Returns true if command was sent successfully
// commandOk = true  → MQTT command to Tasmota was published OK
// commandOk = false → MQTT command to Tasmota failed
// ============================================================
bool setPumpEdge(bool on, int* mqtt_messages_count) {
  bool success = true;

  if (USE_RELAY) {
    digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  }

  if (USE_TASMOTA) {
    if (!localMqtt.connected()) {
      connectLocalMQTT();
    }

    const char* payload = on ? "ON" : "OFF";

    bool ok = localMqtt.publish(TASMOTA_TOPIC, payload, true);

    (*mqtt_messages_count)++;

    Serial.printf("[TASMOTA] publish %s → %s\n", payload, ok ? "OK" : "FAIL");

    if (!ok) {
      success = false;
    }
  }

  return success;
}

// ============================================================
// WIFI CONNECTION
// ============================================================
void connectWiFi() {
  Serial.print("[WiFi] Connecting to ");
  Serial.print(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("[WiFi] Connected, IP: ");
  Serial.println(WiFi.localIP());
}

// ============================================================
// LOCAL MQTT CONNECTION
// ============================================================
void connectLocalMQTT() {
  while (!localMqtt.connected()) {
    Serial.print("[LocalMQTT] Connecting... ");

    if (localMqtt.connect("ESP32_EDGE_CONTROLLER")) {
      Serial.println("connected.");
    }
    else {
      Serial.printf("failed rc=%d, retrying...\n", localMqtt.state());
      delay(3000);
    }
  }
}

// ============================================================
// CSV HEADER
// ============================================================
void printCsvHeader() {
  Serial.println("architecture,cycle_id,raw_value,moisture_pct,end_to_end_latency_ms,mqtt_messages_count,reading_received,command_ack_received,data_loss,command_loss,decision,min_free_heap");
}

// ============================================================
// CSV ROW
// ============================================================
void printCsvRow(
  unsigned long cycle_id,
  int rawVal,
  float moisturePct,
  unsigned long end_to_end_latency_ms,
  int mqtt_messages_count,
  int reading_received,
  int command_ack_received,
  int data_loss,
  int command_loss,
  String decision,
  uint32_t min_free_heap
) {
  Serial.print("edge");
  Serial.print(",");
  Serial.print(cycle_id);
  Serial.print(",");
  Serial.print(rawVal);
  Serial.print(",");
  Serial.print(moisturePct, 2);
  Serial.print(",");
  Serial.print(end_to_end_latency_ms);
  Serial.print(",");
  Serial.print(mqtt_messages_count);
  Serial.print(",");
  Serial.print(reading_received ? "RECEIVED" : "FAILED");
  Serial.print(",");

  if (command_ack_received == 1) {
  Serial.print("SUCCESS");
  }
  else if (command_ack_received == 0) {
    Serial.print("FAILURE");
  }
  else {
    Serial.print("NA");
  }
  Serial.print(",");

  Serial.print(data_loss ? "YES" : "NO");
  Serial.print(",");


  if (command_loss == 1) {
  Serial.print("YES");
  }
  else if (command_loss == 0) {
    Serial.print("NO");
  }
  else {
    Serial.print("NA");
  }
  Serial.print(",");
  
  Serial.print(decision);
  Serial.print(",");
  Serial.println(min_free_heap);
}








// ===================================
// TEST LOGIC
// =========================================
CycleResult runCycleLogic(
  int reading_received,
  int rawVal,
  float moisturePct,
  bool initialPumpState,
  bool mqttShouldSucceed
) {
  CycleResult r;

  r.reading_received = reading_received;
  r.data_loss = 0;
  r.mqtt_messages_count = 0;
  r.command_ack_received = -1;
  r.command_loss = -1;
  r.finalPumpState = initialPumpState;
  r.decision = "NO_ACTION";

  bool pumpState = initialPumpState;
  bool newPumpState = pumpState;

  if (reading_received == 0) {
    r.data_loss = 0;          // sensor failed, not architecture data loss
    r.command_ack_received = -1;
    r.command_loss = -1;
    r.decision = "NO_ACTION";
    return r;
  }

  if (moisturePct < DRY_THRESHOLD) {
    newPumpState = true;
    r.decision = "PUMP_ON";
  }
  else if (moisturePct > WET_THRESHOLD) {
    newPumpState = false;
    r.decision = "PUMP_OFF";
  }
  else {
    newPumpState = pumpState;
    r.decision = pumpState ? "KEEP_ON" : "KEEP_OFF";
  }

  if (newPumpState != pumpState) {
    r.mqtt_messages_count = 1;

    if (mqttShouldSucceed) {
      r.command_ack_received = 1;
      r.command_loss = 0;
      r.finalPumpState = newPumpState;
    }
    else {
      r.command_ack_received = 0;
      r.command_loss = 1;
      r.finalPumpState = pumpState;
    }
  }
  else {
    r.command_ack_received = -1;
    r.command_loss = -1;
    r.finalPumpState = pumpState;
  }

  return r;
}

void runAllTests() {
  Serial.println("=== Running Scenario Tests ===");

  testCase(1, 0, 0, 0, false, true, 0, 0, -1, -1, false, "NO_ACTION");
  testCase(2, 0, 0, 0, true,  true, 0, 0, -1, -1, true,  "NO_ACTION");

  testCase(3, 1, 2500, 50, false, true,  0, 0, -1, -1, false, "KEEP_OFF");
  testCase(4, 1, 2500, 50, true,  true,  0, 0, -1, -1, true,  "KEEP_ON");

  testCase(5, 1, 4000, 30, false, true,  0, 1, 1, 0, true,  "PUMP_ON");
  testCase(6, 1, 4000, 30, false, false, 0, 1, 0, 1, false, "PUMP_ON");

  testCase(7, 1, 3500,  70, true,  true,  0, 1, 1, 0, false, "PUMP_OFF");
  testCase(8, 1, 3500,  70, true,  false, 0, 1, 0, 1, true,  "PUMP_OFF");  

  Serial.println("=== Tests Finished ===");
}

void testCase(  
  int id,
  int reading_received,
  int rawVal,
  float moisturePct,
  bool initialPumpState,
  bool mqttShouldSucceed,
  int expected_data_loss,
  int expected_mqtt_count,
  int expected_command_ack,
  int expected_command_loss,
  bool expected_finalPumpState,
  String expected_decision
) {
  CycleResult r = runCycleLogic(
    reading_received,
    rawVal,
    moisturePct,
    initialPumpState,
    mqttShouldSucceed
  );

  bool pass =
    r.data_loss == expected_data_loss &&
    r.mqtt_messages_count == expected_mqtt_count &&
    r.command_ack_received == expected_command_ack &&
    r.command_loss == expected_command_loss &&
    r.finalPumpState == expected_finalPumpState &&
    r.decision == expected_decision;

  Serial.println();
  Serial.println("--------------------------------------------------");
  Serial.print("TEST CASE ");
  Serial.print(id);
  Serial.print(" -> ");
  Serial.println(pass ? "PASS" : "FAIL");
  Serial.println("--------------------------------------------------");

  Serial.println("INPUTS:");
  Serial.print("reading_received      = ");
  Serial.println(reading_received == 1 ? "RECEIVED" : "FAILED");

  Serial.print("raw_value             = ");
  Serial.println(rawVal);

  Serial.print("moisturePct           = ");
  Serial.println(moisturePct);

  Serial.print("initialPumpState      = ");
  Serial.println(initialPumpState ? "ON" : "OFF");

  Serial.print("mqttShouldSucceed     = ");
  Serial.println(mqttShouldSucceed ? "YES" : "NO");

  Serial.println();

  Serial.println("RESULTS:");
  Serial.print("decision              = ");
  Serial.println(r.decision);

  Serial.print("data_loss             = ");
  Serial.println(r.data_loss == 1 ? "YES" : "NO");

  Serial.print("mqtt_messages_count   = ");
  Serial.println(r.mqtt_messages_count);

  Serial.print("command_ack_received  = ");
  if (r.command_ack_received == 1) {
    Serial.println("SUCCESS");
  }
  else if (r.command_ack_received == 0) {
    Serial.println("FAILURE");
  }
  else {
    Serial.println("NA");
  }

  Serial.print("command_loss          = ");
  if (r.command_loss == 1) {
    Serial.println("YES");
  }
  else if (r.command_loss == 0) {
    Serial.println("NO");
  }
  else {
    Serial.println("NA");
  }

  Serial.print("finalPumpState        = ");
  Serial.println(r.finalPumpState ? "ON" : "OFF");

  Serial.println("--------------------------------------------------");
}