#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>   // Install: Arduino Library Manager → "ArduinoJson" by Benoit Blanchon
#define TEST_MODE false  // run scenario tests
// #define TEST_MODE false  // run real experiment
// ============================================================
//  GreenRoofMonitor_CLOUD.ino
//  CLOUD ARCHITECTURE ONLY
//
//  ESP32 reads soil moisture
//  ESP32 makes local rule-based decision
//  ESP32 controls pump via Tasmota MQTT and/or relay
//  ESP32 prints clean CSV rows to Serial
// ============================================================

// ─────────────────────────────────────────────
// USER CONFIGURATION
// ─────────────────────────────────────────────
const char* WIFI_SSID = "";
const char* WIFI_PASSWORD = "";

// // Local MQTT broker for Tasmota
const char* LOCAL_BROKER = "";
const int LOCAL_BROKER_PORT = 1883;


// ----------------------
// For Personal hotspot setup
// --------------------------------------------
// const char* WIFI_SSID       = "Meenu Gupta";
// const char* WIFI_PASSWORD   = "d0wckk4bdnzig";

// Local MQTT broker for Tasmota
// At cmd C:\Users\prash>arp -a; result: Interface: 172.20.10.5 --- 0x2. so this local broker is correct.
// const char* LOCAL_BROKER       = "172.20.10.5";


// Tasmota topic
const char* TASMOTA_TOPIC = "cmnd/plug1/POWER";

// ThingsBoard MQTT broker
const char* TB_BROKER = "eu.thingsboard.cloud";  // change if using local ThingsBoard
const int TB_PORT = 1883;
// const char* TB_TOKEN = "EeJJUtIBncsHLNC4wAgj";  // Thingsboard token for ESP32

const char* TB_TOKEN = "2pBKVCSoevYGxRwPpuUV"; //Home device esp32



// ─────────────────────────────────────────────
// PIN DEFINITIONS
// ─────────────────────────────────────────────
const int MOISTURE_PIN = 4;
const int RELAY_PIN = 26;

// ─────────────────────────────────────────────
// ACTUATOR SELECTION
// ─────────────────────────────────────────────
const bool USE_RELAY = false;
const bool USE_TASMOTA = true;

// ─────────────────────────────────────────────
// CONTROL THRESHOLDS
// ─────────────────────────────────────────────
const float DRY_THRESHOLD = 40.0;  // below this → pump ON
const float WET_THRESHOLD = 60.0;  // above this → pump OFF

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

// MQTT client for ThingsBoard
WiFiClient tbWifiClient;
PubSubClient tbMqtt(tbWifiClient);

unsigned long lastTelemetryMs = 0;
unsigned long cycle_id = 0;

bool pumpState = false;
// Added below variable for cloud.
bool cloudCommandReceived = false;
bool cloudDesiredPumpState = false;


// --------------------------------------
// variable for manual override button
// --------------------------------------
bool manualOverrideActive = false;
unsigned long manualOverrideUntilMs = 0;
const unsigned long MANUAL_OVERRIDE_MS = 30000;

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

void connectThingsBoard();                                                  // For Thingsboard
void thingsBoardCallback(char* topic, byte* payload, unsigned int length);  // For Thingsboard


// void runEdgeCycle();

void runCloudCycle();  // For Thingsboard

float readMoisturePercent(int* rawOut);
bool setPumpEdge(bool on, int* mqtt_messages_count);

bool publishTelemetryToThingsBoard(int rawVal, float moisturePct, bool pumpState, int* mqtt_messages_count);  // For Thingsboard

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
  bool tb_connected,  // ADD THIS
  uint32_t min_free_heap);

// ============================================================
// SETUP
// Initializes the system: starts Serial logging, sets up pins, connects WiFi and MQTT so ESP32 is ready to operate.
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);


  Serial.println();
  Serial.println("=== GreenRoofMonitor CLOUD starting ===");

  pinMode(MOISTURE_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, LOW);

  printCsvHeader();

  if (TEST_MODE) {
    runAllTests();
    return;   // stop setup here
  }

  connectWiFi();

  localMqtt.setServer(LOCAL_BROKER, LOCAL_BROKER_PORT);
  connectLocalMQTT();

  // For Thingsboard next 3 line of code
  tbMqtt.setServer(TB_BROKER, TB_PORT);
  tbMqtt.setCallback(thingsBoardCallback);
  connectThingsBoard();

  Serial.println("=== Setup complete ===");

}

// ============================================================
// LOOP
// Every few seconds → runs one full control cycle (read → decide → act → log)
// ============================================================
void loop() {
  if (TEST_MODE) {
    return;
  }
  static unsigned long lastWifiWarnMs = 0;
  
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWifiWarnMs > 5000) {
      lastWifiWarnMs = millis();
      Serial.println("[WiFi] Lost connection. Reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      printCsvRow(cycle_id + 1, 0, 0.0, 0, 0, 0, -1, 1, -1, "NO_WIFI", false, ESP.getMinFreeHeap());
    }
    return;
}
  if (!localMqtt.connected()) {
    connectLocalMQTT();
  }

  if (!tbMqtt.connected()) {
    connectThingsBoard();
  }

  localMqtt.loop();
  tbMqtt.loop();  //For TB

  unsigned long now = millis();

  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    runCloudCycle();
  }
}


// == == == == == == == == == == == == == == == == == == == == == == == == == == == == == ==
  // CLOUD CONTROL CYCLE
  // 1. Start cycle
  // 2. Read soil sensor
  // 3. ESP32 sends sensor data to ThingsBoard
  // 4. Convert raw value to moisture % at ThingsBoard
  // 5. ThingsBoard Decide pump ON/OFF
  // 6. ThingsBoard sends command/RPC back to ESP32
  // 7. ESP32  command to Tasmota/relay
  // 8. Calculate latency
  // 9. Print one CSV row
  // ============================================================
void runCloudCycle() {
  cycle_id++;

  //Cycle starts here to calculate end to end latency
  unsigned long cycleStart = millis(); //millis() = current time right now. measured as milliseconds since ESP32 started (boot/reset)

  // cloudCommandReceived = false;   // clear old command

  int mqtt_messages_count = 0;
  int reading_received = 1;
  //   command_ack_received = 1 → command was successfully sent to Tasmota (MQTT publish OK)
  //   command_ack_received = 0 → command failed to send (MQTT publish failed)
// //   command_ack_received = -1 → when soil sensor reading is not detected
  int command_ack_received = -1;
  int data_loss = 0;
  int command_loss = -1;

  String decision = pumpState ? "CLOUD_KEEP_ON" : "CLOUD_KEEP_OFF";

    // 1. Read sensor
  int rawVal = 0;
  float moisturePct = readMoisturePercent(&rawVal);

  // Basic sensor validity check
// For ESP32, analogRead is normally 0–4095.
// If your board gives larger values, adjust this condition.
  if (rawVal < 2001 || rawVal > 7000) {
    reading_received = 0;
    data_loss = 0;   //why here....data loss = 0.......as it is sensor not working properly so measurement loss
    decision = "NO_ACTION";
    }
    // cloudCommandReceived = false;
  if (reading_received == 1) {
    bool telemetryOk = publishTelemetryToThingsBoard(
      rawVal,
      moisturePct,
      pumpState,
      &mqtt_messages_count);
    // data loss is only applicable in cloud when telemetry data unable to reach to cloud.
     bool tb_connected = tbMqtt.connected();  // ADD HERE
     if (!telemetryOk || !tb_connected) {     // CHANGE THIS
      data_loss = 1;
    }

  

      unsigned long waitStart = millis();
      const unsigned long CLOUD_COMMAND_TIMEOUT_MS = 3000;

      // Wait up to 3 seconds for cloud to reply. If not, move on.
      while (!cloudCommandReceived && millis() - waitStart < CLOUD_COMMAND_TIMEOUT_MS) {
        tbMqtt.loop();
        delay(1);
      }

    if (cloudCommandReceived) {
        bool newPumpState = cloudDesiredPumpState;

        

        if (newPumpState != pumpState) {
          decision = newPumpState ? "CLOUD_PUMP_ON" : "CLOUD_PUMP_OFF";
          bool commandOk = setPumpEdge(newPumpState, &mqtt_messages_count);

          // 3. Edge actuation
      // commandOk = true  → MQTT command to Tasmota was published OK
      // commandOk = false → MQTT command to Tasmota failed
      // Same for cloud.
          if (commandOk) {
            command_ack_received = 1;
            command_loss = 0;
            pumpState = newPumpState;
          } else {
            command_ack_received = 0;
            command_loss = 1;
          }
        } 
          // else {
      //     decision = pumpState ? "CLOUD_KEEP_ON" : "CLOUD_KEEP_OFF";  //in your ThingsBoard rule chain, if state change is not needed, it returns null. 
      //     // As no command sent from ThingsBoard so, comm_recd & command_loss = NA
      //     command_ack_received = -1;
      //     command_loss = -1;
      //   }
        cloudCommandReceived = false;  // IMPORTANT reset
      // } else {
      //   decision = pumpState ? "CLOUD_KEEP_ON" : "CLOUD_KEEP_OFF";
      //   // decision = "TimeOUT";
      //   command_ack_received = -1;
      //   command_loss = -1;
      // }

        } else {
      if (!tb_connected) {
        // genuine internet outage
        decision = pumpState ? "CLOUD_TIMEOUT_KEEP_ON" : "CLOUD_TIMEOUT_KEEP_OFF";
      } else {
        // connected but ThingsBoard sent nothing = no action needed
        decision = pumpState ? "CLOUD_KEEP_ON" : "CLOUD_KEEP_OFF";
      }
      command_ack_received = -1;
      command_loss = -1;
        }
  }
      


    unsigned long cycleEnd = millis();
    unsigned long end_to_end_latency_ms = cycleEnd - cycleStart;
    uint32_t min_free_heap = ESP.getMinFreeHeap();
    bool tb_connected_status = tbMqtt.connected();  // ADD THIS

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
      tb_connected_status,
      min_free_heap);
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
// PUMP CONTROL — CLOUD ONLY
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
// SENDING TELEMETRY — CLOUD ONLY
// Send sensor data to ThingsBoard
// If publish succeeds:
// data reaches ThingsBoard
// mqtt_messages_count = +1
// ============================================================

bool publishTelemetryToThingsBoard(
  int rawVal,
  float moisturePct,
  bool pumpState,
  int* mqtt_messages_count) {
  String payload = "{";
  payload += "\"raw_value\":";
  payload += rawVal;
  payload += ",";
  payload += "\"moisture_pct\":";
  payload += moisturePct;
  payload += ",";
  payload += "\"pump_state\":";
  payload += pumpState ? "true" : "false";
  payload += ",";
  payload += "\"cycle_id\":";
  payload += cycle_id;
  payload += "}";

  // Serial.print("[ThingsBoard] payload: "); //faltu
  // Serial.println(payload); //faltu

  // Send this JSON telemetry payload to ThingsBoard for the currently authenticated device token. This automatically becomes:POST_TELEMETRY
  bool ok = tbMqtt.publish("v1/devices/me/telemetry", payload.c_str());

  (*mqtt_messages_count)++;

  // Serial.printf("[ThingsBoard] telemetry → %s\n", ok ? "OK" : "FAIL"); //Debugging code

  return ok;
}

// This function automatically runs when ThingsBoard sends a command
// ThingsBoard sends: ON
// Callback receives: ON
// Sets:
// cloudCommandReceived = true
// cloudDesiredPumpState = true
// void thingsBoardCallback(char* topic, byte* payload, unsigned int length) {
//   String msg = "";

//   for (unsigned int i = 0; i < length; i++) {
//     msg += (char)payload[i];
//   }
//   Serial.print("[ThingsBoard] topic: ");
//   Serial.println(topic);        // ← ADD THIS
//   Serial.print("[ThingsBoard] RPC received: ");
//   Serial.println(msg);

//   if (msg.indexOf("ON") >= 0 || msg.indexOf("true") >= 0) {
//     cloudDesiredPumpState = true;
//     cloudCommandReceived = true;
//   } else if (msg.indexOf("OFF") >= 0 || msg.indexOf("false") >= 0) {
//     cloudDesiredPumpState = false;
//     cloudCommandReceived = true;
//   }
// }

void thingsBoardCallback(char* topic, byte* payload, unsigned int length) {
 
  String msg = "";

  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  // Serial.print("[ThingsBoard] topic: "); //debugging
  // Serial.println(topic);  //debugging
  

  // Serial.print("[ThingsBoard] RPC received: ");
  // Serial.println(msg);  //debugging

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, msg);

  if (error) {
    Serial.println("[ThingsBoard] JSON parse failed");
    return;
  }

  String method = doc["method"];
  bool requestedState = doc["params"];

  if (method == "setPump") {
    cloudDesiredPumpState = requestedState;
    cloudCommandReceived = true;

    Serial.print("[ThingsBoard] setPump received: ");
    Serial.println(requestedState ? "ON" : "OFF");
  }
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
  if (localMqtt.connected()) return;

  Serial.print("[LocalMQTT] Connecting... ");

  if (localMqtt.connect("ESP32_EDGE_CONTROLLER")) {
    Serial.println("connected.");
  } else {
    Serial.printf("failed rc=%d\n", localMqtt.state());
    

  }
}
// ============================================================
// ThingsBoard CONNECTION FUNCTION
// ESP32 logs into ThingsBoard using device token
// Then subscribes to cloud commands
// ============================================================
void connectThingsBoard() {
  if (tbMqtt.connected()) return;

  Serial.print("[ThingsBoard] Connecting... ");

  if (tbMqtt.connect("ESP32_CLOUD_CONTROLLER", TB_TOKEN, NULL)) {
    Serial.println("connected.");
    tbMqtt.subscribe("v1/devices/me/rpc/request/+");
  //   bool subOk = tbMqtt.subscribe("v1/devices/me/rpc/request/+"); //For checking
  //   Serial.printf("[ThingsBoard] RPC subscribe → %s\n", subOk ? "OK" : "FAIL"); //For Checking
  } else {
    Serial.printf("failed rc=%d\n", tbMqtt.state());
  }
}

// ============================================================
// CSV HEADER
// ============================================================
void printCsvHeader() {
  Serial.println("architecture,cycle_id,raw_value,moisture_pct,end_to_end_latency_ms,mqtt_messages_count,reading_received,command_ack_received,data_loss,command_loss,decision,tb_connected,min_free_heap");
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
  bool tb_connected, //Added this to catch internet outage. basically connection to tb is lost
  uint32_t min_free_heap) {
  Serial.print("cloud");
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
  } else if (command_ack_received == 0) {
    Serial.print("FAILURE");
  } else {
    Serial.print("NA");
  }
  Serial.print(",");

  Serial.print(data_loss ? "YES" : "NO");
  Serial.print(",");


  if (command_loss == 1) {
    Serial.print("YES");
  } else if (command_loss == 0) {
    Serial.print("NO");
  } else {
    Serial.print("NA");
  }
  Serial.print(",");

  Serial.print(decision);
  Serial.print(",");
  Serial.print(tb_connected ? "YES" : "NO");
  Serial.print(",");  
  Serial.println(min_free_heap);
}








/// ===================================
// TEST LOGIC - CLOUD MODE ASYNC
// =========================================

CycleResult runCycleLogic(
  int reading_received,
  bool telemetryShouldSucceed,
  bool cloudCommandReceived,
  bool cloudDesiredPumpState,
  bool initialPumpState,
  bool tasmotaShouldSucceed
) {
  CycleResult r;

  r.reading_received = reading_received;
  r.data_loss = 0;
  r.mqtt_messages_count = 0;
  r.command_ack_received = -1;
  r.command_loss = -1;
  r.finalPumpState = initialPumpState;
  r.decision = "WAITING_CLOUD";

  bool pumpState = initialPumpState;

  // Sensor failed
  if (reading_received == 0) {
    r.data_loss = 0;
    r.decision = "NO_ACTION";
    return r;
  }

  // Telemetry attempt to ThingsBoard
  r.mqtt_messages_count++;

  if (!telemetryShouldSucceed) {
    r.data_loss = 1;
  }

  // Async cloud model: no command received means keep current state
  if (!cloudCommandReceived) {
    r.decision = pumpState ? "CLOUD_TIMEOUT_KEEP_ON" : "CLOUD_TIMEOUT_KEEP_OFF";
    r.command_ack_received = -1;
    r.command_loss = -1; // NA: no command arrived from cloud, so not command loss as command loss is only for tasmota
    r.finalPumpState = pumpState;
    return r;
  }

  bool newPumpState = cloudDesiredPumpState;

  // Cloud command received but same as current state
  if (newPumpState == pumpState) {
    r.decision = pumpState ? "CLOUD_KEEP_ON" : "CLOUD_KEEP_OFF";
    r.command_ack_received = -1;
    r.command_loss = -1;
    r.finalPumpState = pumpState;
    return r;
  }

  // Cloud command requires pump state change
  r.decision = newPumpState ? "CLOUD_PUMP_ON" : "CLOUD_PUMP_OFF";
  r.mqtt_messages_count++;

  if (tasmotaShouldSucceed) {
    r.command_ack_received = 1;
    r.command_loss = 0;
    r.finalPumpState = newPumpState;
  } else {
    r.command_ack_received = 0;
    r.command_loss = 1;
    r.finalPumpState = pumpState;
  }

  return r;
}

void runAllTests() {
  Serial.println("=== Running CLOUD ASYNC Scenario Tests ===");

  // id, reading, telemetryOK, cloudCommandReceived, cloudWantsPump,
  // initialPump, tasmotaOK,
  // expected data_loss, mqtt_count, ack, cmd_loss, finalPump, decision

  testCase(1, 0, true,  false, false, false, true,  0, 0, -1, -1, false, "NO_ACTION");

  // Telemetry fails, no cloud command received
  testCase(2, 1, false, false, false, false, true,  1, 1, -1, -1, false, "CLOUD_TIMEOUT_KEEP_OFF");

  // Telemetry OK, no cloud command received
  testCase(3, 1, true,  false, false, false, true,  0, 1, -1, -1, false, "CLOUD_TIMEOUT_KEEP_OFF");

  // Cloud says ON, pump was OFF, Tasmota succeeds
  testCase(4, 1, true,  true,  true,  false, true,  0, 2, 1, 0, true,  "CLOUD_PUMP_ON");

  // Cloud says ON, pump was OFF, Tasmota fails
  testCase(5, 1, true,  true,  true,  false, false, 0, 2, 0, 1, false, "CLOUD_PUMP_ON");

  // Cloud says OFF, pump was ON, Tasmota succeeds
  testCase(6, 1, true,  true,  false, true,  true,  0, 2, 1, 0, false, "CLOUD_PUMP_OFF");

  // Cloud says OFF, pump was ON, Tasmota fails
  testCase(7, 1, true,  true,  false, true,  false, 0, 2, 0, 1, true,  "CLOUD_PUMP_OFF");

  // Cloud says OFF, pump already OFF
  testCase(8, 1, true,  true,  false, false, true,  0, 1, -1, -1, false, "CLOUD_KEEP_OFF");

  // Cloud says ON, pump already ON
  testCase(9, 1, true,  true,  true,  true,  true,  0, 1, -1, -1, true,  "CLOUD_KEEP_ON");

  // No cloud command, pump already ON
  testCase(10, 1, true, false, false, true, true, 0, 1, -1, -1, true, "CLOUD_TIMEOUT_KEEP_ON");

  Serial.println("=== Tests Finished ===");
}

void testCase(
  int id,
  int reading_received,
  bool telemetryShouldSucceed,
  bool cloudShouldReply,
  bool cloudDesiredPumpState,
  bool initialPumpState,
  bool tasmotaShouldSucceed,
  int expected_data_loss,
  int expected_mqtt_count,
  int expected_command_ack,
  int expected_command_loss,
  bool expected_finalPumpState,
  String expected_decision
) {
  CycleResult r = runCycleLogic(
    reading_received,
    telemetryShouldSucceed,
    cloudShouldReply,
    cloudDesiredPumpState,
    initialPumpState,
    tasmotaShouldSucceed
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
  Serial.print("reading_received        = ");
  Serial.println(reading_received ? "RECEIVED" : "FAILED");

  Serial.print("telemetryShouldSucceed  = ");
  Serial.println(telemetryShouldSucceed ? "YES" : "NO");

  Serial.print("cloudShouldReply        = ");
  Serial.println(cloudShouldReply ? "YES" : "NO");

  Serial.print("cloudDesiredPumpState   = ");
  Serial.println(cloudDesiredPumpState ? "ON" : "OFF");

  Serial.print("initialPumpState        = ");
  Serial.println(initialPumpState ? "ON" : "OFF");

  Serial.print("tasmotaShouldSucceed    = ");
  Serial.println(tasmotaShouldSucceed ? "YES" : "NO");

  Serial.println();

  Serial.println("RESULTS:");
  Serial.print("decision                = ");
  Serial.println(r.decision);

  Serial.print("data_loss               = ");
  Serial.println(r.data_loss ? "YES" : "NO");

  Serial.print("mqtt_messages_count     = ");
  Serial.println(r.mqtt_messages_count);

  Serial.print("command_ack_received    = ");
  if (r.command_ack_received == 1) Serial.println("SUCCESS");
  else if (r.command_ack_received == 0) Serial.println("FAILURE");
  else Serial.println("NA");

  Serial.print("command_loss            = ");
  if (r.command_loss == 1) Serial.println("YES");
  else if (r.command_loss == 0) Serial.println("NO");
  else Serial.println("NA");

  Serial.print("finalPumpState          = ");
  Serial.println(r.finalPumpState ? "ON" : "OFF");

  Serial.println("--------------------------------------------------");
}