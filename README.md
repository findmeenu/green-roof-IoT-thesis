# green-roof-IoT-thesis
Design and Evaluation of an Edge–Cloud-Hybrid IoT Architecture for Smart Green Roof 


Overview
This repository contains the firmware and supporting code for a small-scale IoT-based green roof monitoring and irrigation control system. The system uses a capacitive soil moisture sensor and an ESP32-S2 microcontroller to monitor soil conditions and automate irrigation control through a Tasmota-controlled smart plug.
The project evaluates and compares three deployment architectures — Edge, Cloud, and Hybrid — to assess trade-offs in latency, reliability, communication overhead, and resource usage for IoT-based environmental monitoring systems.

Research Questions

RQ1: Can an open-source cloud platform be effectively integrated into an IoT-based green roof monitoring and irrigation control system?
RQ2: How do edge, cloud, and hybrid deployment architectures compare in terms of responsiveness, reliability, and resource usage for green roof irrigation control?


System Architecture
Hardware Components
ComponentSpecificationMicrocontrollerESP32-S2 Dev ModuleSoil Moisture SensorCapacitive, analog output, 3.3–5VSmart PlugTasmota-flashed Wi-Fi plugIrrigation Pump220–240V ACWater ReservoirSmall-scale laboratory tank
Software & Protocols
ComponentDetailsFirmwareArduino (C++) via Arduino IDECommunicationMQTTLocal MQTT BrokerEclipse Mosquitto v2.1.2Cloud PlatformThingsBoard Community Edition (CE)Data LoggingPython-based logging script

Deployment Architectures
Edge Architecture
Irrigation decisions are made locally on the ESP32 using predefined soil moisture thresholds. The ESP32 directly publishes MQTT commands to the Tasmota smart plug.
Soil Sensor → ESP32 (decision) → MQTT → Tasmota Plug → Pump
Cloud Architecture
Sensor readings are transmitted to ThingsBoard Cloud, where the Rule Chain makes the irrigation decision and sends an RPC command back to the ESP32.
Soil Sensor → ESP32 → MQTT → ThingsBoard Rule Engine → RPC → ESP32 → Tasmota Plug → Pump
Hybrid Architecture
Irrigation decisions are made locally on the ESP32, while aggregated data is transmitted to ThingsBoard for remote monitoring. During internet outages, data is buffered in ESP32 flash memory and synced when connectivity is restored.
Soil Sensor → ESP32 (decision) → MQTT → Tasmota Plug → Pump
                     ↓
              ThingsBoard (monitoring + visualization)

Irrigation Control Logic
IF moisture < 40%  → Activate pump (ON)
IF moisture > 60%  → Deactivate pump (OFF)
IF 40% ≤ moisture ≤ 60% → Keep current pump state
Control commands are only sent when a state change is required, reducing unnecessary MQTT communication overhead.

├── code/
│   ├── GreenRoofEdge.ino                  # ESP32 firmware — Edge architecture
│   ├── GreenRoofCloud.ino                 # ESP32 firmware — Cloud architecture
│   ├── GreenRoofHybrid.ino                # ESP32 firmware — Hybrid architecture
│   ├── GreenRoofMonitorTbIntegration.ino  # ThingsBoard monitoring integration
│   ├── EdgeLogger.py                      # Python metric logger — Edge
│   ├── CloudLogger.py                     # Python metric logger — Cloud
│   └── HybridLogger.py                    # Python metric logger — Hybrid
├── experiment_data/
│   ├── edge_log_20260512_111448.csv        # Edge experiment run 1
│   ├── edge_log_20260512_112535.csv        # Edge experiment run 2
│   ├── edge_log_20260512_114403.csv        # Edge experiment run 3
│   ├── cloud_log_20260514_002323.csv       # Cloud experiment run 1
│   ├── cloud_log_20260514_004743.csv       # Cloud experiment run 2
│   ├── cloud_log_20260514_005847.csv       # Cloud experiment run 3
│   ├── hybrid_log_20260514_122630.csv      # Hybrid experiment run 1
│   ├── hybrid_log_20260514_124010.csv      # Hybrid experiment run 2
│   ├── hybrid_log_20260514_125505.csv      # Hybrid experiment run 3
│   └── logs.txt                            # Raw system logs
├── LICENSE
└── README.md

Key Results:-

<img width="796" height="465" alt="image" src="https://github.com/user-attachments/assets/6aeb7793-f42a-496e-a1ad-184a48c3b5af" />
<img width="1246" height="314" alt="image" src="https://github.com/user-attachments/assets/b8fbeac7-398e-45e8-9c4a-b2cb805053b1" />

<img width="1437" height="415" alt="image" src="https://github.com/user-attachments/assets/946a0475-8920-46e0-b3a1-1652cb8158e7" />
<img width="1238" height="1010" alt="image" src="https://github.com/user-attachments/assets/a148239b-2217-4c3f-8d04-86b6b17fc3e7" />
<img width="1519" height="315" alt="image" src="https://github.com/user-attachments/assets/5e11c78c-f167-4a67-bbb9-efe2cb7b765a" />



Getting Started
Prerequisites

Arduino IDE with ESP32-S2 board support
Libraries: PubSubClient, ArduinoJson, WiFi
Eclipse Mosquitto MQTT broker installed on local machine
ThingsBoard CE account (free tier at eu.thingsboard.cloud)
Tasmota-flashed smart plug configured with MQTT topic cmnd/plug1/POWER

Configuration
In the firmware sketch, update the following constants before uploading:

cpp// Wi-Fi
const char* WIFI_SSID     = "your_network_name";
const char* WIFI_PASSWORD = "your_password";


// ThingsBoard (cloud/hybrid only)
const char* TB_SERVER = "eu.thingsboard.cloud";
const char* TB_TOKEN  = "your_device_access_token";

// Local MQTT Broker (your laptop IP)
const char* LOCAL_BROKER = "192.168.x.xx";
ThingsBoard Rule Chain (Cloud Architecture)
Import thingsboard/rule_chain_export.json into your ThingsBoard Rule Chains. Ensure the Message Type Switch node routes RPC Request to Device messages to the RPC Call Request node targeting your ESP32 device.

Experiment Setup

Each architecture tested under identical hardware, sensor, Wi-Fi, and threshold conditions
Each scenario run for 8 minutes, repeated 3 times per architecture
3-minute internet outage scenarios introduced to test resilience
Metrics logged per control cycle using Python logging script


License
This project is developed for academic research purposes.

Author
Meenu — IoT Systems Thesis, 2026
