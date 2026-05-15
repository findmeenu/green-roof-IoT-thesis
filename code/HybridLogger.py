import serial
import csv
import os
from datetime import datetime

SERIAL_PORT = "COM5"
BAUD_RATE = 115200

OUTPUT_FOLDER = r"C:\Users\prash\thesis_logs"

# Create folder if it doesn't exist
os.makedirs(OUTPUT_FOLDER, exist_ok=True)

# Full file path
output_file = os.path.join(
    OUTPUT_FOLDER,
    f"hybrid_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
)

header = [
    "architecture",
    "cycle_id",
    "rawVal",
    "moisturePct",
    "end_to_end_latency_ms",
    "mqtt_messages_count",
    "reading_received",
    "command_ack_received",
    "data_loss",
    "command_loss",
    "decision",
    "cloud_outage",
    "min_free_heap"
]

with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser, open(output_file, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(header)

    print(f" Logging to: {output_file}")
    print("Press CTRL+C to stop.")

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()

            if not line:
                continue

            if line.startswith("hybrid,"):
                parts = line.split(",")

                if len(parts) == 13:
                    writer.writerow(parts)
                    f.flush()
                    print(line)

    except KeyboardInterrupt:
        print("\n Stopped logging.")
        print(f" Saved file: {output_file}")