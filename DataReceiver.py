import socket
from datetime import datetime

# === EDIT THESE ===
ESP32_IP = "172.20.10.5"  # Replace with your ESP32 IP from Serial Monitor
PORT = 80              # Must match your ESP32's server port

# === CONNECT TO ESP32 ===
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    print(f"Connecting to {ESP32_IP}:{PORT} ...")
    s.connect((ESP32_IP, PORT))
    print("Connected! Logging data...\n")

    # Timestamp
    timestamp = datetime.now().strftime("%Y.%m.%dT%H.%M.%S")

    filename = "data_logger" + timestamp + ".txt"

    with open(filename, "a") as f: # timestamped filename

        # Header
        f.write("timestamp,packetNumber,rssi,roll,pitch,yaw,altitude,pressure,temperature\n")
        # Uncomment above if force writing header, otherwise let device transmit header

        while True:
            timestamp = datetime.now().strftime("%H.%M.%S")
            data = s.recv(1024)  # receive up to 1024 bytes
            if not data:
                break
            decoded = data.decode().strip()


            if "," in decoded: #making sure its not a blank line

                lines = decoded.split("\n")

                for line in lines:
                    if line[0] == "b":
                        log_line = f"{timestamp},{line[1:]}"
                        f.write(log_line + "\n")

            f.flush()