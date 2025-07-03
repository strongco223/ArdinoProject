import socket
import struct
import time
import threading
import math

# Server 資訊
HOST = '192.168.144.30'
PORT = 1000

s = None
lock = threading.Lock()
running = True

# 地球半徑與常數
EARTH_RADIUS = 6378137  # m
STEP_DISTANCE = 1       # 每次移動 1 公尺（0.2s 以 5 m/s）
# 狀態變數

# 24.947325 121.229435 31.0000
lat = 24.947325
lon = 121.229435
alt = 31.0000
initial_set = True
direction_index = 0  # 0:E, 1:N, 2:W, 3:S
step_count = 0       # 每方向最多 500 次（一段 500m）

def connect_to_server():
    global s
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))
            print("client: connected to server")
            return
        except Exception as e:
            print(f"client: connection failed, retrying in 3s... ({e})")
            time.sleep(3)

def send_floats(lat, lon, alt):
    global s
    packet = b'\xAA' + struct.pack('<fff', lat, lon, alt)
    try:
        s.sendall(packet)
        #response = s.recv(1024).decode('utf-8', errors='ignore')
        #print(f"server: {response.strip()}")
    except Exception as e:
        print(f"client: error during send/recv: {e}")
        s.close()
        connect_to_server()

def move_one_step(lat, lon, direction_deg):
    """根據角度方向與當前位置，每次移動 STEP_DISTANCE 公尺"""
    d = STEP_DISTANCE
    rad = math.radians(direction_deg)
    dlat = (d * math.cos(rad)) / EARTH_RADIUS
    dlon = (d * math.sin(rad)) / (EARTH_RADIUS * math.cos(math.radians(lat)))

    new_lat = lat + math.degrees(dlat)
    new_lon = lon + math.degrees(dlon)
    return new_lat, new_lon

def auto_send_loop():
    global lat, lon, alt, running, direction_index, step_count

    # 方向順序：東、北、西、南（以角度表示）
    directions = [90, 0, 270, 180]

    while running:
        with lock:
            if not initial_set:
                time.sleep(0.2)
                continue

            # 計算移動後的新座標
            direction = directions[direction_index]
            lat, lon = move_one_step(lat, lon, direction)
            send_floats(lat, lon, alt)
            print(f"{lat},{lon},{alt}")

            step_count += 1
            if step_count >= 500:
                direction_index = (direction_index + 1) % 4
                step_count = 0

        time.sleep(0.2)

def input_loop():
    global lat, lon, alt, initial_set, running
    while True:
        try:
            user_input = input("client: enter lat lon alt (e.g., 23.111111 121.1111 10.0): ")
            parts = user_input.strip().split()
            if len(parts) != 3:
                print("client: invalid input, expected 3 values")
                continue
            lat_f, lon_f, alt_f = map(float, parts)
            with lock:
                lat = lat_f
                lon = lon_f
                alt = alt_f
                initial_set = True
                print("client: start movement from initial position...")
        except KeyboardInterrupt:
            print("\nclient: exiting.")
            running = False
            if s:
                s.close()
            break
        except Exception as e:
            print(f"client: input error: {e}")

# 主程式
connect_to_server()
threading.Thread(target=auto_send_loop, daemon=True).start()
input_loop()