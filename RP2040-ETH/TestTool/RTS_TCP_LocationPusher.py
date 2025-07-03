import socket
import struct
import time
import threading
import random

# Server 資訊
HOST = '192.168.144.30'
PORT = 1000

s = None
current_lat = 24.947325
current_lon = 121.229435
current_alt = 31.0000
lock = threading.Lock()
running = True

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
        #print(f"client: sent lat={lat}, lon={lon}, alt={alt}")
        response = s.recv(1024).decode('utf-8', errors='ignore')
        #print(f"server: {response.strip()}")
    except Exception as e:
        print(f"client: error during send/recv: {e}")
        s.close()
        connect_to_server()

def auto_send_loop():
    global current_lat, current_lon, current_alt, running
    while running:
        with lock:
            lat = current_lat + random.uniform(-0.0000005, 0.0000005)
            lon = current_lon + random.uniform(-0.0000005, 0.0000005)
            alt = current_alt + random.uniform(-0.01, 0.01)  # 高度微抖動
        send_floats(lat, lon, alt)
        time.sleep(1)

def input_loop():
    global current_lat, current_lon, current_alt, running
    while True:
        try:
            user_input = input("client: enter lon lat  alt (e.g., 23.111111 121.1111  10.000): ")
            parts = user_input.strip().split()
            if len(parts) != 3:
                print("client: invalid input, expected 3 values")
                continue
            lat, lon, alt = map(float, parts)
            with lock:
                current_lat = lat
                current_lon = lon
                current_alt = alt
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

# 啟動發送執行緒
threading.Thread(target=auto_send_loop, daemon=True).start()

# 啟動輸入迴圈
input_loop()