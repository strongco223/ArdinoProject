import socket
import struct
import time
import threading
import serial
import math

# TCP Server 資訊
HOST = '192.168.144.30'
PORT = 1000

# Serial Port 設定
SERIAL_PORT = 'COM16'
BAUD_RATE = 9600

a = 6378137.0
f = 1 / 298.257223563
b = a * (1 - f)
e_sq = f * (2 - f)

s = None
ser = None
origin_set = False
origin_lat = 0.0
origin_lon = 0.0
origin_alt = 0.0
origin_ecef = (0.0, 0.0, 0.0)
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

def connect_to_serial():
    global ser
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"serial: connected to {SERIAL_PORT}")
            return
        except Exception as e:
            print(f"serial: connection failed, retrying in 3s... ({e})")
            time.sleep(3)

def send_floats(lat, lon, alt):
    global s
    packet = b'\xAA' + struct.pack('<fff', lat, lon, alt)
    try:
        s.sendall(packet)
    except Exception as e:
        print(f"client: error during send: {e}")
        s.close()
        connect_to_server()

def geodetic_to_ecef(lat, lon, alt):
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    N = a / math.sqrt(1 - e_sq * math.sin(lat_rad)**2)
    x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
    z = ((b**2 / a**2) * N + alt) * math.sin(lat_rad)
    return (x, y, z)

def ecef_to_geodetic(x, y, z):
    lon = math.atan2(y, x)
    p = math.sqrt(x*x + y*y)
    theta = math.atan2(z * a, p * b)
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)
    lat = math.atan2(z + (e_sq * b) * sin_theta**3, p - (e_sq * a) * cos_theta**3)
    N = a / math.sqrt(1 - e_sq * math.sin(lat)**2)
    alt = p / math.cos(lat) - N
    return (math.degrees(lat), math.degrees(lon), alt)

def enu_to_ecef(n, e, u):
    lat0_rad = math.radians(origin_lat)
    lon0_rad = math.radians(origin_lon)
    x0, y0, z0 = origin_ecef
    t = [
        [-math.sin(lon0_rad), -math.sin(lat0_rad)*math.cos(lon0_rad), math.cos(lat0_rad)*math.cos(lon0_rad)],
        [ math.cos(lon0_rad), -math.sin(lat0_rad)*math.sin(lon0_rad), math.cos(lat0_rad)*math.sin(lon0_rad)],
        [ 0, math.cos(lat0_rad), math.sin(lat0_rad)]
    ]
    dx = t[0][0]*e + t[0][1]*n + t[0][2]*u
    dy = t[1][0]*e + t[1][1]*n + t[1][2]*u
    dz = t[2][0]*e + t[2][1]*n + t[2][2]*u
    return (x0 + dx, y0 + dy, z0 + dz)

def serial_loop():
    global origin_set
    while running:
        try:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue
            parts = line.split(',')
            if len(parts) >= 4:
                n = float(parts[1])
                e = float(parts[2])
                h = float(parts[3])
                with lock:
                    if origin_set:
                        x, y, z = enu_to_ecef(n, e, h)
                        lat, lon, alt = ecef_to_geodetic(x, y, z)
                        send_floats(lat, lon, alt)
        except Exception as e:
            print(f"serial: error: {e}")
            ser.close()
            connect_to_serial()

def input_origin():
    global origin_set, origin_lat, origin_lon, origin_alt, origin_ecef
    while True:
        try:
            user_input = input("請輸入原點經緯度與高度 (lat lon alt): ")
            lat, lon, alt = map(float, user_input.strip().split())
            with lock:
                origin_lat = lat
                origin_lon = lon
                origin_alt = alt
                origin_ecef = geodetic_to_ecef(lat, lon, alt)
                origin_set = True
                print(f"原點設定成功：lat={lat}, lon={lon}, alt={alt}")
                break
        except Exception as e:
            print(f"輸入錯誤：{e}")

connect_to_server()
connect_to_serial()
input_origin()

threading.Thread(target=serial_loop, daemon=True).start()

try:
    while running:
        time.sleep(1)
except KeyboardInterrupt:
    running = False
    if s:
        s.close()
    if ser:
        ser.close()
    print("\n程式結束")
