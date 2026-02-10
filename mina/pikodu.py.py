import time
import cv2
import serial
import numpy as np
from ultralytics import YOLO

# -------------------- AYARLAR (SETTINGS) --------------------
CAM_INDEX = 0
MIRROR_VIEW = True

MODEL_PATH = "yolov8n.pt"
IMGSZ = 320
CONF = 0.50

# "Yakin engel" cizgisi
NEAR_Y_RATIO = 0.55 

# Pico Serial
SERIAL_PORT = "/dev/ttyACM0"
BAUD = 115200

# Mesafe esikleri (cm)
STOP_CM = 25.0
AVOID_CM = 80.0

# ---- Smooth kontrol ----
DEADBAND = 0.08
SWITCH_HYS = 0.12
GAIN = 1.25
STEER_MIN = 0.20
STEER_MAX = 0.90

# Direksiyon filtreleri
EMA_ALPHA = 0.80
MAX_DELTA = 0.10

# Hiz ayarlari
V_CLEAR = 1.00
V_MANEUVER = 0.70
V_REVERSE = -0.60

# -------------------- SERIAL HAZIRLIK --------------------
# Gorseldeki "PICO_BAGLI is not defined" hatasini burada cozuyoruz
PICO_BAGLI = False
ser = None

try:
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.05)
    PICO_BAGLI = True
    print(f"Pico baglantisi basarili: {SERIAL_PORT}")
except Exception as e:
    print(f"Pico baglanamadi: {e}")

# -------------------- FONKSIYONLAR --------------------

def parse_packet(line: str):
    out = {}
    parts = line.strip().split(";")
    for p in parts:
        if "=" not in p: continue
        k, v = p.split("=", 1)
        k, v = k.strip(), v.strip()
        if not k or v.lower() == "nan": continue
        try:
            out[k] = float(v)
        except: pass
    return out

latest_data = {}
def pico_read_latest():
    global latest_data
    if ser and ser.in_waiting > 0:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                data = parse_packet(line)
                if data:
                    latest_data.update(data)
        except: pass
    return latest_data

def send_motor_command(v, s, h_pos=0.0):
    if not PICO_BAGLI or ser is None:
        return
    try:
        motor_komutu = f"V:{v:.2f};S:{s:.2f};H:{h_pos:.2f}\n"
        ser.write(motor_komutu.encode())
    except Exception as e:
        print(f"Seri port hatasi: {e}")

def occupancy_scores(results, w, h, near_y_ratio):
    sol, sag = 0.0, 0.0
    near_line = h * near_y_ratio
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            if y2 > near_line:
                area = max(0.0, (x2 - x1) * (y2 - y1))
                cx = (x1 + x2) / 2.0
                if cx < (w / 2.0): sol += area
                else: sag += area
    return sol, sag, near_line

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

# -------------------- YOLO & KAMERA (Camera Module 3) --------------------
model = YOLO(MODEL_PATH)

# Pi Camera 3 icin Libcamera tabanli pipeline
pipeline = (
    "libcamerasrc ! video/x-raw, width=640, height=480, framerate=30/1 ! "
    "videoconvert ! videoscale ! video/x-raw, width=640, height=480 ! appsink"
)
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

w = 640
h = 480
steer_prev = 0.0
yon_kilit = 0

# -------------------- ANA DONGU --------------------
try:
    while cap.isOpened():
        ok, frame = cap.read()
        if not ok: break

        if MIRROR_VIEW:
            frame = cv2.flip(frame, 1)

        data = pico_read_latest()
        mesafe_cm = data.get("D", None)

        yolo_gerekli = (mesafe_cm is None) or (mesafe_cm < AVOID_CM)
        
        results = []
        if yolo_gerekli:
            results = model.predict(frame, imgsz=IMGSZ, conf=CONF, verbose=False)

        komut = "YOL ACIK: ILERI"
        renk = (0, 255, 0)
        steer_raw = 0.0
        oran = 0.0

        if mesafe_cm is not None and mesafe_cm < STOP_CM:
            komut = "ENGEL! GERI GIDIYOR"
            renk = (0, 0, 255)
            send_motor_command(0.0, 0.0, h_pos=0.0)
            send_motor_command(0.0, 0.0, h_pos=-1.0)
            time.sleep(0.4)
            send_motor_command(0.0, 0.0, h_pos=1.0)
            time.sleep(0.4)
            send_motor_command(V_REVERSE, 0.0, h_pos=0.0)
            continue
        else:
            if yolo_gerekli and results:
                sol_puan, sag_puan, near_line = occupancy_scores(results, w, h, NEAR_Y_RATIO)
                toplam = sol_puan + sag_puan

                if toplam <= 1e-6:
                    steer_raw = 0.0
                    yon_kilit = 0
                else:
                    oran = abs(sol_puan - sag_puan) / toplam
                    hedef_yon = -1 if sol_puan < sag_puan else +1

                    if yon_kilit == 0:
                        yon_kilit = hedef_yon
                    elif hedef_yon != yon_kilit and oran > SWITCH_HYS:
                        yon_kilit = hedef_yon

                    if oran < DEADBAND:
                        steer_raw = 0.0
                    else:
                        mag = clamp(STEER_MIN + (oran - DEADBAND) * GAIN, STEER_MIN, STEER_MAX)
                        steer_raw = yon_kilit * mag
                        renk = (0, 0, 255)

        steer_ema = (EMA_ALPHA * steer_prev) + ((1 - EMA_ALPHA) * steer_raw)
        delta = clamp(steer_ema - steer_prev, -MAX_DELTA, MAX_DELTA)
        steer = clamp(steer_prev + delta, -STEER_MAX, STEER_MAX)
        steer_prev = steer

        v = V_CLEAR if not yolo_gerekli else V_MANEUVER
        if mesafe_cm is not None and mesafe_cm < STOP_CM: v = V_REVERSE

        send_motor_command(v, steer)

        cv2.imshow("Pi4 Camera3 Vision", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

finally:
    cap.release()
    cv2.destroyAllWindows()
    if ser:
        try:
            ser.close()
        except:
            pass 