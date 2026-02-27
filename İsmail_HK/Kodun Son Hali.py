import warnings
warnings.filterwarnings("ignore")  # FutureWarning vs. gizle

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

import cv2
import numpy as np
import serial
import time
import threading
from collections import deque

# YOLO
import torch


# ==============================================================================
# PID CONTROLLER CLASS (DRIVING BRAIN)  [1. KOD - KORUNDU]
# ==============================================================================
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        # P: Proportional (Oransal)
        P = error

        # I: Integral (Toplam)
        self.integral += error
        # Anti-windup (Integral sismesini onle)
        if self.integral > 1000:
            self.integral = 1000
        if self.integral < -1000:
            self.integral = -1000

        # D: Derivative (Turev)
        derivative = error - self.prev_error
        self.prev_error = error

        # PID Formula
        output = (self.Kp * P) + (self.Ki * self.integral) + (self.Kd * derivative)
        return int(output)


# ==============================================================================
# MULTI-THREADED CAMERA CLASS (LIBCAMERA / GSTREAMER) [PI ICIN]
# ==============================================================================
class VideoStream:
    def __init__(self, src=0, width=640, height=480):
        # libcamera ile calismasi icin GStreamer Pipeline (Pi Camera Module 3)
        gst_pipeline = (
            "libcamerasrc ! "
            f"video/x-raw, width={width}, height={height}, framerate=30/1 ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink"
        )

        print("[INFO] GStreamer ile kamera baslatiliyor...")
        self.stream = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        # Kamera isinana kadar bekle
        time.sleep(2.0)

        if self.stream.isOpened():
            (self.grabbed, self.frame) = self.stream.read()
            self.stopped = False
        else:
            print("[ERROR] Kamera GStreamer ile acilamadi!")
            self.grabbed = False
            self.frame = None
            self.stopped = True

    def start(self):
        threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return
            grabbed, frame = self.stream.read()
            if grabbed:
                self.grabbed = grabbed
                self.frame = frame
            else:
                time.sleep(0.01)

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()


# ==============================================================================
# CONFIGURATION (PI ICIN - PC TESTTEKI IYILESTIRILMIS YOLO AYARLARI KORUNDU)
# ==============================================================================
class Config:
    # Camera
    FRAME_WIDTH = 640
    FRAME_HEIGHT = 480
    MIRROR_VIEW = True  # 2. koddan korundu

    # --- PID SETTINGS (1. kod) ---
    PID_KP = 0.55
    PID_KI = 0.00
    PID_KD = 0.35

    # --- SPEED SETTINGS (1. kod) ---
    BASE_SPEED = 30
    MAX_TURN_SPEED = 50

    # --- MEMORY (1. kod) ---
    LANE_WIDTH = 380

    # --- YOLO SETTINGS (2. kod) ---
    MODEL_PATH = "yolov5n.pt"
    IMGSZ = 320
    CONF = 0.50

    # "Yakin engel" cizgisi (2. kod)
    NEAR_Y_RATIO = 0.55

    # Mesafe esikleri (cm) (2. kod)
    STOP_CM = 25.0
    AVOID_CM = 80.0

    # ---- Smooth kontrol (YOLO steer daha anlamli ve daha stabil) ----
    # Not:
    # - steer siddeti sadece sol/sag farktan degil, nesnenin merkeze yakinligindan da etkilenir
    # - "hafif sagda" ile "en sagda" arasinda daha gorunur fark verir
    DEADBAND = 0.10
    SWITCH_HYS = 0.30
    GAIN = 1.15
    STEER_MIN = 0.06
    STEER_MAX = 0.55

    # Direksiyon filtreleri
    EMA_ALPHA = 0.93
    MAX_DELTA = 0.02

    # Hiz ayarlari (2. kod - normalize mantik)
    V_CLEAR = 1.00
    V_MANEUVER = 0.70
    V_REVERSE = -0.60

    # UART / Protokol secimi
    # "LEGACY"  -> <speed,angle>   (1. kod formati)
    # "VS"      -> V:0.70;S:-0.35  (2. kod formati)
    UART_OUTPUT_MODE = "VS"

    # Pi4 UART (GPIO14/15)
    SERIAL_PORT = "/dev/ttyS0"
    BAUD = 115200


# ==============================================================================
# SERIAL COMMUNICATION (PI4 <-> PICO W UART)
# ==============================================================================
ser = None
PICO_BAGLI = False

try:
    ser = serial.Serial(Config.SERIAL_PORT, Config.BAUD, timeout=0.02)
    ser.flush()
    PICO_BAGLI = True
    print(f"[INFO] UART BAGLANDI ({Config.SERIAL_PORT}) - SISTEM ONLINE")
except Exception as e:
    ser = None
    PICO_BAGLI = False
    print(f"[WARNING] UART HATASI - SIMULASYON MODU: {e}")


# --- 1. kod formatı (korundu) ---
def send_command(speed, angle):
    if ser:
        try:
            angle = max(min(int(angle), 100), -100)  # -100..100
            msg = "<{},{}>\n".format(int(speed), angle)
            ser.write(msg.encode('utf-8'))
        except:
            pass


# --- 2. kod formatı (korundu) ---
def send_motor_command(v, s):
    if PICO_BAGLI and ser:
        try:
            motor_komutu = f"V:{v:.2f};S:{s:.2f}\n"
            ser.write(motor_komutu.encode())
        except:
            pass


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def drive_send_unified(speed_int, angle_int):
    """
    Karar mekanizmasi hangi modda olursa olsun tek yerden gönderir.
    - speed_int: 1. kod mantigi (yaklasik -50..50)
    - angle_int: 1. kod mantigi (-100..100)
    """
    # Legacy format (1. kod)
    if Config.UART_OUTPUT_MODE == "LEGACY":
        send_command(speed_int, angle_int)
        return

    # VS format (2. kod) icin donusum
    # speed_int ~ [-50..50] => v ~ [-1..1]
    v = clamp(speed_int / float(Config.MAX_TURN_SPEED), -1.0, 1.0)
    s = clamp(angle_int / 100.0, -1.0, 1.0)
    send_motor_command(v, s)


# ==============================================================================
# PICO'DAN GELEN TELEMETRI PARSING (2. KOD - KORUNDU/PI'YA UYARLANDI)
# Beklenen örnek: "D=73.2;L=0.0;R=0.0"
# ==============================================================================
def parse_packet(line: str):
    out = {}
    parts = line.strip().split(";")
    for p in parts:
        if "=" not in p:
            continue
        k, v = p.split("=", 1)
        k, v = k.strip(), v.strip()
        if not k or v.lower() == "nan":
            continue
        try:
            out[k] = float(v)
        except:
            pass
    return out


latest_data = {}


def pico_read_latest():
    global latest_data
    if ser and ser.in_waiting > 0:
        # Kuyrukta birikenleri son değere kadar oku (en güncel veri)
        try:
            while ser.in_waiting > 0:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    data = parse_packet(line)
                    if data:
                        latest_data.update(data)
        except:
            pass
    return latest_data


def get_distance_cm_from_pico(data_dict):
    """
    Pico'nun gönderdiği mesafe anahtarı farklı olabilir.
    Olası anahtarlar: D, DIST, DIST_CM, mesafe, mesafe_cm
    """
    if not data_dict:
        return 999.0

    for key in ["D", "DIST", "DIST_CM", "distance", "distance_cm", "mesafe", "mesafe_cm"]:
        if key in data_dict:
            try:
                return float(data_dict[key])
            except:
                pass
    return 999.0


# ==============================================================================
# IMAGE PROCESSING & DYNAMIC ROI (1. KOD - KORUNDU)
# ==============================================================================
def preprocess_image(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    edges = cv2.Canny(blur, 50, 150)
    return edges


def dynamic_roi(image, steering_bias):
    """
    Direksiyon ne tarafa donukse robot oraya bakar.
    """
    height, width = image.shape[:2]

    # Bakis acisini kaydir (Shift)
    shift = int(steering_bias * 0.8)

    # ROI Noktalari
    top_left_x = int(width * 0.1) + shift
    top_right_x = int(width * 0.9) + shift

    # Sinirlandirma
    top_left_x = max(0, min(width, top_left_x))
    top_right_x = max(0, min(width, top_right_x))

    polygons = np.array([
        [(0, height), (width, height), (top_right_x, int(height * 0.5)), (top_left_x, int(height * 0.5))]
    ])

    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    return cv2.bitwise_and(image, mask), polygons


def get_lane_lines(lines, width, height, prev_width):
    left_lines, right_lines = [], []

    if lines is None:
        return None, None, prev_width

    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x1 == x2:
            continue

        params = np.polyfit((x1, x2), (y1, y2), 1)
        slope = params[0]
        intercept = params[1]

        # Egim Filtresi (Yatay cizgileri at)
        if abs(slope) < 0.3 or abs(slope) > 3.0:
            continue

        if slope < 0:
            left_lines.append((slope, intercept))
        else:
            right_lines.append((slope, intercept))

    left_lane = np.average(left_lines, axis=0) if left_lines else None
    right_lane = np.average(right_lines, axis=0) if right_lines else None

    return left_lane, right_lane, prev_width


def make_coords(image, line_params):
    if line_params is None:
        return None
    slope, intercept = line_params
    y1 = image.shape[0]
    y2 = int(y1 * 0.65)
    if slope == 0:
        slope = 0.001
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return ((x1, y1), (x2, y2))


# ==============================================================================
# YOLO / ENGEL LOGIC (2. KOD - HASSASIYET / MAGNITUDE IYILESTIRILDI)
# ==============================================================================
def occupancy_scores(results, w, h, near_y_ratio):
    """
    Sol/sag doluluk + merkez baskisi (center_pressure) hesaplar.

    center_pressure:
      0.0 -> nesne/agirlikli merkez kenarlarda
      1.0 -> nesne/agirlikli merkez ekranda merkeze yakin

    Iyilestirme:
    - BBox'in TAM alanini saymak yerine sadece near_line altindaki kismi sayiyoruz
    - BBox'i center cizgisine gore sol/sag alanlara BOLUYORUZ (tek tarafa yazmiyoruz)
    Bu sayede "hafif sagda" ile "en sagda" arasinda fark olusur.
    """
    sol, sag = 0.0, 0.0
    near_line = int(h * near_y_ratio)
    center_x = w / 2.0

    total_area = 0.0
    x_moment = 0.0  # agirlikli x merkezi

    det = results.xyxy[0].cpu().numpy()

    for *xyxy, conf, cls in det:
        x1, y1, x2, y2 = [float(v) for v in xyxy]

        # Goruntu sinirlarina kirp
        x1 = max(0.0, min(float(w), x1))
        x2 = max(0.0, min(float(w), x2))
        y1 = max(0.0, min(float(h), y1))
        y2 = max(0.0, min(float(h), y2))

        if x2 <= x1 or y2 <= y1:
            continue

        # Sadece near_line altini say
        if y2 <= near_line:
            continue

        y_top = max(y1, float(near_line))
        h_near = max(0.0, y2 - y_top)
        if h_near <= 0:
            continue

        w_box = max(0.0, x2 - x1)
        area = w_box * h_near
        if area <= 0:
            continue

        # Agirlikli merkez (center pressure icin)
        cx = (x1 + x2) / 2.0
        total_area += area
        x_moment += cx * area

        # BBox'i center cizgisine gore sol/sag diye bol
        left_w = max(0.0, min(x2, center_x) - x1)
        right_w = max(0.0, x2 - max(x1, center_x))

        sol += left_w * h_near
        sag += right_w * h_near

    center_pressure = 0.0
    if total_area > 1e-6:
        cx_w = x_moment / total_area  # agirlikli merkez x
        dist_norm = abs(cx_w - center_x) / max(1e-6, center_x)  # 0 merkez, 1 kenar
        dist_norm = clamp(dist_norm, 0.0, 1.0)
        center_pressure = 1.0 - dist_norm  # 1 merkezde, 0 kenarda

    return sol, sag, near_line, center_pressure


# ==============================================================================
# HUD (1. KOD + EK BILGILER)
# ==============================================================================
def draw_dashboard(image, error, pid_out, fps, roi_poly,
                   mode_text="", dist_cm=None, yolo_steer=None):
    h, w = image.shape[:2]

    # 1. ROI Cizimi
    if roi_poly is not None:
        cv2.polylines(image, [roi_poly], True, (255, 100, 0), 2)

    # 2. Ust Bilgi Bari (biraz yukselttik)
    cv2.rectangle(image, (0, 0), (w, 95), (0, 0, 0), -1)

    # 3. Veriler
    cv2.putText(image, f"FPS: {int(fps)}", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    # PID bar
    bar_width = int((pid_out / 100) * 100)
    cv2.rectangle(image, (w // 2 - 100, 10), (w // 2 + 100, 30), (100, 100, 100), -1)
    color = (0, 255, 0) if abs(pid_out) < 30 else (0, 0, 255)
    cv2.rectangle(image, (w // 2, 10), (w // 2 + bar_width, 30), color, -1)
    cv2.line(image, (w // 2, 5), (w // 2, 35), (255, 255, 255), 2)
    cv2.putText(image, f"PID: {pid_out}", (w // 2 - 40, 52),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Mode
    cv2.putText(image, f"MODE: {mode_text}", (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # Distance
    if dist_cm is not None:
        renk = (0, 255, 0)
        if dist_cm < Config.AVOID_CM:
            renk = (0, 165, 255)
        if dist_cm <= Config.STOP_CM:
            renk = (0, 0, 255)
        cv2.putText(image, f"DIST: {dist_cm:.1f} cm", (20, 86),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, renk, 2)

    # YOLO steer (HUD ayni kalsin)
    if yolo_steer is not None:
        cv2.putText(image, f"YOLO steer: {yolo_steer:.2f}", (w - 230, 86),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    return image


# ==============================================================================
# MAIN PROGRAM (BIRLESTIRILMIS)
# ==============================================================================
def main():
    print("[INFO] KAMERA BASLATILIYOR (LIBCAMERA MODU)...")
    try:
        vs = VideoStream(width=Config.FRAME_WIDTH, height=Config.FRAME_HEIGHT).start()
    except Exception as e:
        print("[ERROR] KAMERA ACILAMADI! Baglantiyi kontrol et.")
        print("Hata Detayi:", e)
        return

    if vs.stopped:
        print("[FATAL] Kamera baslatilamadi. Gstreamer yuklu mu?")
        return

    # Kameranin isinmasi icin bekle
    time.sleep(1.0)

    # PID init (1. kod)
    pid = PIDController(Config.PID_KP, Config.PID_KI, Config.PID_KD)
    error_buffer = deque(maxlen=5)

    # FPS
    fps_start = time.time()
    frame_count = 0
    fps = 0

    # Lane memory
    avg_width = Config.LANE_WIDTH
    last_steer_pid = 0

    # YOLO state
    steer_prev = 0.0
    yon_kilit = 0
    model = None

    print("[INFO] YOLO model yukleniyor...")
    try:
        # ilk calistirmada internetten indirebilir / yavas olabilir
        model = torch.hub.load('ultralytics/yolov5', 'custom', path=Config.MODEL_PATH)
        model.conf = Config.CONF
        print("[INFO] YOLO hazir.")
    except Exception as e:
        print(f"[WARNING] YOLO yuklenemedi, sadece serif takip calisacak: {e}")
        model = None

    print("[INFO] TEMIRBUG PRO + YOLO SISTEM BASLADI")
    print("[INFO] Cikmak icin 'q' tusuna basin.")

    # son komut (lane kaybında istenirse tutulur)
    last_sent_speed = 0
    last_sent_angle = 0

    try:
        while True:
            # ------------------------------------------------------------------
            # A) KARE AL
            # ------------------------------------------------------------------
            frame = vs.read()
            if frame is None:
                continue

            # 2. koddaki mirror davranisi (opsiyon)
            if Config.MIRROR_VIEW:
                frame = cv2.flip(frame, 1)

            # FPS hesabı
            frame_count += 1
            if time.time() - fps_start > 1:
                fps = frame_count
                frame_count = 0
                fps_start = time.time()

            # ------------------------------------------------------------------
            # B) PICO TELEMETRI (MESAFE)
            # ------------------------------------------------------------------
            pico_data = pico_read_latest()
            mesafe_cm = get_distance_cm_from_pico(pico_data)

            # ------------------------------------------------------------------
            # C) LANE FOLLOWING (1. KOD MANTIGI - HESAPLANIR)
            # ------------------------------------------------------------------
            edges = preprocess_image(frame)
            roi_edges, roi_poly = dynamic_roi(edges, last_steer_pid)

            lines = cv2.HoughLinesP(
                roi_edges, 2, np.pi / 180, 50, np.array([]),
                minLineLength=30, maxLineGap=100
            )

            left_p, right_p, avg_width = get_lane_lines(
                lines, Config.FRAME_WIDTH, Config.FRAME_HEIGHT, avg_width
            )

            center_x = Config.FRAME_WIDTH // 2
            lane_center = None

            l_coords = make_coords(frame, left_p)
            r_coords = make_coords(frame, right_p)

            # lane hesap çıktıları (karar mekanizmasında kullanılacak)
            lane_pid_output = 0
            lane_speed_cmd = 0
            lane_available = False
            lane_error_smooth = 0

            if l_coords and r_coords:
                lane_center = int((l_coords[1][0] + r_coords[1][0]) / 2)
                current_w = r_coords[1][0] - l_coords[1][0]
                avg_width = int(avg_width * 0.95 + current_w * 0.05)
                lane_available = True

            elif l_coords:
                lane_center = int(l_coords[1][0] + (avg_width / 2))
                lane_available = True

            elif r_coords:
                lane_center = int(r_coords[1][0] - (avg_width / 2))
                lane_available = True

            if lane_available and lane_center is not None:
                raw_error = lane_center - center_x
                error_buffer.append(raw_error)
                lane_error_smooth = sum(error_buffer) / len(error_buffer)

                lane_pid_output = pid.compute(lane_error_smooth)
                last_steer_pid = lane_pid_output

                # Dinamik hiz kontrolu (1. kod)
                turn_strength = abs(lane_pid_output)
                lane_speed_cmd = Config.BASE_SPEED - int(turn_strength * 0.2)
                lane_speed_cmd = max(15, lane_speed_cmd)

            # ------------------------------------------------------------------
            # D) YOLO / ENGEL KACIS (2. KOD MANTIGI - GEREKLIYSE)
            # ------------------------------------------------------------------
            yolo_gerekli = (mesafe_cm < Config.AVOID_CM)

            yolo_steer = 0.0
            yolo_total_occ = 0.0
            yolo_near_line = int(Config.FRAME_HEIGHT * Config.NEAR_Y_RATIO)
            yolo_komut_metni = "YOL ACIK"
            yolo_renk = (0, 255, 0)
            annotated_frame = frame.copy()

            if model is not None and yolo_gerekli:
                try:
                    results = model(frame, size=Config.IMGSZ)

                    # Tespit kutularını çiz
                    annotated_frame = np.squeeze(results.render())

                    sol_puan, sag_puan, yolo_near_line, center_pressure = occupancy_scores(
                        results, Config.FRAME_WIDTH, Config.FRAME_HEIGHT, Config.NEAR_Y_RATIO
                    )
                    toplam = sol_puan + sag_puan
                    yolo_total_occ = toplam

                    if toplam > 1e-6:
                        balance_ratio = abs(sol_puan - sag_puan) / toplam
                        hedef_yon = -1 if sol_puan < sag_puan else +1

                        # Yon kilidi: gereksiz sag-sol ziplamalar azalir
                        if yon_kilit == 0 or (hedef_yon != yon_kilit and balance_ratio > Config.SWITCH_HYS):
                            yon_kilit = hedef_yon

                        steer_raw = 0.0

                        # Yeni magnitude sinyali:
                        # - center_pressure ana etken (merkeze girince daha guclu kacis)
                        # - balance_ratio yan etken (hangi taraf daha dolu guveni)
                        mag_signal = (0.80 * center_pressure) + (0.20 * balance_ratio)
                        mag_signal = clamp(mag_signal, 0.0, 1.0)

                        if mag_signal >= Config.DEADBAND:
                            x = (mag_signal - Config.DEADBAND) / max(1e-6, (1.0 - Config.DEADBAND))
                            x = clamp(x, 0.0, 1.0)

                            # Easing (lineer degil): kademeli buyume
                            x_soft = x * x

                            mag = Config.STEER_MIN + (Config.STEER_MAX - Config.STEER_MIN) * x_soft * Config.GAIN
                            mag = clamp(mag, Config.STEER_MIN, Config.STEER_MAX)

                            steer_raw = yon_kilit * mag

                            yolo_renk = (0, 0, 255)
                            if yon_kilit == -1:
                                yolo_komut_metni = "<<< SOLA KAC"
                            else:
                                yolo_komut_metni = "SAGA KAC >>>"

                        # Smooth filtreleme
                        steer_ema = (Config.EMA_ALPHA * steer_prev) + ((1 - Config.EMA_ALPHA) * steer_raw)
                        delta = clamp(steer_ema - steer_prev, -Config.MAX_DELTA, Config.MAX_DELTA)
                        yolo_steer = clamp(steer_prev + delta, -Config.STEER_MAX, Config.STEER_MAX)
                        steer_prev = yolo_steer
                    else:
                        # Yakinda engel yoksa steer sifira yumuşat
                        steer_ema = (Config.EMA_ALPHA * steer_prev)
                        delta = clamp(steer_ema - steer_prev, -Config.MAX_DELTA, Config.MAX_DELTA)
                        yolo_steer = clamp(steer_prev + delta, -Config.STEER_MAX, Config.STEER_MAX)
                        steer_prev = yolo_steer

                except Exception:
                    # YOLO frame bazli hata verirse sistemi dusurme
                    # Lane takibi devam etsin
                    pass

            # ------------------------------------------------------------------
            # E) GORSEL (tek pencere) - YOLO kutulari varsa onlarin ustune lane HUD
            # ------------------------------------------------------------------
            final_view = annotated_frame.copy()

            # 1. kod lane çizimleri
            if l_coords:
                color_l = (0, 255, 0) if r_coords else (0, 255, 255)
                cv2.line(final_view, l_coords[0], l_coords[1], color_l, 5)
            if r_coords:
                color_r = (0, 255, 0) if l_coords else (0, 255, 255)
                cv2.line(final_view, r_coords[0], r_coords[1], color_r, 5)

            if lane_center is not None:
                cv2.circle(final_view, (lane_center, int(Config.FRAME_HEIGHT * 0.65)), 10, (0, 0, 255), -1)
            else:
                cv2.putText(final_view, "KAYIP!", (260, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)

            # 2. koddaki near obstacle çizgisi
            cv2.line(final_view, (0, yolo_near_line), (Config.FRAME_WIDTH, yolo_near_line), (255, 0, 0), 2)

            # ------------------------------------------------------------------
            # F) KARAR MEKANIZMASI (ASIL BIRLESTIRME NOKTASI)
            # ------------------------------------------------------------------
            mode = "LANE"
            cmd_speed = 0
            cmd_angle = 0

            # 1) Cok yakin engel -> güvenlik/geri manevra
            if mesafe_cm <= Config.STOP_CM:
                mode = "EMERGENCY_REVERSE"
                cmd_speed = int(Config.V_REVERSE * Config.MAX_TURN_SPEED)  # örn: -30
                # yolo_steer (±STEER_MAX) -> angle (±90)
                cmd_angle = int(clamp((yolo_steer / max(1e-6, Config.STEER_MAX)) * 90.0, -90.0, 90.0))

            # 2) Engel yakın -> YOLO kaçış modu
            elif mesafe_cm < Config.AVOID_CM and model is not None:
                mode = "YOLO_AVOID"
                cmd_speed = int(Config.V_MANEUVER * Config.MAX_TURN_SPEED)  # örn: +35
                # yolo_steer (±STEER_MAX) -> angle (±90)
                cmd_angle = int(clamp((yolo_steer / max(1e-6, Config.STEER_MAX)) * 90.0, -90.0, 90.0))

            # 3) Yol açık -> Lane PID modu
            else:
                if lane_available:
                    mode = "LANE_PID"
                    cmd_speed = lane_speed_cmd
                    cmd_angle = lane_pid_output
                else:
                    # Şerit kayıp: orijinal davranışı bozmamak için son komutu koru (hold)
                    mode = "LANE_LOST_HOLD"
                    cmd_speed = last_sent_speed
                    cmd_angle = last_sent_angle

            # Komutu gönder
            drive_send_unified(cmd_speed, cmd_angle)
            last_sent_speed, last_sent_angle = cmd_speed, cmd_angle

            # ------------------------------------------------------------------
            # G) EKRAN BILGILERI (YOLO + LANE + MODE)
            # ------------------------------------------------------------------
            cv2.putText(final_view, f"Komut: {yolo_komut_metni}", (20, Config.FRAME_HEIGHT - 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, yolo_renk, 2)
            cv2.putText(final_view, f"YOLO Steer: {yolo_steer:.2f}", (20, Config.FRAME_HEIGHT - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            draw_dashboard(
                final_view,
                lane_error_smooth if lane_available else 0,
                lane_pid_output if lane_available else 0,
                fps,
                roi_poly[0] if roi_poly is not None else None,
                mode_text=mode,
                dist_cm=mesafe_cm,
                yolo_steer=yolo_steer
            )

            cv2.imshow("TEMIRBUG PRO HUD + YOLO", final_view)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                drive_send_unified(0, 0)
                break

    except KeyboardInterrupt:
        drive_send_unified(0, 0)

    finally:
        vs.stop()
        cv2.destroyAllWindows()
        if ser:
            ser.close()


if __name__ == "__main__":
    main()