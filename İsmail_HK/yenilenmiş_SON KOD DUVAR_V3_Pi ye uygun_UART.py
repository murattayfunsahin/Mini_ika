import cv2
import numpy as np
import serial
import time
import threading
from collections import deque

# ==============================================================================
# PID CONTROLLER CLASS (DRIVING BRAIN)
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
        if self.integral > 1000: self.integral = 1000
        if self.integral < -1000: self.integral = -1000
        
        # D: Derivative (Turev)
        derivative = error - self.prev_error
        self.prev_error = error
        
        # PID Formula
        output = (self.Kp * P) + (self.Ki * self.integral) + (self.Kd * derivative)
        return int(output)

# ==============================================================================
# MULTI-THREADED CAMERA CLASS (MODIFIYE EDILDI: LIBCAMERA / GSTREAMER)
# ==============================================================================
class VideoStream:
    def __init__(self, src=0, width=640, height=480):
        # --- MODIFIYE BASLANGICI ---
        # libcamera ile calismasi icin GStreamer Pipeline kullaniyoruz.
        # Bu satir, kameranin donanimini dogrudan 'libcamerasrc' ile baglar.
        
        gst_pipeline = (
            "libcamerasrc ! "
            "video/x-raw, width=640, height=480, framerate=30/1 ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink"
        )
        
        print("[INFO] GStreamer ile kamera baslatiliyor...")
        # cv2.CAP_GSTREAMER parametresi cok onemlidir
        self.stream = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        # --- MODIFIYE BITISI ---
        
        # Kamera isinana kadar bekle (Pi kameralari icin onemli)
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
        # Kamerayi ayri is parcaciginda baslat
        threading.Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            if self.stopped: return
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()

# ==============================================================================
# CONFIGURATION
# ==============================================================================
class Config:
    FRAME_WIDTH = 640
    FRAME_HEIGHT = 480
    
    # --- PID SETTINGS ---
    # Kp: Donus gucu
    # Ki: Gecmis hata (Genelde 0)
    # Kd: Titreme onleyici (Frenleme)
    PID_KP = 0.55
    PID_KI = 0.00
    PID_KD = 0.35
    
    # SPEED SETTINGS
    BASE_SPEED = 30
    MAX_TURN_SPEED = 50 
    
    # MEMORY
    LANE_WIDTH = 380

# ==============================================================================
# SERIAL COMMUNICATION (PHYSICAL UART - GPIO)
# ==============================================================================
try:
    # --- KRITIK DEGISIKLIK ---
    # Pi 4 GPIO 14/15 pinleri (UART) genelde /dev/ttyS0 adresindedir.
    # USB kablo ile degil, pinlerle bagladigimiz icin bunu kullaniyoruz.
    ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
    ser.flush()
    print("[INFO] UART (GPIO PINLERI) BAGLANDI - SISTEM ONLINE")
except:
    ser = None
    print("[WARNING] UART HATASI - SIMULASYON MODU")

def send_command(speed, angle):
    if ser:
        try:
            # Acisi sinirla (-100 ile 100 arasi)
            angle = max(min(angle, 100), -100)
            msg = "<{},{}>\n".format(speed, angle)
            ser.write(msg.encode('utf-8'))
        except: pass

# ==============================================================================
# IMAGE PROCESSING & DYNAMIC ROI
# ==============================================================================
def preprocess_image(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Blur seviyesi 7x7 idealdir
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    # Canny esikleri 50-150
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
        [(0, height), (width, height), (top_right_x, int(height*0.5)), (top_left_x, int(height*0.5))]
    ])
    
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    return cv2.bitwise_and(image, mask), polygons

# --- Wall / Lane Logic ---
def get_lane_lines(lines, width, height, prev_width):
    left_lines, right_lines = [], []
    
    if lines is None: return None, None, prev_width
    
    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x1 == x2: continue
        params = np.polyfit((x1, x2), (y1, y2), 1)
        slope = params[0]
        intercept = params[1]
        
        # Egim Filtresi (Yatay cizgileri at)
        if abs(slope) < 0.3 or abs(slope) > 3.0: continue
        
        if slope < 0: left_lines.append((slope, intercept))
        else: right_lines.append((slope, intercept))
    
    left_lane = np.average(left_lines, axis=0) if left_lines else None
    right_lane = np.average(right_lines, axis=0) if right_lines else None
    
    return left_lane, right_lane, prev_width

def make_coords(image, line_params):
    if line_params is None: return None
    slope, intercept = line_params
    y1 = image.shape[0]
    y2 = int(y1 * 0.65)
    if slope == 0: slope = 0.001
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return ((x1, y1), (x2, y2))

# ==============================================================================
# HUD (DASHBOARD)
# ==============================================================================
def draw_dashboard(image, error, pid_out, fps, roi_poly):
    h, w = image.shape[:2]
    
    # 1. ROI Cizimi
    if roi_poly is not None:
        cv2.polylines(image, [roi_poly], True, (255, 100, 0), 2)
    
    # 2. Ust Bilgi Bari
    cv2.rectangle(image, (0, 0), (w, 60), (0, 0, 0), -1)
    
    # 3. Veriler
    # FPS
    cv2.putText(image, "FPS: {}".format(int(fps)), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    # PID Ciktisi
    bar_width = int((pid_out / 100) * 100)
    cv2.rectangle(image, (w//2 - 100, 20), (w//2 + 100, 40), (100, 100, 100), -1)
    
    color = (0, 255, 0) if abs(pid_out) < 30 else (0, 0, 255)
    cv2.rectangle(image, (w//2, 20), (w//2 + bar_width, 40), color, -1)
    cv2.line(image, (w//2, 15), (w//2, 45), (255, 255, 255), 2)
    
    cv2.putText(image, "PID: {}".format(pid_out), (w//2 - 40, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return image

# ==============================================================================
# MAIN PROGRAM
# ==============================================================================
def main():
    print("[INFO] KAMERA BASLATILIYOR (LIBCAMERA MODU)...")
    try:
        vs = VideoStream().start()
    except Exception as e:
        print("[ERROR] KAMERA ACILAMADI! Baglantiyi kontrol et.")
        print("Hata Detayi:", e)
        return
        
    if vs.stopped:
        print("[FATAL] Kamera baslatilamadi. Gstreamer yuklu mu?")
        return

    # Kameranin isinmasi icin bekle
    time.sleep(1.0)
    
    pid = PIDController(Config.PID_KP, Config.PID_KI, Config.PID_KD)
    
    error_buffer = deque(maxlen=5) 
    fps_start = time.time()
    frame_count = 0
    fps = 0
    
    avg_width = Config.LANE_WIDTH
    last_steer = 0 
    
    print("[INFO] TEMIRBUG PRO SISTEM BASLADI")
    print("[INFO] Cikmak icin 'q' tusuna basin.")
    
    try:
        while True:
            # A. Goruntu Al
            frame = vs.read()
            
            # Eger kamera hata verirse veya goruntu alamazsa donguyu kir
            if frame is None:
                continue # Ilk karelerde bos gelebilir
            
            # FPS Hesabi
            frame_count += 1
            if time.time() - fps_start > 1:
                fps = frame_count
                frame_count = 0
                fps_start = time.time()
            
            # B. Isleme
            edges = preprocess_image(frame)
            
            # C. Dinamik ROI
            roi_edges, roi_poly = dynamic_roi(edges, last_steer)
            
            # D. Hough Transform
            lines = cv2.HoughLinesP(roi_edges, 2, np.pi/180, 50, np.array([]), minLineLength=30, maxLineGap=100)
            
            # E. Duvarlari Bul
            left_p, right_p, avg_width = get_lane_lines(lines, Config.FRAME_WIDTH, Config.FRAME_HEIGHT, avg_width)
            
            # F. Hata Hesapla
            center_x = Config.FRAME_WIDTH // 2
            lane_center = None
            
            l_coords = make_coords(frame, left_p)
            r_coords = make_coords(frame, right_p)
            
            final_view = frame.copy()
            
            if l_coords and r_coords:
                lane_center = int((l_coords[1][0] + r_coords[1][0]) / 2)
                current_w = r_coords[1][0] - l_coords[1][0]
                avg_width = int(avg_width * 0.95 + current_w * 0.05)
                # Ciz
                cv2.line(final_view, l_coords[0], l_coords[1], (0, 255, 0), 5)
                cv2.line(final_view, r_coords[0], r_coords[1], (0, 255, 0), 5)
                
            elif l_coords:
                lane_center = int(l_coords[1][0] + (avg_width / 2))
                cv2.line(final_view, l_coords[0], l_coords[1], (0, 255, 255), 5) 
                
            elif r_coords:
                lane_center = int(r_coords[1][0] - (avg_width / 2))
                cv2.line(final_view, r_coords[0], r_coords[1], (0, 255, 255), 5) 
            
            # G. Surus Karari
            if lane_center is not None:
                raw_error = lane_center - center_x
                
                error_buffer.append(raw_error)
                smooth_error = sum(error_buffer) / len(error_buffer)
                
                pid_output = pid.compute(smooth_error)
                last_steer = pid_output 
                
                # --- DINAMIK HIZ KONTROLU (BURASI DEGISTI) ---
                turn_strength = abs(pid_output)
                current_speed = Config.BASE_SPEED - int(turn_strength * 0.2)
                current_speed = max(15, current_speed) # Viraj ne kadar keskin olursa olsun hiz 15'in altina inmez
                # ---------------------------------------------
                
                send_command(current_speed, pid_output)
                
                # Dashboard
                cv2.circle(final_view, (lane_center, int(Config.FRAME_HEIGHT*0.65)), 10, (0, 0, 255), -1)
                draw_dashboard(final_view, smooth_error, pid_output, fps, roi_poly[0])
                
            else:
                cv2.putText(final_view, "KAYIP!", (300, 240), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)
            
            cv2.imshow("TEMIRBUG PRO HUD", final_view)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                send_command(0, 0)
                break

    except KeyboardInterrupt:
        send_command(0, 0)
        
    finally:
        vs.stop()
        cv2.destroyAllWindows()
        if ser: ser.close()

if __name__ == "__main__":
    main()