import cv2
import numpy as np
import serial
import time
import threading
from collections import deque

# ==============================================================================
#  PID KONTROLCÜ SINIFI (PROFESYONEL SÜRÜŞÜN SIRRI)
# ==============================================================================
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        
    def compute(self, error):
        # P: Oransal Hata
        P = error
        
        # I: Toplam Hata (Zamanla biriken hatayı düzeltir)
        self.integral += error
        # Integral şişmesini önle (Windup guard)
        if self.integral > 1000: self.integral = 1000
        if self.integral < -1000: self.integral = -1000
        
        # D: Hata Değişim Hızı (Ani hareketleri frenler)
        derivative = error - self.prev_error
        self.prev_error = error
        
        # PID Formülü
        output = (self.Kp * P) + (self.Ki * self.integral) + (self.Kd * derivative)
        return int(output)

# ==============================================================================
#  MULTI-THREADED KAMERA SINIFI (FPS ARTIRICI)
# ==============================================================================
class VideoStream:
    def __init__(self, src=0, width=640, height=480):
        self.stream = cv2.VideoCapture(src)
        self.stream.set(3, width)
        self.stream.set(4, height)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False
        
    def start(self):
        # Kamerayı ayrı bir işlem parçacığında (thread) başlat
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
#  KONFİGÜRASYON (AYARLAR)
# ==============================================================================
class Config:
    FRAME_WIDTH = 640
    FRAME_HEIGHT = 480
    
    # --- PID AYARLARI (BUNLARLA OYNAYABİLİRSİN) ---
    # Kp: Ana dönüş gücü (Azsa dönemez, çoksa titrer)
    # Ki: İnce ayar (Genelde 0 veya çok küçük tutulur)
    # Kd: Frenleme etkisi (Titremeyi azaltır)
    PID_KP = 0.55
    PID_KI = 0.00
    PID_KD = 0.35
    
    # Hız
    BASE_SPEED = 30
    MAX_TURN_SPEED = 50 # PID çıktısı buna eklenir
    
    # Hafıza
    LANE_WIDTH = 380

# ==============================================================================
#  PICO İLE HABERLEŞME
# ==============================================================================
try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    ser.flush()
    print("✅ PICO BAĞLANDI - SISTEM ONLINE")
except:
    ser = None
    print("⚠️ PICO YOK - SİMÜLASYON MODU")

def send_command(speed, angle):
    if ser:
        try:
            # PID çıktısını sınırla (-100 ile 100 arası)
            angle = max(min(angle, 100), -100)
            msg = f"<{speed},{angle}>\n"
            ser.write(msg.encode('utf-8'))
        except: pass

# ==============================================================================
#  GELİŞMİŞ GÖRÜNTÜ İŞLEME & DİNAMİK ROI
# ==============================================================================
def preprocess_image(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    edges = cv2.Canny(blur, 50, 150)
    return edges

def dynamic_roi(image, steering_bias):
    """
    Direksiyon ne tarafa dönükse, robot oraya bakar.
    steering_bias: Önceki döngüdeki hata değeri (Negatif: Sol, Pozitif: Sağ)
    """
    height, width = image.shape[:2]
    
    # Bakış açısını kaydır (Shift)
    # Hata -50 ise (Sola dönüyorsa), shift -50 olur.
    shift = int(steering_bias * 0.8) 
    
    # ROI Noktaları (Yamuk)
    # Üst noktaları kaydırıyoruz
    top_left_x = int(width * 0.1) + shift
    top_right_x = int(width * 0.9) + shift
    
    # Sınırların dışına çıkmasın
    top_left_x = max(0, min(width, top_left_x))
    top_right_x = max(0, min(width, top_right_x))
    
    polygons = np.array([
        [(0, height), (width, height), (top_right_x, int(height*0.5)), (top_left_x, int(height*0.5))]
    ])
    
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    return cv2.bitwise_and(image, mask), polygons # Poligonu da döndür ki çizelim

# --- Çizgi ve Hafıza Mantığı (Klasik Hough) ---
def get_lane_lines(lines, width, height, prev_width):
    left_lines, right_lines = [], []
    
    if lines is None: return None, None, prev_width
    
    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x1 == x2: continue
        params = np.polyfit((x1, x2), (y1, y2), 1)
        slope = params[0]
        intercept = params[1]
        
        if abs(slope) < 0.3 or abs(slope) > 3.0: continue
        
        if slope < 0: left_lines.append((slope, intercept))
        else: right_lines.append((slope, intercept))
    
    # Ortalamaları al
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
#  PROFESYONEL HUD (DASHBOARD)
# ==============================================================================
def draw_dashboard(image, error, pid_out, fps, roi_poly):
    h, w = image.shape[:2]
    
    # 1. ROI Alanını Çiz (Mavi Çerçeve)
    if roi_poly is not None:
        cv2.polylines(image, [roi_poly], True, (255, 100, 0), 2)
    
    # 2. Üst Bilgi Barı (Siyah Şerit)
    cv2.rectangle(image, (0, 0), (w, 60), (0, 0, 0), -1)
    
    # 3. Veriler
    # FPS
    cv2.putText(image, f"FPS: {int(fps)}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    # PID Çıktısı (Bar şeklinde göster)
    bar_width = int((pid_out / 100) * 100) # Ölçekle
    cv2.rectangle(image, (w//2 - 100, 20), (w//2 + 100, 40), (100, 100, 100), -1) # Arkaplan
    
    color = (0, 255, 0) if abs(pid_out) < 30 else (0, 0, 255)
    cv2.rectangle(image, (w//2, 20), (w//2 + bar_width, 40), color, -1) # Değer
    cv2.line(image, (w//2, 15), (w//2, 45), (255, 255, 255), 2) # Merkez çizgisi
    
    cv2.putText(image, f"PID: {pid_out}", (w//2 - 40, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return image

# ==============================================================================
#  ANA PROGRAM (MAIN LOOP)
# ==============================================================================
def main():
    # 1. Sistem Başlatılıyor
    vs = VideoStream().start()
    time.sleep(1.0) # Kameranın ısınmasını bekle
    
    # 2. PID Kontrolcü
    pid = PIDController(Config.PID_KP, Config.PID_KI, Config.PID_KD)
    
    # 3. Filtreler (Gürültü Önleme)
    error_buffer = deque(maxlen=5) # Son 5 hatanın ortalamasını alacağız
    fps_start = time.time()
    frame_count = 0
    fps = 0
    
    avg_width = Config.LANE_WIDTH
    last_steer = 0 # Dinamik ROI için
    
    print("--- TEMİRBUĞ PRO SYSTEM STARTED ---")
    
    try:
        while True:
            # A. Görüntü Al (Thread'den gelir, bekleme yapmaz)
            frame = vs.read()
            if frame is None: break
            
            # FPS Hesabı
            frame_count += 1
            if time.time() - fps_start > 1:
                fps = frame_count
                frame_count = 0
                fps_start = time.time()
            
            # B. İşleme
            edges = preprocess_image(frame)
            
            # C. Dinamik ROI (Önceki hataya göre bakış açısını kaydır)
            roi_edges, roi_poly = dynamic_roi(edges, last_steer)
            
            # D. Hough Transform
            lines = cv2.HoughLinesP(roi_edges, 2, np.pi/180, 50, np.array([]), minLineLength=30, maxLineGap=100)
            
            # E. Duvarları Bul
            left_p, right_p, avg_width = get_lane_lines(lines, Config.FRAME_WIDTH, Config.FRAME_HEIGHT, avg_width)
            
            # F. Hata Hesapla
            center_x = Config.FRAME_WIDTH // 2
            lane_center = None
            
            # Koordinatları Al
            l_coords = make_coords(frame, left_p)
            r_coords = make_coords(frame, right_p)
            
            # Çizim için ana resmin kopyası
            final_view = frame.copy()
            
            if l_coords and r_coords:
                lane_center = int((l_coords[1][0] + r_coords[1][0]) / 2)
                # Genişliği öğren
                current_w = r_coords[1][0] - l_coords[1][0]
                avg_width = int(avg_width * 0.95 + current_w * 0.05)
                # Çiz
                cv2.line(final_view, l_coords[0], l_coords[1], (0, 255, 0), 5)
                cv2.line(final_view, r_coords[0], r_coords[1], (0, 255, 0), 5)
                
            elif l_coords:
                lane_center = int(l_coords[1][0] + (avg_width / 2))
                cv2.line(final_view, l_coords[0], l_coords[1], (0, 255, 255), 5) # Sarı (Tek duvar)
                
            elif r_coords:
                lane_center = int(r_coords[1][0] - (avg_width / 2))
                cv2.line(final_view, r_coords[0], r_coords[1], (0, 255, 255), 5) # Sarı (Tek duvar)
            
            # G. Sürüş Kararı (PID + Filtre)
            if lane_center is not None:
                raw_error = lane_center - center_x
                
                # Hata Tamponuna Ekle (Smoothing)
                error_buffer.append(raw_error)
                smooth_error = sum(error_buffer) / len(error_buffer)
                
                # PID Hesapla
                pid_output = pid.compute(smooth_error)
                last_steer = pid_output # ROI için kaydet
                
                # Virajda yavaşla
                if abs(pid_output) > 30:
                    current_speed = Config.BASE_SPEED - 5
                else:
                    current_speed = Config.BASE_SPEED
                
                send_command(current_speed, pid_output)
                
                # Dashboard Çiz
                cv2.circle(final_view, (lane_center, int(Config.FRAME_HEIGHT*0.65)), 10, (0, 0, 255), -1) # Hedef Nokta
                draw_dashboard(final_view, smooth_error, pid_output, fps, roi_poly[0])
                
            else:
                # Duvar yoksa
                cv2.putText(final_view, "KAYIP!", (300, 240), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)
            
            cv2.imshow("TEMIRBUG PRO HUD", final_view)
            # cv2.imshow("Robot Eye", roi_edges) # İstersen aç
            
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