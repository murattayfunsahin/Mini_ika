import machine, utime, sys, math
import select
from machine import I2C, Pin, PWM

# ==================== YENİ FİZİKSEL AYARLAR ====================
# Mesafe Sensörü (HC-SR04)
TRIG_PIN = 0  # Yeni: GP0
ECHO_PIN = 1  # Yeni: GP1

# MPU-6050 (I2C)
SDA_PIN = 2   # Yeni: GP2
SCL_PIN = 3   # Yeni: GP3
I2C_ID = 1    # GP2/GP3 genellikle I2C1 veri yolundadır

# Motor ve Servo Pinleri (Öncekiyle aynı bıraktım, gerekirse değiştirirsin)
SOL_SERVO_PIN = 20
SAG_SERVO_PIN = 21
RPWM_PIN = 2
LPWM_PIN = 3
EN_PIN = 4

# Sabitler
SERVO_CENTER = 90
SERVO_RANGE = 45
MOTOR_FREQ = 15000
MPU_ADDR = 0x68
HILL_SIGN = 1.0
HILL_K = 0.02
SLOW_DIST = 40.0
STOP_DIST = 15.0

poll_obj = select.poll()
poll_obj.register(sys.stdin, select.POLLIN)
buffer = ""

# ==================== DONANIM BAŞLATMA ====================
rpwm = PWM(Pin(RPWM_PIN))
lpwm = PWM(Pin(LPWM_PIN))
en = Pin(EN_PIN, Pin.OUT)
en.high()
rpwm.freq(MOTOR_FREQ)
lpwm.freq(MOTOR_FREQ)

sol_servo = PWM(Pin(SOL_SERVO_PIN)); sol_servo.freq(50)
sag_servo = PWM(Pin(SAG_SERVO_PIN)); sag_servo.freq(50)

trig = Pin(TRIG_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)

# I2C Ayarı (SDA=GP2, SCL=GP3)
i2c = I2C(I2C_ID, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000)

try:
    i2c.writeto_mem(MPU_ADDR, 0x6B, b"\x00")
except:
    print("MPU Baglanti Hatasi! Kablolari kontrol et.")

# ==================== FONKSİYONLAR ====================
def clamp(x, lo, hi):
    return max(lo, min(x, hi))

def set_motor(speed):
    speed = clamp(speed, -0.8, 0.8)
    duty = int(abs(speed) * 65535)
    if speed > 0.02:
        rpwm.duty_u16(duty); lpwm.duty_u16(0)
    elif speed < -0.02:
        rpwm.duty_u16(0); lpwm.duty_u16(duty)
    else:
        rpwm.duty_u16(0); lpwm.duty_u16(0)

def set_steering(steer):
    steer = clamp(steer, -1.0, 1.0)
    sol_aci = SERVO_CENTER + (steer * SERVO_RANGE)
    sag_aci = SERVO_CENTER - (steer * SERVO_RANGE)
    sol_servo.duty_u16(int(1638 + (sol_aci / 180.0) * (8192 - 1638)))
    sag_servo.duty_u16(int(1638 + (sag_aci / 180.0) * (8192 - 1638)))

def get_distance_filt():
    trig.low(); utime.sleep_us(2)
    trig.high(); utime.sleep_us(10); trig.low()
    t0 = utime.ticks_us()
    
    # Echo başlangıcını bekle
    timeout = 26000 # Yaklaşık 4 metre sınırı
    while echo.value() == 0:
        if utime.ticks_diff(utime.ticks_us(), t0) > timeout: return 400.0
    
    t1 = utime.ticks_us()
    # Echo bitişini bekle
    while echo.value() == 1:
        if utime.ticks_diff(utime.ticks_us(), t1) > timeout: return 400.0
    
    t2 = utime.ticks_us()
    pulse_width = utime.ticks_diff(t2, t1)
    return (pulse_width * 0.0343) / 2.0

def update_pitch():
    try:
        data = i2c.readfrom_mem(MPU_ADDR, 0x3B, 2)
        ax = (data[0] << 8) | data[1]
        if ax > 32768: ax -= 65536
        return (ax / 16384.0) * 90.0
    except:
        return 0.0

# ==================== ANA DÖNGÜ ====================
v_cmd = 0.5
s_cmd = 0.0
last_print = 0

print("Fiziksel İKA Başlatıldı...")

while True:
    now = utime.ticks_ms()
    
    # Seri Okuma (Komut Geliyor mu?)
    if poll_obj.poll(0):
        ch = sys.stdin.read(1)
        if ch == "\n":
            try:
                for p in buffer.strip().split(";"):
                    if ":" in p:
                        k, val = p.split(":")
                        if k == "V": v_cmd = float(val)
                        elif k == "S": s_cmd = float(val)
            except: pass
            buffer = ""
        else: buffer += ch

    # Sensör Verileri
    dist = get_distance_filt()
    pitch_val = update_pitch()

    # Engel ve Yokuş Kontrolü
    v_limited = 0.0 if dist < STOP_DIST else (v_cmd * 0.4 if dist < SLOW_DIST else v_cmd)
    
    if v_limited == 0:
        v_safe = 0.0
    else:
        hill_boost = clamp(pitch_val * HILL_SIGN * HILL_K, -0.3, 0.3)
        v_safe = clamp(v_limited + hill_boost, -0.8, 0.8)

    # Motor ve Servo Sürüşü
    set_motor(v_safe)
    set_steering(s_cmd)

    # Log
    if utime.ticks_diff(now, last_print) > 200:
        print("Mesafe:{:.1f}cm | Egim:{:.1f} | Hiz:{:.2f}".format(dist, pitch_val, v_safe))
        last_print = now
    
    utime.sleep_ms(20)
