import machine, utime, sys, math
from machine import I2C, Pin, PWM, UART

# ==================== AYARLAR ====================
SOL_SERVO_PIN = 20
SAG_SERVO_PIN = 21
SERVO_CENTER = 90
SERVO_RANGE = 45

# --- BTS7960 PIN TANIMLAMALARI (4 ADET) ---
# Her motor için RPWM ve LPWM pinleri.
# Not: Pin çakışması olmaması için boş pinler seçildi.
M1_RPWM = 6; M1_LPWM = 7 # Ön Sol
M2_RPWM = 8; M2_LPWM = 9 # Ön Sağ
M3_RPWM = 10; M3_LPWM = 11 # Arka Sol
M4_RPWM = 12; M4_LPWM = 13 # Arka Sağ
MOTOR_FREQ = 15000

TRIG_PIN = 14
ECHO_PIN = 15

I2C_ID = 0
SCL_PIN = 5
SDA_PIN = 4
MPU_ADDR = 0x68

HILL_SIGN = -1.0

PITCH_DB = 3.0

PITCH_MAX = 30.0

HILL_K = 0.05

BOOST_MAX = 0.6

HILL_STEER_SOFT_DEG = 15.0
HILL_STEER_SCALE = 0.6
ALPHA_FILTER = 0.98

SLOW_DIST = 40.0
STOP_DIST = 15.0

# ==================== DONANIM BAŞLATMA ====================

# BTS7960 Motor Sınıfı
class BTS7960:
  def _init_(self, r_pin, l_pin, freq):
    self.rpwm = PWM(Pin(r_pin))
    self.lpwm = PWM(Pin(l_pin))
    self.rpwm.freq(freq)
    self.lpwm.freq(freq)

  def drive(self, speed):
    duty = int(abs(speed) * 65535)
    if speed > 0:
      self.rpwm.duty_u16(duty); self.lpwm.duty_u16(0)
    elif speed < 0:
      self.rpwm.duty_u16(0); self.lpwm.duty_u16(duty)
    else:
      self.rpwm.duty_u16(0); self.lpwm.duty_u16(0)

# 4 Motoru Başlatma
motorlar = [
  BTS7960(M1_RPWM, M1_LPWM, MOTOR_FREQ),
  BTS7960(M2_RPWM, M2_LPWM, MOTOR_FREQ),
  BTS7960(M3_RPWM, M3_LPWM, MOTOR_FREQ),
  BTS7960(M4_RPWM, M4_LPWM, MOTOR_FREQ)
]

sol_servo = PWM(Pin(SOL_SERVO_PIN))
sol_servo.freq(50)
sag_servo = PWM(Pin(SAG_SERVO_PIN))
sag_servo.freq(50)

trig = Pin(TRIG_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)

i2c = I2C(I2C_ID, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000)
try:
  i2c.writeto_mem(MPU_ADDR, 0x6B, b"\x00")
  utime.sleep_ms(50)
except:
  print("MPU6050 Bağlantı Hatası!")
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

# ==================== YARDIMCI FONKSİYONLAR ====================
def clamp(x, lo, hi):
  return max(lo, min(x, hi))


def set_motor(speed):
  speed = clamp(speed, -1.0, 1.0)
  for m in motorlar:
    m.drive(speed)

def set_steering(steer):
  steer = clamp(steer, -1.0, 1.0)
  sol_aci = SERVO_CENTER + (steer * SERVO_RANGE)
  sol_duty = int(1638 + (sol_aci / 180.0) * (8192 - 1638))
  sol_servo.duty_u16(sol_duty)
 
  sag_aci = SERVO_CENTER - (steer * SERVO_RANGE)
  sag_duty = int(1638 + (sag_aci / 180.0) * (8192 - 1638))
  sag_servo.duty_u16(sag_duty)

def get_distance_raw():
  trig.low(); utime.sleep_us(2)
  trig.high(); utime.sleep_us(10); trig.low()
  t0 = utime.ticks_us()
  while echo.value() == 0:
    if utime.ticks_diff(utime.ticks_us(), t0) > 20000: return None
  t1 = utime.ticks_us()
  while echo.value() == 1:
    if utime.ticks_diff(utime.ticks_us(), t1) > 20000: return None
  return (utime.ticks_diff(utime.ticks_us(), t1) * 0.0343) / 2.0
_d_ema = None
def get_distance_filt():
  global _d_ema
  d = get_distance_raw()
  if d is None or d > 400:
    d = 400.0
 
  if d < 1: return _d_ema

  _d_ema = d if _d_ema is None else (0.7 * _d_ema + 0.3 * d)
  return _d_ema

def read_i16(reg):
  try:
    data = i2c.readfrom_mem(MPU_ADDR, reg, 2)
    val = (data[0] << 8) | data[1]
    return val - 65536 if val > 32768 else val
  except: return 0

roll = 0.0; pitch = 0.0; t_prev_us = utime.ticks_us()
def update_pitch():
  global roll, pitch, t_prev_us
  t = utime.ticks_us()
  dt = max(utime.ticks_diff(t, t_prev_us) / 1000000.0, 0.001)
  t_prev_us = t
  ax = read_i16(0x3B) / 16384.0
  ay = read_i16(0x3D) / 16384.0
  az = read_i16(0x3F) / 16384.0
  gy = read_i16(0x45) / 131.0
  pitch_acc = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
  pitch = ALPHA_FILTER * (pitch + gy * dt) + (1.0 - ALPHA_FILTER) * pitch_acc
  return pitch
def pi_komutlarini_oku():
    global v_cmd, s_cmd, last_cmd_ms
    if uart.any():
        try:
            raw_data = uart.readline()
            if raw_data:
                line = raw_data.decode('utf-8').strip()
                if "V:" in line or "S:" in line:
                    parts = line.split(";")
                    for part in parts:
                        if part.startswith("V:"):
                            v_cmd = float(part.split(":")[1])
                        elif part.startswith("S:"):
                            s_cmd = float(part.split(":")[1])
                    # Yeni komut geldiğinde güvenlik/zamanlayıcı değişkenini güncelle
                    last_cmd_ms = utime.ticks_ms() 
        except Exception:
            pass

def pi_telemetri_gonder(mesafe_cm):
    if mesafe_cm is not None:
        mesaj = "DIST_CM={:.1f};\n".format(mesafe_cm)
        uart.write(mesaj.encode('utf-8'))
# ==================== ANA DÖNGÜ ====================
v_cmd = 0.0
s_cmd = 0.0
v_safe = 0.0
last_ultra_ms = 0
last_print_ms = 0
last_cmd_ms = utime.ticks_ms()

print("SİSTEM HAZIR. 4 Motor ve Çift Servo Devrede...")

while True:
  now = utime.ticks_ms()
  pi_komutlarini_oku()
  if utime.ticks_diff(now, last_cmd_ms) > 1000:
        v_cmd = 0.0
        s_cmd = 0.0
  pitch_val = update_pitch()
#1 saniye boyunca komut gelmezse durdur,teste göre kaldırabilriz
  if utime.ticks_diff(now, last_ultra_ms) >= 100:
    dist = get_distance_filt()
    last_ultra_ms = now
  else:
    dist = _d_ema

  v_target = v_cmd
  engel_kritik = False

  if dist is not None:
    if dist < SLOW_DIST:
      if dist < STOP_DIST:
        if v_cmd > 0:
          v_target = 0.0
          engel_kritik = True
        else:
          v_target = v_cmd
      else:
        if v_cmd > 0:
          v_target = v_cmd * 0.4
        else:
          v_target = v_cmd

  p_eff = pitch_val * HILL_SIGN
  if engel_kritik:
    v_safe = 0.0
  else:
    hill = 0.0
    if abs(p_eff) > PITCH_DB:
      p_eff_clamped = clamp(p_eff, -PITCH_MAX, PITCH_MAX)
      hill = p_eff_clamped * HILL_K
    v_safe = clamp(v_target + hill, -1.0, 1.0)

  s_safe = s_cmd * HILL_STEER_SCALE if abs(pitch_val) > HILL_STEER_SOFT_DEG else s_cmd
 
 
  set_motor(v_safe)
  set_steering(s_safe)

  if utime.ticks_diff(now, last_print_ms) >= 100:
    d_send = "%.1f" % dist if dist is not None else "Hata"
    age = utime.ticks_diff(now, last_cmd_ms)
    print("D: {} | P: {:.1f} | S: {:.2f} | V: {:.2f} | age: {}".format(
      d_send, pitch_val, s_safe, v_safe, age))
    if dist is not None:
           pi_telemetri_gonder(dist)
    last_print_ms = now
 
  utime.sleep_ms(10)