import socket
import time
import json
import os
from datetime import datetime

# ---------------------------
# Utility / Control Functions
# ---------------------------

def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))

def expo(x: float, k: float = 0.4) -> float:
    """
    x: -1..+1
    k: 0..1  (0=lineer, 1=çok yumuşak)
    """
    x = clamp(x)
    return (1.0 - k) * x + k * (x ** 3)

def approach(current: float, target: float, max_delta: float):
    if target > current:
        return min(current + max_delta, target)
    else:
        return max(current - max_delta, target)

def steer_limit_by_speed(steer: float, throttle: float, min_lim: float = 0.45, max_lim: float = 1.0):
    """
    Hız arttıkça steer limitini düşürür.
    |throttle|=0  -> limit=max_lim
    |throttle|=1  -> limit=min_lim
    """
    t = clamp(abs(throttle), 0.0, 1.0)
    lim = max_lim - (max_lim - min_lim) * t
    return clamp(steer, -lim, +lim)

def mix_differential(throttle: float, steer: float):
    """
    Diferansiyel sürüş mixing (normalize).
    throttle: -1..+1
    steer:    -1..+1
    return: (left, right) -1..+1
    """
    throttle = clamp(throttle)
    steer = clamp(steer)

    max_mag = max(1.0, abs(throttle) + abs(steer))
    left = (throttle + steer) / max_mag
    right = (throttle - steer) / max_mag
    return clamp(left), clamp(right)

def to_motor_command(x: float, deadband: float = 0.02):
    """
    x: -1..+1
    return: (direction, pwm_0_255)
    """
    x = clamp(x)

    if abs(x) < deadband:
        return "STOP", 0

    direction = "FWD" if x > 0 else "REV"
    pwm = int(abs(x) * 255)
    pwm = max(0, min(255, pwm))
    return direction, pwm

# ---------------------------
# Network / Protocol Settings
# ---------------------------

# Command RX (PC -> Vehicle)
CMD_HOST = "127.0.0.1"
CMD_PORT = 5005

# Telemetry TX (Vehicle -> PC)
TELEM_IP = "127.0.0.1"
TELEM_PORT = 6006
TELEM_HZ = 10
TELEM_DT = 1.0 / TELEM_HZ

# Safety
TIMEOUT_S = 0.35   # 200ms biraz agresif oluyor; 0.35-0.5 daha stabil
HARD_STOP_ON_TIMEOUT = False  # True yaparsan timeout anında actual=0

# Control dynamics
RAMP_RATE = 2.5     # 0->1 yaklaşık 0.4 s

# Input shaping
EXPO_THR = 0.35
EXPO_STR = 0.60

# Protocol versioning
PROTO_CMD_TYPE = "cmd"
PROTO_TELEM_TYPE = "telem"
PROTO_VER = 1

# ---------------------------
# Logging (TXT)
# ---------------------------

LOG_DIR = "logs"
os.makedirs(LOG_DIR, exist_ok=True)

log_filename = datetime.now().strftime("vehicle_log_%Y%m%d_%H%M%S.txt")
log_path = os.path.join(LOG_DIR, log_filename)
log_file = open(log_path, "w", encoding="utf-8")
log_file.write(
    "t_unix | state | seq | age_ms | estop_latched | thr | str | "
    "tgt_L | tgt_R | act_L | act_R | L_dir L_pwm | R_dir R_pwm\n"
)
log_file.flush()

# ---------------------------
# Sockets
# ---------------------------

cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.bind((CMD_HOST, CMD_PORT))
cmd_sock.settimeout(0.05)  # 50 ms

telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ---------------------------
# State Variables
# ---------------------------

state = "SAFE"

# Monotonic clock for age/timeout (daha sağlam)
now_s = time.monotonic
last_cmd_s = None

# last received command fields (for telemetry)
last_seq = -1
last_throttle = 0.0
last_steer = 0.0
last_estop = 0
last_reset = 0

# Robustness: UDP out-of-order drop
last_seq_seen = -1

# E-STOP latch
estop_latched = False

# control targets & ramped actuals
target_left = 0.0
target_right = 0.0
actual_left = 0.0
actual_right = 0.0

# timers (use monotonic too)
last_print_s = now_s()
last_telem_s = now_s()
prev_s = now_s()

print(f"[VEHICLE] CMD UDP dinleniyor: {CMD_HOST}:{CMD_PORT}")
print(f"[VEHICLE] TELEM UDP gönderiyor: {TELEM_IP}:{TELEM_PORT} ({TELEM_HZ} Hz)")
print(f"[VEHICLE] TXT log: {log_path}")

try:
    while True:
        t = now_s()

        # dt for ramp
        dt = t - prev_s
        prev_s = t
        dt = clamp(dt, 0.0, 0.1)

        # 1) Receive command
        try:
            data, addr = cmd_sock.recvfrom(2048)
            msg = json.loads(data.decode("utf-8"))

            # Protocol validation
            if msg.get("type") != PROTO_CMD_TYPE or int(msg.get("ver", 0)) != PROTO_VER:
                state = "FAULT_BAD_PACKET"
                target_left = 0.0
                target_right = 0.0
            else:
                seq = int(msg.get("seq", -1))

                # out-of-order / replay drop
                if seq != -1 and seq <= last_seq_seen:
                    # Eski paket: ignore (state'i bozma)
                    pass
                else:
                    if seq != -1:
                        last_seq_seen = seq

                    last_cmd_s = t

                    last_seq = seq
                    last_throttle = float(msg.get("throttle", 0.0))
                    last_steer = float(msg.get("steer", 0.0))
                    last_estop = int(msg.get("estop", 0))
                    last_reset = int(msg.get("reset", 0))  # client göndermiyorsa 0 kalır

                    # E-STOP latch logic
                    if last_estop == 1:
                        estop_latched = True
                    if last_reset == 1:
                        estop_latched = False

                    if estop_latched:
                        state = "FAULT_ESTOP_LATCHED"
                        target_left = 0.0
                        target_right = 0.0
                    else:
                        state = "ACTIVE"

                        # shaping: expo + speed-based steer limit
                        thr = expo(last_throttle, EXPO_THR)
                        str_ = expo(last_steer, EXPO_STR)
                        str_ = steer_limit_by_speed(str_, thr, min_lim=0.45, max_lim=1.0)

                        target_left, target_right = mix_differential(thr, str_)

        except socket.timeout:
            pass
        except json.JSONDecodeError:
            state = "FAULT_BAD_PACKET"
            target_left = 0.0
            target_right = 0.0

        # 2) Failsafe: timeout
        age_ms = None if last_cmd_s is None else int((t - last_cmd_s) * 1000)
        if last_cmd_s is not None and (t - last_cmd_s) > TIMEOUT_S:
            state = "FAULT_TIMEOUT"
            target_left = 0.0
            target_right = 0.0
            if HARD_STOP_ON_TIMEOUT:
                actual_left = 0.0
                actual_right = 0.0

        # 3) Ramp actual commands toward target
        max_delta = RAMP_RATE * dt
        actual_left = approach(actual_left, target_left, max_delta)
        actual_right = approach(actual_right, target_right, max_delta)

        # 4) Motor command mapping (direction + PWM)
        l_dir, l_pwm = to_motor_command(actual_left)
        r_dir, r_pwm = to_motor_command(actual_right)

        # 5) Console log (10 Hz)
        if t - last_print_s >= 0.1:
            print(f"[MOTOR_CMD] L:{l_dir} {l_pwm:3d} | R:{r_dir} {r_pwm:3d} | state={state} | age_ms={age_ms}")
            last_print_s = t

        # 6) Telemetry + TXT log (10 Hz)
        if t - last_telem_s >= TELEM_DT:
            # Telemetry JSON
            telem = {
                "type": PROTO_TELEM_TYPE,
                "ver": PROTO_VER,
                "t_ms": int(time.time() * 1000),  # unix ms (dashboard için)
                "state": state,
                "seq": last_seq,
                "link_age_ms": age_ms,
                "estop_latched": int(estop_latched),
                "throttle": float(last_throttle),
                "steer": float(last_steer),
                "target_left": float(target_left),
                "target_right": float(target_right),
                "actual_left": float(actual_left),
                "actual_right": float(actual_right),
                "L_dir": l_dir,
                "L_pwm": int(l_pwm),
                "R_dir": r_dir,
                "R_pwm": int(r_pwm),
            }

            try:
                telem_sock.sendto(json.dumps(telem).encode("utf-8"), (TELEM_IP, TELEM_PORT))
            except Exception as e:
                # telemetri gönderimi patlarsa kontrol döngüsü ölmesin
                # print istersen aç
                # print("TELEM SEND ERROR:", repr(e))
                pass

            # TXT LOG (okunabilir)
            now_unix = time.time()
            log_line = (
                f"{now_unix:.3f} | {state:16s} | {last_seq:5d} | "
                f"{'' if age_ms is None else age_ms:>6} | {int(estop_latched):>12d} | "
                f"{last_throttle:+.2f} | {last_steer:+.2f} | "
                f"{target_left:+.2f} | {target_right:+.2f} | "
                f"{actual_left:+.2f} | {actual_right:+.2f} | "
                f"{l_dir:4s} {l_pwm:3d} | {r_dir:4s} {r_pwm:3d}\n"
            )
            log_file.write(log_line)
            log_file.flush()

            last_telem_s = t

finally:
    log_file.close()
    print("[LOG] TXT log dosyası kapatıldı.")
