import socket
import time
import json
import msvcrt  # Windows non-blocking keyboard

# Vehicle command endpoint (for real Pi: HOST="PI_IP")
HOST = "127.0.0.1"
PORT = 5005

SEND_HZ = 50
DT = 1.0 / SEND_HZ

# Protocol versioning
PROTO_CMD_TYPE = "cmd"
PROTO_VER = 1

def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))

throttle = 0.0
steer = 0.0
estop = 0
seq = 0

print("[PC] WASD: sürüş | X: sıfırla | Q: E-STOP | E: E-STOP reset | ESC: çıkış")
print("[PC] Not: Program çalıştığı sürece komut gönderir. Kapatınca araç 200ms sonra timeout STOP yapar.")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
next_send = time.time()

while True:
    # Keyboard input
    if msvcrt.kbhit():
        key = msvcrt.getch()

        if key == b'\x1b':  # ESC
            print("[PC] Çıkış.")
            break

        if key in (b'w', b'W'):
            throttle += 0.10
        elif key in (b's', b'S'):
            throttle -= 0.10
        elif key in (b'a', b'A'):
            steer -= 0.10
        elif key in (b'd', b'D'):
            steer += 0.10
        elif key in (b'x', b'X'):
            throttle, steer = 0.0, 0.0
        elif key in (b'q', b'Q'):
            estop = 1
        elif key in (b'e', b'E'):
            estop = 0

        throttle = clamp(throttle)
        steer = clamp(steer)

        print(f"[PC] throttle={throttle:+.2f} steer={steer:+.2f} estop={estop}")

    # Send at 50 Hz
    now = time.time()
    if now >= next_send:
        seq += 1
        msg = {
            "type": PROTO_CMD_TYPE,
            "ver": PROTO_VER,
            "seq": seq,
            "t_ms": int(now * 1000),
            "throttle": throttle,
            "steer": steer,
            "estop": estop
        }
        sock.sendto(json.dumps(msg).encode("utf-8"), (HOST, PORT))
        next_send += DT

    time.sleep(0.001)