import socket
import json
import time

HOST = "127.0.0.1"
PORT = 6006

# Protocol versioning
PROTO_TELEM_TYPE = "telem"
PROTO_VER = 1

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))

print(f"[TELEM] Dinleniyor: {HOST}:{PORT}")

while True:
    data, addr = sock.recvfrom(4096)
    try:
        msg = json.loads(data.decode("utf-8"))

        # accept only telemetry packets v1
        if msg.get("type") != PROTO_TELEM_TYPE or int(msg.get("ver", 0)) != PROTO_VER:
            continue

        t = time.strftime("%H:%M:%S")
        state = msg.get("state")

        alarm = ""
        if state in ("FAULT_ESTOP", "FAULT_TIMEOUT", "FAULT_BAD_PACKET"):
            alarm = "  <<< ALARM"

        age_ms = msg.get("link_age_ms")
        age_ms_str = "None" if age_ms is None else str(age_ms)

        print(
            f"[{t}] state={state} seq={msg.get('seq')} age_ms={age_ms_str} "
            f"thr={msg.get('throttle'):+.2f} str={msg.get('steer'):+.2f} | "
            f"L={msg.get('actual_left'):+.2f} R={msg.get('actual_right'):+.2f} "
            f"PWM(L,R)=({msg.get('L_pwm')},{msg.get('R_pwm')})"
            f"{alarm}"
        )

    except Exception as e:
        print(f"[TELEM] Hatalı paket: {e} raw={data[:80]}")