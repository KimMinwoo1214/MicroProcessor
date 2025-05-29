import cv2
import json
import numpy as np
import websocket
import ssl
import socket
import time
from ultralytics import YOLO

# — 설정값 —
WS_URL      = "ws://172.16.66.99/ws"
CONF_THRESH = 0.7
IMG_SIZE    = 416
PIN         = 4

# YOLOv8 모델 로드
model = YOLO('fire_model.pt')

# 최종 GPIO 상태 추적
last_val = None

def on_open(ws):
    print("▶ WebSocket 연결 성공, 스트림 수신 시작")
    # TCP Nagle 비활성화 & Keep-Alive 켜기
    sock = ws.sock
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)

def on_message(ws, message):
    global last_val
    if isinstance(message, str):
        return

    # JPEG 바이너리 디코딩
    img = cv2.imdecode(np.frombuffer(message, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return

    # YOLO 추론
    res = model(img, conf=CONF_THRESH, imgsz=IMG_SIZE)[0]

    # 상태 변화 시에만 GPIO 전송
    val = 1 if len(res.boxes) > 0 else 0
    if val != last_val:
        ws.send(json.dumps({"pin": PIN, "val": val}))
        last_val = val

    # 바운딩박스 & 레이블 그리기
    for box in res.boxes:
        conf = float(box.conf[0])
        if conf < CONF_THRESH:
            continue
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cls = int(box.cls[0])
        label = f"{model.names[cls]} {conf:.2f}"
        cv2.rectangle(img, (x1, y1), (x2, y2), (0,255,0), 2)
        cv2.putText(img, label, (x1, y1-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

    # 화면에 출력
    cv2.imshow("Fire Detection", img)
    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        ws.close()

def on_pong(ws, data):
    pass

def on_close(ws, close_status_code, close_msg):
    print(f"◀ WebSocket 연결 종료 (code={close_status_code}, msg={close_msg})")

def on_error(ws, error):
    print("⚠ WebSocket 에러:", error)

def start_ws():
    while True:
        ws_app = websocket.WebSocketApp(
            WS_URL,
            on_open=on_open,
            on_message=on_message,
            on_error=on_error,
            on_close=on_close,
            on_pong=on_pong
        )
        # ping_interval: 10초마다 Ping, ping_timeout: 5초 내 Pong 없으면 타임아웃
        ws_app.run_forever(
            ping_interval=10,
            ping_timeout=5,
            sslopt={"cert_reqs": ssl.CERT_NONE}
        )
        print(">>> 재접속 시도 5초 후 다시 연결합니다...")
        time.sleep(5)

if __name__ == "__main__":
    start_ws()
