#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import cv2
import numpy as np
import requests
from ultralytics import YOLO

# === 설정 ===
ESP_IP      = "192.168.0.174"
STREAM_URL  = f"http://{ESP_IP}:81/stream"
GPIO_URL    = f"http://{ESP_IP}/gpio"
CONF_THRESH = 0.3
PIN_LIST    = [4, 16]

# YOLO 모델 로드
model = YOLO('fire_model.pt')

# 마지막으로 보낸 값 기억 (중복 호출 방지)
last_vals = {pin: None for pin in PIN_LIST}

def set_gpio(pin, val):
    """HTTP GET 으로 ESP32 /gpio?pin=&val= 호출"""
    try:
        r = requests.get(GPIO_URL,
                         params={"pin": pin, "val": val},
                         timeout=1)
        if r.status_code != 200:
            print(f"[!] GPIO{pin} error: {r.status_code}")
        else:
            print(f"▶ GPIO{pin} {'HIGH' if val else 'LOW'}")
    except Exception as e:
        print(f"[!] GPIO request failed:", e)

def main():
    # 비디오 스트림 열기
    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        print("스트림 열기 실패:", STREAM_URL)
        return

    print("ESC 누르면 종료합니다.")
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w = frame.shape[:2]
        # YOLO 추론
        res = model(frame, conf=CONF_THRESH, imgsz=320)[0]

        # 화면 중앙 1/3 영역 안에 박스 중심이 하나라도 있으면 True
        center_in = False
        for box in res.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            if w//3 <= cx <= 2*w//3 and h//3 <= cy <= 2*h//3:
                center_in = True
            # 사각형 그리기
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,0,255), 2)

        # 검출 여부에 따라 값 결정
        val = 1 if center_in else 0

        # GPIO 4,16 에만 상태가 바뀔 때만 호출
        for pin in PIN_LIST:
            if last_vals[pin] != val:
                set_gpio(pin, val)
                last_vals[pin] = val

        # 프레임 보여주기
        cv2.imshow("Fire Detection", frame)
        if cv2.waitKey(1) == 27:  # ESC
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
