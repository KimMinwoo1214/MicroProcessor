#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import requests
import time
from ultralytics import YOLO

# — 설정값 —
ESP_IP       = "192.168.0.174"
STREAM_URL   = f"http://{ESP_IP}/stream"
GPIO_URL     = f"http://{ESP_IP}/gpio"
CONF_THRESH  = 0.3

# 제어할 GPIO 핀 두 개
PIN_LIST     = [4, 16]

# 재시도 관련 설정
RETRY_COUNT  = 3
RETRY_DELAY  = 0.5
TIMEOUT_SEC  = 2.0

# YOLOv8 모델 로드
# → imgsz를 320으로 낮춰서 경량 모델처럼 동작
model = YOLO('fire_model.pt')

# 마지막으로 보낸 GPIO 상태를 추적
last_vals = {pin: None for pin in PIN_LIST}

def send_gpio(pin: int, val: int):
    params = {"pin": pin, "val": val}
    for attempt in range(1, RETRY_COUNT + 1):
        try:
            resp = requests.get(GPIO_URL, params=params, timeout=TIMEOUT_SEC)
            if resp.status_code == 200:
                return
            else:
                print(f"⚠ GPIO 요청 비정상 응답({attempt}/{RETRY_COUNT}): "
                      f"{resp.status_code}, 내용: {resp.text}")
                return
        except requests.exceptions.Timeout:
            if attempt < RETRY_COUNT:
                time.sleep(RETRY_DELAY)
            else:
                print(f"⚠ GPIO 요청 Timeout ({GPIO_URL}?pin={pin}&val={val})")
        except requests.exceptions.RequestException as e:
            if attempt < RETRY_COUNT:
                time.sleep(RETRY_DELAY)
            else:
                print(f"⚠ GPIO 요청 실패({attempt}/{RETRY_COUNT}): {e}")
        break

def send_gpio_to_all(val: int):
    for pin in PIN_LIST:
        if last_vals[pin] != val:
            send_gpio(pin, val)
            last_vals[pin] = val
            print(f"  ▶ GPIO{pin} {'HIGH' if val == 1 else 'LOW'}")

def main():
    global last_vals

    # 1) VideoCapture 객체 생성 (MJPEG 스트림)
    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        print(f"⚠ 스트림 열기 실패: {STREAM_URL}")
        return

    # 2) 프레임 크기 한 번만 읽어오기 (ESP32가 QVGA→QQVGA로 설정되었으면 160×120)
    orig_w  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    orig_h  = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"▶ 원본 스트림 해상도: {orig_w}×{orig_h}")

    # 3) YOLO 입력 전용 크기 (작게 잡으면 더 빠름)
    DETECT_SIZE = 320  # 기본 416 → 320으로 줄임

    # 중앙 영역 계산 (원본 해상도 기준)
    x_min = orig_w / 3
    x_max = orig_w * 2/3
    y_min = orig_h / 3
    y_max = orig_h * 2/3
    print(f"▶ 중앙 영역 (원본 기준): x ∈ [{x_min:.0f}, {x_max:.0f}], y ∈ [{y_min:.0f}, {y_max:.0f}]")

    cv2.namedWindow("Fire Detection (Resized)", cv2.WINDOW_NORMAL)
    print("▶ 스트림 수신 및 화재 검출 시작")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            time.sleep(0.1)
            continue

        # 4) 화면 전체(원본 QVGA/QQVGA) 디스플레이 (선택사항)
        display_img = frame.copy()
        cv2.rectangle(display_img, (int(x_min), int(y_min)), (int(x_max), int(y_max)),
                      (0, 255, 0), 2)

        # 5) YOLO용 이미지 생성: 원본에서 양쪽 여백 고려해서 비율 유지하며 리사이즈
        #    → 만약 원본 가로세로 비율이 다르면 padding을 넣거나 그냥 resize(일그러져도 됨)
        detect_img = cv2.resize(frame, (DETECT_SIZE, DETECT_SIZE))
        #    model() 호출 시 imgsz=DETECT_SIZE로 맞춰야 내부적으로 추가 리사이즈가 안 일어남
        results = model(detect_img, conf=CONF_THRESH, imgsz=DETECT_SIZE)[0]

        # 6) YOLO 결과를 원본 좌표로 다시 매핑
        #    - detect_img 는 (DETECT_SIZE×DETECT_SIZE)지만, 원본은 (orig_w×orig_h)이므로
        #    - box 좌표를 비율로 환산해서 원본 크기로 변환해야 함
        scale_x = orig_w / DETECT_SIZE
        scale_y = orig_h / DETECT_SIZE

        center_in_mid = False
        for box in results.boxes:
            # detect_img 기준 box 좌표
            x1_d, y1_d, x2_d, y2_d = map(int, box.xyxy[0])

            # 원본(scale_x, scale_y)로 변환
            x1 = int(x1_d * scale_x)
            y1 = int(y1_d * scale_y)
            x2 = int(x2_d * scale_x)
            y2 = int(y2_d * scale_y)

            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2

            # 중앙 영역 판별 (원본 기준)
            if x_min <= cx <= x_max and y_min <= cy <= y_max:
                center_in_mid = True

            # 시각화: 원본 이미지에 바운딩 박스 및 중심점 표시
            cv2.rectangle(display_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            label = f"{model.names[int(box.cls[0])]} {float(box.conf[0]):.2f}"
            cv2.putText(display_img, label, (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            cv2.circle(display_img, (int(cx), int(cy)), 5, (0, 0, 255), -1)

        # 7) GPIO 제어
        val = 1 if center_in_mid else 0
        send_gpio_to_all(val)

        # 8) 결과 화면 출력
        cv2.imshow("Fire Detection (Resized)", display_img)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC 누르면 종료
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
