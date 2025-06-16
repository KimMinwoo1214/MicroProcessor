import cv2
import requests
import numpy as np

# — ESP SoftAP 기본 IP 및 포트 —
STREAM_URL = "http://192.168.0.174/snapshot"
# GPIO_URL   = "http://172.16.66.99/gpio?pin=4&val={}"
# PIN        = 4

def set_gpio(val):
    try:
        requests.get(GPIO_URL.format(val), timeout=0.5)
    except:
        pass

# MJPEG 스트림 연결
resp = requests.get(STREAM_URL, stream=True)
if resp.status_code != 200:
    raise RuntimeError(f"Stream 연결 실패: HTTP {resp.status_code}")

buf = b""
while True:
    for chunk in resp.iter_content(chunk_size=1024):
        buf += chunk
        start = buf.find(b'\xff\xd8')
        end   = buf.find(b'\xff\xd9')
        if start != -1 and end != -1 and end > start:
            jpg = buf[start:end+2]
            buf = buf[end+2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8),
                                  cv2.IMREAD_COLOR)
            if frame is None:
                continue

            cv2.imshow("ESP32-CAM (AP Mode)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('h'):
                set_gpio(1)    # pin 4 HIGH
            elif key == ord('l'):
                set_gpio(0)    # pin 4 LOW
            elif key == 27:    # ESC
                resp.close()
                cv2.destroyAllWindows()
                exit(0)
