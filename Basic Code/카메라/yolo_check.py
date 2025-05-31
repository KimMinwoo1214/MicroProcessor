# 0) YOLOv8을 위한 ultralytics 설치 필요
# pip install ultralytics

from ultralytics import YOLO
import cv2

# 1) YOLOv8 모델 로드
model = YOLO('fire_model.pt')  # 학습된 커스텀 모델 경로

# 2) 영상 소스 열기 (0 = 웹캠)
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("카메라를 열 수 없습니다.")

CONF_THRESH = 0.8  # 최소 confidence threshold

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 3) YOLOv8 추론 (이미지 한 장 → 결과 반환)
    results = model.predict(frame, conf=CONF_THRESH, verbose=False)

    # 4) 결과 시각화
    for result in results:
        boxes = result.boxes
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            label = f"{model.names[cls_id]} {conf:.2f}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

    # 5) 화면에 출력
    cv2.imshow('YOLOv8 Fire Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
