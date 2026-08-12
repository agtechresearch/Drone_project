# PinkyPro 과수원 미션

라즈베리파이4(PinkyPro) 위에서 라인트레이싱 + 아루코 마커 분기 +
YOLO 사과 카운트를 함께 돌리는 미션 코드입니다.
**현장에서 수치는 `config.yaml`만 수정**하면 됩니다.

## 1. 파일 구성

```
orchard_mission/
├── config.yaml             ← 모든 수치 (현장에서 여기만 만짐)
├── main.py                 ← 미션 실행 진입점
├── marker_detector.py      ← 아루코 마커 탐지 (백그라운드 스레드)
├── apple_counter.py        ← YOLO 사과 카운트 (정지 시에만)
├── line_tracer.py          ← IR 3센서 라인트레이싱
├── calibrate_and_test.py   ← 캘리브레이션 / 마커 테스트 유틸
├── models/
│   └── apple_yolov8n.onnx  ← (직접 준비) 사과 탐지 모델
└── logs/                   ← 디버그 이미지 / 미션 로그 자동 생성
```

## 2. 사전 준비

### (1) 카메라 캘리브레이션 (권장 - 마커 거리 측정 정확해짐)
1. 8x6 체커보드(한 칸 25mm) 인쇄
2. `calib_img/` 폴더 만들고 다양한 각도로 10장 이상 촬영
3. `python3 calibrate_and_test.py calib` 실행 → `calib.npz` 생성

> 캘리브레이션을 못 하면 `approach_pixel_ratio`(마커 픽셀 크기 비율)로
> 대신 도착 판정합니다. `calibrate_and_test.py test`로 값을 확인하며 튜닝하세요.

### (2) YOLO 모델 준비
사과 모형 50~200장에 라벨링 → YOLOv8n 학습 → ONNX export 권장:
```bash
yolo train data=apple.yaml model=yolov8n.pt imgsz=320 epochs=80
yolo export model=runs/detect/train/weights/best.pt format=onnx imgsz=320
```
파일을 `models/apple_yolov8n.onnx`에 둡니다. 모델이 없어도 미션은 돌지만 카운트가 0이 됩니다.

### (3) 마커 ID 인쇄
config.yaml의 `marker.ids` 매핑대로 5x5(`DICT_5X5_100`) 5cm 마커 인쇄하여 부착.

## 3. 실행
```bash
python3 main.py
# 또는 다른 config로:
python3 main.py my_config.yaml
```

## 4. 현장 튜닝 체크리스트

| 증상 | 만질 곳 |
|---|---|
| 마커를 너무 자주 놓침 | `marker.detect_every_n_frames` ↓, `confirm_streak` ↓ |
| 한두 프레임 오탐으로 잘못 분기 | `confirm_streak` ↑ (3 → 4~5) |
| 마커 너무 가까이 가서야 멈춤 | `approach_distance_m` ↑ 또는 `approach_pixel_ratio` ↓ |
| 멀리서 미리 멈춤 | 위 두 값 반대로 |
| 라인을 자꾸 놓침 | `linetrace.threshold` 조정 (자료 기준 흰<500, 검>3000) |
| 너무 빨라 마커 못 봄 | `linetrace.base_speed` ↓ (30 → 20) |
| 회전이 모자라거나 과함 | `turning.turn_90_duration_s` 조정 (직접 측정) |
| YOLO 너무 느림 | `yolo.imgsz` ↓ (320 → 256), `vote_frames` ↓ |
| YOLO 오탐 많음 | `yolo.conf_threshold` ↑ (0.4 → 0.5~0.6) |
| 사과 개수가 매번 흔들림 | `yolo.vote_frames` ↑ (중앙값으로 결정) |

## 5. 라즈베리파이4 성능 고려사항 (코드에서 어떻게 대응했나)

| 부담 | 대응 |
|---|---|
| 카메라 캡처와 마커 탐지로 메인 루프 막힘 | `MarkerDetector`를 별도 스레드, 메인은 `consume_confirmed()`만 호출 |
| 매 프레임 마커 검출 비용 | 그레이스케일 + 다운스케일(0.5) + `detect_every_n_frames` |
| 1프레임 오탐 분기 | `confirm_streak` 누적 후에만 확정 |
| YOLO가 주행 중 돌면 멈춤 | 카운트는 **정지 후**에만 실행 (count_apples 행동) |
| YOLO 결과 한 프레임이 흔들림 | `vote_frames` 중앙값 사용 |
| 카메라 객체 동시 접근 | `Camera` 인스턴스 1개를 두 모듈이 공유 |

## 6. 미션 시퀀스 (config.yaml `sequence`)

맵(과수원_맵_표지판_배치_설명용.png)을 기준으로:

```
START → 마커1(좌회전) → 마커3(과수원A, 카운트) → U턴
      → 마커1(우회전 메인복귀) → 마커2(좌회전) → 마커6(과수원B, 카운트) → U턴
      → 마커2(우회전 복귀) → 마커4(우회전) → 마커9(과수원C, 카운트) → U턴
      → 마커4(좌회전 복귀) → 마커5(좌회전) → 마커0(하차장, 사과-1 저장) → U턴
      → 마커5(우회전 복귀) → 마커10(도착, 종료)
```

회전 방향이 실제 맵과 다르면 **config.yaml의 `sequence`에서 `turn_left`↔`turn_right`만 바꾸면 됩니다.**
