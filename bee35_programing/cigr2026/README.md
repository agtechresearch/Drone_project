# CIGR 2026 포스터 실험 자료

`bee35_programing` 정렬 알고리즘의 CIGR 2026 포스터 발표용 본 실험 기록 (2026년 6월 13일 ~ 22일).
상위 폴더 [`../README.md`](../README.md)의 3월 실험 이후, 임계값을 px → cm 기준으로 바꾼 v14~v17 시리즈와
PID 정렬(poster) vs LOITER 단독(baseline_poster) 대조 실험이다.

## 폴더 구조

```
cigr2026/
├── README.md
├── alighncompare.py            # 초록·포스터용 그림 생성 (figures/ 출력)
├── figures/                    # fig1_key_metrics, fig2_time_boxplot, fig3_error_cdf
├── logs/                       # 6/13~6/17 개발 중 비행 로그 (hybrid_v4~6, loiter_v2~v17) + 분석 report.txt
├── test16/
│   ├── v16/                    # v16 튜닝 비행 로그 (6/17)
│   └── poster/
│       ├── alighn_l_poster.py           # 본 실험 비행 코드 (PID 정렬, cm 임계)
│       ├── alighn_l_baseline_poster.py  # 대조군 비행 코드 (첫 정렬 후 PID 해제, LOITER+OF만)
│       ├── check_loiter_poster.py       # v14+ 로그 분석 (cm 기준 성공률·거리 분포·PID 응답·운영 박스)
│       ├── poster/                      # 본 실험 로그 (v16, 6/17 20:30~21:31)
│       ├── baseline_poster/             # 대조군 로그 (baseline_v14, 6/17~6/22)
│       └── poster_fgure/                # check_loiter_poster.py 출력 그림·report
└── test17/                     # v17 비행 로그 (6/17 17:21~17:32)
```

## 비행 영상

비행 영상 mp4(29개, 약 640 MB)는 용량 때문에 리포에 넣지 않았다.
연구실 공유 드라이브에 보관 예정. 원본은 실험 PC의 `BEE35_cigr/cigr_poster/` 아래 같은 경로에 있다.

- `logs/`: `flight_video_v6~v13_*_visualized.mp4` (6개)
- `test16/v16/`: `flight_video_v16_20260617_*.mp4` (6개)
- `test16/poster/poster/`: `flight_video_v16_20260617_{203050,205055,211912,213126}.mp4` (4개, `_visualized` 1개 별도)
- `test16/poster/baseline_poster/`: `flight_video_baseline_v14_*.mp4` (5개)

> 드라이브 링크: (추가 예정)

## 로그 파일명 규칙

`{모드}_{버전}_{realtime|summary}_{YYYYMMDD_HHMMSS}.csv`, 분석 결과는 같은 이름에 `_report.txt`.
컬럼 정의는 상위 README의 "로그 시스템" 절 참조. v14 이후 `marker_px`, `err_x_cm`, `distance_cm`, `lateral_thr_px` 컬럼이 추가됐다.

## 사용법

```bash
# 본 실험 로그 분석 (가장 최근 로그 자동 선택)
python test16/poster/check_loiter_poster.py
python test16/poster/check_loiter_poster.py --baseline   # 대조군

# 초록·포스터 그림 3장 재생성
python alighncompare.py logs/
```
