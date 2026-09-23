# 합성 검증 장면 (test_marker_drift.py --save-dir 산출물)

정답(카메라 위치·방향)을 정해 놓고 그 자리에서 보일 마커를 원근법으로 그린 가짜 사진과, 실제 검출기로 정답을 되찾은 결과.
초록 테두리 = 검출된 태그, 파란 글씨 = 정답(truth) 대 복원값(solved). hires 120° 화각·1280×800, 7 cm 태그, 0.5 m 간격 가정.

| 파일 | 내용 |
|---|---|
| `tag36h11_id0_texture.png` | 태그 원본 무늬(ID 0). 코너 순서 규약 확인용 |
| `scene_x0.30_z0.60_yaw+4.0.png` | 시작 부근, 오른쪽 4° 회전. 태그 0·1 |
| `scene_x2.50_z0.60_yaw+0.0.png` | 중간, 정면. 태그 4·5 부근 |
| `scene_x5.25_z2.00_yaw-2.0.png` | 끝, 고도 2.0 m 상단열, 왼쪽 2° |
| `joint_vs_single_yaw±N.png` | 같은 위치에서 yaw 만 바꿔 태그 1개 PnP 와 태그 묶음 PnP 의 yaw 를 비교 |

다시 만들기: `.venv/Scripts/python analysis/test_marker_drift.py --save-dir analysis/samples/synthetic`
