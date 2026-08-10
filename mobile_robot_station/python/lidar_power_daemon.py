#!/usr/bin/env python3
"""
라이다 모터 전원 데몬 — GPIO 20번 핀을 HIGH로 유지.
myAGV에서 라이다 모터는 GPIO 20으로 제어됨 (AGV_UI 방식과 동일).

실행: python3 lidar_power_daemon.py &
종료: kill <PID>  또는  Ctrl+C  →  GPIO 20 LOW (모터 정지)
"""
import signal
import sys
import RPi.GPIO as GPIO

RADAR_PIN = 20


def power_off(sig=None, frame=None):
    GPIO.output(RADAR_PIN, GPIO.LOW)
    GPIO.cleanup()
    sys.exit(0)


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(RADAR_PIN, GPIO.OUT)
GPIO.output(RADAR_PIN, GPIO.HIGH)

signal.signal(signal.SIGTERM, power_off)
signal.signal(signal.SIGINT, power_off)

print(f"[lidar_power] GPIO {RADAR_PIN} HIGH — 라이다 모터 ON")
signal.pause()  # SIGTERM/SIGINT 올 때까지 대기
