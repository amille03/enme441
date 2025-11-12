#!/usr/bin/env python3
# Test 28BYJ-48 stepper via L293D on Raspberry Pi Zero 2 W

import time
import math
import RPi.GPIO as GPIO

# ========= Pin map (BCM) =========
PIN_IN1 = 17
PIN_IN2 = 27
PIN_IN3 = 22
PIN_IN4 = 23

# ========= Motor constants =========
# 28BYJ-48 typically ~4096 half-steps per output shaft revolution (gearbox)
HALFSTEPS_PER_REV = 4096

# Half-step sequence: energize one or two coils at a time
HALFSTEP_SEQ = [
    (1,0,0,0),
    (1,1,0,0),
    (0,1,0,0),
    (0,1,1,0),
    (0,0,1,0),
    (0,0,1,1),
    (0,0,0,1),
    (1,0,0,1),
]

def setup():
    GPIO.setmode(GPIO.BCM)
    for p in (PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4):
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

    if PIN_EN12 is not None:
        GPIO.setup(PIN_EN12, GPIO.OUT, initial=GPIO.HIGH)
    if PIN_EN34 is not None:
        GPIO.setup(PIN_EN34, GPIO.OUT, initial=GPIO.HIGH)

def set_coils(a, b, c, d):
    GPIO.output(PIN_IN1, GPIO.HIGH if a else GPIO.LOW)
    GPIO.output(PIN_IN2, GPIO.HIGH if b else GPIO.LOW)
    GPIO.output(PIN_IN3, GPIO.HIGH if c else GPIO.LOW)
    GPIO.output(PIN_IN4, GPIO.HIGH if d else GPIO.LOW)

def release():
    set_coils(0,0,0,0)

def step_once(index, direction=1, delay_s=0.001):
    """Perform one half-step and return next index."""
    index = (index + direction) % 8
    set_coils(*HALFSTEP_SEQ[index])
    time.sleep(delay_s)
    return index

def run_steps(n_steps, rpm=10.0, direction=1):
    """
    Run given number of half-steps at target RPM of the output shaft.
    direction: +1 (CW) or -1 (CCW)
    """
    # time per half-step based on desired shaft RPM
    sec_per_rev = 60.0 / max(rpm, 0.1)
    delay_s = sec_per_rev / HALFSTEPS_PER_REV

    idx = 0
    for _ in range(abs(int(n_steps))):
        idx = step_once(idx, direction=1 if direction >= 0 else -1, delay_s=delay_s)

def move_degrees(deg, rpm=10.0):
    """Move the shaft by a signed number of degrees."""
    steps = int(round((deg / 360.0) * HALFSTEPS_PER_REV))
    direction = 1 if steps >= 0 else -1
    run_steps(abs(steps), rpm=rpm, direction=direction)

def demo():
    print("L293D 28BYJ-48 test: 1 rev CW, pause, 1 rev CCW, then 90° steps.")
    # 1 revolution CW
    run_steps(HALFSTEPS_PER_REV, rpm=12.0, direction=1)
    time.sleep(0.5)
    # 1 revolution CCW
    run_steps(HALFSTEPS_PER_REV, rpm=12.0, direction=-1)
    time.sleep(0.5)
    # Nudge tests
    for _ in range(4):
        move_degrees(90, rpm=15.0)
        time.sleep(0.3)
    for _ in range(4):
        move_degrees(-90, rpm=15.0)
        time.sleep(0.3)

if __name__ == "__main__":
    try:
        setup()
        demo()
    except KeyboardInterrupt:
        pass
    finally:
        release()
        GPIO.cleanup()
        print("Clean exit.")
