#!/usr/bin/env python3
import time, sys
import RPi.GPIO as GPIO

PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4 = 5, 6, 13, 19
PIN_EN12, PIN_EN34 = 24, 25  # set to GPIO numbers if you wired them

FULLSTEP_SEQ = [
    (1,1,0,0), (0,1,1,0), (0,0,1,1), (1,0,0,1)
]

_pins_configured = False

def setup():
    global _pins_configured
    try:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for p in (PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4):
            GPIO.setup(p, GPIO.OUT, initial=GPIO.HIGH)  # HIGH = off (LOW=ON with L293D)
        if PIN_EN12 is not None:
            GPIO.setup(PIN_EN12, GPIO.OUT, initial=GPIO.HIGH)
        if PIN_EN34 is not None:
            GPIO.setup(PIN_EN34, GPIO.OUT, initial=GPIO.HIGH)
        _pins_configured = True
    except Exception as e:
        print(f"[setup] Failed to claim GPIO: {e}")
        GPIO.cleanup()
        sys.exit(1)

def write_coils(a,b,c,d):
    # Only drive pins if we actually configured them
    if not _pins_configured:
        return
    GPIO.output(PIN_IN1, GPIO.LOW if a else GPIO.HIGH)
    GPIO.output(PIN_IN2, GPIO.LOW if b else GPIO.HIGH)
    GPIO.output(PIN_IN3, GPIO.LOW if c else GPIO.HIGH)
    GPIO.output(PIN_IN4, GPIO.LOW if d else GPIO.HIGH)

def release():
    if _pins_configured:
        write_coils(0,0,0,0)

def demo():
    # Very slow full-step to verify motion/torque
    delay = 0.01
    for _ in range(512):  # eighth of a rev (2048 full-steps per rev)
        for patt in FULLSTEP_SEQ:
            write_coils(*patt)
            time.sleep(delay)
    release()

if __name__ == "__main__":
    try:
        setup()
        demo()
    except KeyboardInterrupt:
        pass
    finally:
        release()
        GPIO.cleanup()
