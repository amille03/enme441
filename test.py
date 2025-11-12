#!/usr/bin/env python3
import sys, time
import RPi.GPIO as GPIO

# BCM pins to L293D IN1..IN4
PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4 = 17, 27, 22, 23
PIN_EN12, PIN_EN34 = None, None   # set if you actually wired EN pins to GPIO

FULLSTEP_SEQ = [(1,1,0,0),(0,1,1,0),(0,0,1,1),(1,0,0,1)]
CONFIGURED = False

def setup():
    global CONFIGURED
    try:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for p in (PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4):
            GPIO.setup(p, GPIO.OUT, initial=GPIO.HIGH)  # HIGH = off
        if PIN_EN12 is not None:
            GPIO.setup(PIN_EN12, GPIO.OUT, initial=GPIO.HIGH)
        if PIN_EN34 is not None:
            GPIO.setup(PIN_EN34, GPIO.OUT, initial=GPIO.HIGH)
        CONFIGURED = True
        # Sanity printout
        for p in (PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4):
            fn = GPIO.gpio_function(p)  # 1=OUTPUT, 0=INPUT
            print(f"GPIO{p} mode={fn} (1 means OUTPUT)")
    except Exception as e:
        print(f"[setup] Could not claim GPIO: {e}")
        GPIO.cleanup()
        sys.exit(1)   # <- exit so we never fall through to release()

def write_coils(a,b,c,d):
    if not CONFIGURED:  # guard
        return
    GPIO.output(PIN_IN1, GPIO.LOW if a else GPIO.HIGH)
    GPIO.output(PIN_IN2, GPIO.LOW if b else GPIO.HIGH)
    GPIO.output(PIN_IN3, GPIO.LOW if c else GPIO.HIGH)
    GPIO.output(PIN_IN4, GPIO.LOW if d else GPIO.HIGH)

def release():
    if CONFIGURED:
        write_coils(0,0,0,0)

def demo():
    delay = 0.01
    for _ in range(8):               # tiny move
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
