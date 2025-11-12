#!/usr/bin/env python3
# step_l293d.py — Drive 28BYJ-48 via L293D on Raspberry Pi (Zero 2 W)
# Works for unipolar hookup: red wire to +5V; coils on OUT1..OUT4.
# IMPORTANT: L293D sinks current -> output LOW energizes a coil.

import time
import RPi.GPIO as GPIO

# ================= User config (BCM numbering) =================
# Pi GPIOs to L293D inputs IN1..IN4 (pins 2,7,10,15 on the L293D)
PIN_IN1 = 17
PIN_IN2 = 27
PIN_IN3 = 22
PIN_IN4 = 23

# If EN1,2 (pin1) and EN3,4 (pin9) are wired to GPIO, set pins here.
# If they are tied HIGH on the board, set these to None.
PIN_EN12 = None  # e.g., 24
PIN_EN34 = None  # e.g., 25

# 28BYJ-48 color → which L293D OUTPUT it is on (for your sanity)
# OUT1→Blue, OUT2→Pink, OUT3→Yellow, OUT4→Orange is typical.
COLOR_MAP = {
    "Blue"   : "OUT1",
    "Pink"   : "OUT2",
    "Yellow" : "OUT3",
    "Orange" : "OUT4",
    "Red"    : "+5V (common)"
}

# Steps/rev of 28BYJ-48 gearbox (half-step)
HALFSTEPS_PER_REV = 4096

# Set how long each half-step lasts initially (slower is safer to start)
START_RPM = 8.0
# ===============================================================

# Half-step pattern: 1 means "energize this coil"
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

# Because the L293D sinks current, "energize" == drive LOW.
# We'll translate 1→LOW and 0→HIGH before writing to pins.
def write_coils(a,b,c,d):
    GPIO.output(PIN_IN1, GPIO.LOW if a else GPIO.HIGH)
    GPIO.output(PIN_IN2, GPIO.LOW if b else GPIO.HIGH)
    GPIO.output(PIN_IN3, GPIO.LOW if c else GPIO.HIGH)
    GPIO.output(PIN_IN4, GPIO.LOW if d else GPIO.HIGH)

def release():
    # De-energize all coils (outputs HIGH)
    write_coils(0,0,0,0)

def rpm_to_delay(rpm):
    rpm = max(0.1, float(rpm))
    sec_per_rev = 60.0 / rpm
    return sec_per_rev / HALFSTEPS_PER_REV  # seconds per half-step

def setup():
    GPIO.setmode(GPIO.BCM)
    for p in (PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4):
        GPIO.setup(p, GPIO.OUT, initial=GPIO.HIGH)  # HIGH = off (see note)
    if PIN_EN12 is not None:
        GPIO.setup(PIN_EN12, GPIO.OUT, initial=GPIO.HIGH)
    if PIN_EN34 is not None:
        GPIO.setup(PIN_EN34, GPIO.OUT, initial=GPIO.HIGH)

def coil_self_test(dwell=1.0):
    """
    Slowly energize each individual coil so you can confirm wiring.
    You should feel/see the rotor 'twitch' to four distinct holding positions.
    """
    print("Coil self-test: energizing one coil at a time...")
    names = ["Blue (OUT1)","Pink (OUT2)","Yellow (OUT3)","Orange (OUT4)"]
    patterns = [(1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1)]
    for name, patt in zip(names, patterns):
        print("  ->", name)
        write_coils(*patt)
        time.sleep(dwell)
        release()
        time.sleep(0.3)
    print("Self-test done.\n")

def step_n(n_steps, rpm=START_RPM, cw=True):
    delay = rpm_to_delay(rpm)
    idx = 0
    direction = 1 if cw else -1
    rng = range(abs(int(n_steps)))
    for _ in rng:
        idx = (idx + direction) % 8
        write_coils(*HALFSTEP_SEQ[idx])
        time.sleep(delay)
    release()

def move_degrees(deg, rpm=START_RPM):
    steps = int(round((deg / 360.0) * HALFSTEPS_PER_REV))
    cw = steps >= 0
    step_n(abs(steps), rpm=rpm, cw=cw)

def demo():
    print("\n=== L293D + 28BYJ-48 demo ===")
    print("Mapping:", COLOR_MAP, "\n")
    coil_self_test(dwell=0.8)

    print("1 rev CW @ slow RPM...")
    step_n(HALFSTEPS_PER_REV, rpm=6.0, cw=True)
    time.sleep(0.5)

    print("1 rev CCW @ slow RPM...")
    step_n(HALFSTEPS_PER_REV, rpm=6.0, cw=False)
    time.sleep(0.5)

    print("Quarter-turn nudges (CW)...")
    for _ in range(4):
        move_degrees(90, rpm=10.0)
        time.sleep(0.3)

    print("Quarter-turn nudges (CCW)...")
    for _ in range(4):
        move_degrees(-90, rpm=10.0)
        time.sleep(0.3)

if __name__ == "__main__":
    try:
        setup()
        demo()
        print("Done.")
    except KeyboardInterrupt:
        pass
    finally:
        release()
        GPIO.cleanup()
        print("GPIO cleaned up.")
