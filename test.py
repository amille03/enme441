#!/usr/bin/env python3
# 28BYJ-48 via L293D, FULL-STEP (two coils ON), LOW=ON for torque
import time, RPi.GPIO as GPIO

# BCM pins -> L293D IN1..IN4
PIN_IN1 = 17
PIN_IN2 = 27
PIN_IN3 = 22
PIN_IN4 = 23
PIN_EN12 = None
PIN_EN34 = None  # set to GPIO numbers if you wired EN pins

# 28BYJ-48 ~2048 full-steps/rev (two-phase full-step on output shaft)
FULLSTEPS_PER_REV = 2048

# Two-coil full-step sequence (A,B,C,D). 1 means "energize this coil".
# Because L293D sinks current, we will output LOW for 1 and HIGH for 0.
FULLSTEP_SEQ = [
    (1,1,0,0),  # A+B
    (0,1,1,0),  # B+C
    (0,0,1,1),  # C+D
    (1,0,0,1),  # D+A
]

def setup():
    GPIO.setmode(GPIO.BCM)
    for p in (PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4):
        GPIO.setup(p, GPIO.OUT, initial=GPIO.HIGH)  # HIGH = off (LOW=ON)
    if PIN_EN12 is not None:
        GPIO.setup(PIN_EN12, GPIO.OUT, initial=GPIO.HIGH)
    if PIN_EN34 is not None:
        GPIO.setup(PIN_EN34, GPIO.OUT, initial=GPIO.HIGH)

def write_coils(a,b,c,d):
    GPIO.output(PIN_IN1, GPIO.LOW if a else GPIO.HIGH)
    GPIO.output(PIN_IN2, GPIO.LOW if b else GPIO.HIGH)
    GPIO.output(PIN_IN3, GPIO.LOW if c else GPIO.HIGH)
    GPIO.output(PIN_IN4, GPIO.LOW if d else GPIO.HIGH)

def release():
    write_coils(0,0,0,0)

def rpm_to_delay_fullstep(rpm):
    rpm = max(0.1, float(rpm))
    sec_per_rev = 60.0 / rpm
    return sec_per_rev / FULLSTEPS_PER_REV

def run_steps_full(n_steps, rpm=6.0, cw=True, accel=True):
    """
    Two-phase full-step with optional acceleration ramp to prevent stalls.
    """
    direction = 1 if cw else -1
    idx = 0
    total = abs(int(n_steps))

    # Base delay for target rpm
    base_delay = rpm_to_delay_fullstep(rpm)

    # Accel profile: start slow → target delay
    start_delay = min(0.008, max(0.003, base_delay*3))  # 3x slower at start
    ramp_len = min(300, total // 3)  # ramp over ~first third of motion

    for step in range(total):
        # simple linear ramp on delay
        if accel and step < ramp_len:
            # interpolate from start_delay -> base_delay
            alpha = step / max(1, ramp_len)
            delay = start_delay + (base_delay - start_delay) * alpha
        else:
            delay = base_delay

        idx = (idx + direction) % 4
        write_coils(*FULLSTEP_SEQ[idx])
        time.sleep(delay)

    release()

def move_degrees_full(deg, rpm=6.0, cw=None):
    steps = int(round((deg / 360.0) * FULLSTEPS_PER_REV))
    if steps == 0:
        return
    if cw is None:
        cw = steps >= 0
    run_steps_full(abs(steps), rpm=rpm, cw=cw, accel=True)

def demo():
    print("\n=== Full-step torque demo (two coils ON) ===")
    print("1 rev CW @ 6 RPM...")
    run_steps_full(FULLSTEPS_PER_REV, rpm=6.0, cw=True, accel=True)
    time.sleep(0.5)

    print("1 rev CCW @ 6 RPM...")
    run_steps_full(FULLSTEPS_PER_REV, rpm=6.0, cw=False, accel=True)
    time.sleep(0.5)

    print("Quarter turns CW...")
    for _ in range(4):
        move_degrees_full(90, rpm=8.0)
        time.sleep(0.3)

    print("Quarter turns CCW...")
    for _ in range(4):
        move_degrees_full(-90, rpm=8.0)
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
        print("GPIO cleaned up.")
