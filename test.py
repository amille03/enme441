#!/usr/bin/env python3
# two_l293d_two_steppers.py
# Drive two 28BYJ-48 steppers on two different L293Ds (Raspberry Pi Zero 2 W)
# Unipolar wiring: red wire to +VS, coils on OUT1..OUT4. L293D sinks current -> LOW = ON.

import time
import threading
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# ----- Motion sequences -----
# Two-coil FULL-STEP (more torque, ~2048 steps/rev on 28BYJ-48 output shaft)
FULLSTEP_SEQ = [
    (1,1,0,0),  # A+B
    (0,1,1,0),  # B+C
    (0,0,1,1),  # C+D
    (1,0,0,1),  # D+A
]
FULLSTEPS_PER_REV = 2048

# HALF-STEP (one/two coils alternately, ~4096 steps/rev)
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
HALFSTEPS_PER_REV = 4096


class StepperL293D:
    """
    Controls a single 28BYJ-48 via one L293D using 4 GPIO inputs (IN1..IN4).
    Optional EN pins can be provided; if None, tie EN pins HIGH in hardware.
    """
    def __init__(self, in1, in2, in3, in4, en12=24, en34=25, mode="full"):
        self.IN1, self.IN2, self.IN3, self.IN4 = in1, in2, in3, in4
        self.EN12, self.EN34 = en12, en34
        self.mode = mode  # "full" or "half"
        self._configured = False
        self._stop = threading.Event()
        self._thread = None
        self._setup_gpio()

    def _setup_gpio(self):
        try:
            for p in (self.IN1, self.IN2, self.IN3, self.IN4):
                GPIO.setup(p, GPIO.OUT, initial=GPIO.HIGH)  # HIGH = off (LOW energizes)
            if self.EN12 is not None:
                GPIO.setup(self.EN12, GPIO.OUT, initial=GPIO.HIGH)
            if self.EN34 is not None:
                GPIO.setup(self.EN34, GPIO.OUT, initial=GPIO.HIGH)
            self._configured = True
        except Exception as e:
            print(f"[setup] Could not claim GPIO for motor ({self.IN1},{self.IN2},{self.IN3},{self.IN4}): {e}")
            raise

    def _seq_and_spr(self):
        if self.mode.lower() == "half":
            return HALFSTEP_SEQ, HALFSTEPS_PER_REV
        return FULLSTEP_SEQ, FULLSTEPS_PER_REV

    def _write_coils(self, a, b, c, d):
        if not self._configured:
            return
        # L293D sinks -> drive LOW to energize coil
        GPIO.output(self.IN1, GPIO.LOW if a else GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW if b else GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW if c else GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW if d else GPIO.HIGH)

    def release(self):
        self._write_coils(0,0,0,0)

    @staticmethod
    def _rpm_to_delay(steps_per_rev, rpm):
        rpm = max(0.1, float(rpm))
        sec_per_rev = 60.0 / rpm
        return sec_per_rev / steps_per_rev

    def run_steps(self, n_steps, rpm=6.0, cw=True, accel=True):
        """
        Blocking run: move given number of steps.
        cw=True for clockwise; False for counter-clockwise.
        """
        seq, spr = self._seq_and_spr()
        base_delay = self._rpm_to_delay(spr, rpm)

        # gentle acceleration helps torque and reduces stalls
        start_delay = min(0.010, max(0.003, base_delay * 3.0))
        total = abs(int(n_steps))
        direction = 1 if cw else -1
        idx = 0
        ramp_len = min(300, total // 3)

        for step in range(total):
            if self._stop.is_set():
                break
            # simple linear ramp on delay
            if accel and step < ramp_len:
                alpha = step / max(1, ramp_len)
                delay = start_delay + (base_delay - start_delay) * alpha
            else:
                delay = base_delay

            idx = (idx + direction) % len(seq)
            self._write_coils(*seq[idx])
            time.sleep(delay)

        self.release()

    def move_degrees(self, deg, rpm=6.0, cw=None, accel=True):
        seq, spr = self._seq_and_spr()
        steps = int(round((deg / 360.0) * spr))
        if steps == 0:
            return
        if cw is None:
            cw = steps >= 0
        self.run_steps(abs(steps), rpm=rpm, cw=cw, accel=accel)

    # Async helpers (so two motors can move at once)
    def start_async_steps(self, n_steps, rpm=6.0, cw=True, accel=True):
        self.stop_async()
        self._stop.clear()
        self._thread = threading.Thread(
            target=self.run_steps, args=(n_steps, rpm, cw, accel), daemon=True
        )
        self._thread.start()

    def stop_async(self):
        if self._thread and self._thread.is_alive():
            self._stop.set()
            self._thread.join(timeout=2.0)
        self._thread = None
        self._stop.clear()


def demo_two_motors():
    """
    Example wiring (customize as you like; these BCM pins are normally free):
      Motor A L293D IN1..IN4 <- GPIO 17,27,22,23
      Motor B L293D IN1..IN4 <- GPIO 5,6,13,19
    EN pins can be tied HIGH in hardware or mapped to GPIO and set HIGH here.
    """
    motorA = StepperL293D(in1=17, in2=27, in3=22, in4=23, en12=None, en34=None, mode="full")  # torque
    motorB = StepperL293D(in1=5,  in2=6,  in3=13, in4=19, en12=None, en34=None, mode="full")

    try:
        print("Both motors: 1 rev concurrently (A CW @6 RPM, B CCW @8 RPM)...")
        motorA.start_async_steps(+FULLSTEPS_PER_REV, rpm=6.0, cw=True)
        motorB.start_async_steps(+FULLSTEPS_PER_REV, rpm=8.0, cw=False)
        # Wait for both to finish
        while threading.active_count() > 1:
            time.sleep(0.1)

        time.sleep(0.3)
        print("Quarter turns in opposite directions...")
        motorA.start_async_steps(int(FULLSTEPS_PER_REV/4), rpm=10.0, cw=True)
        motorB.start_async_steps(int(FULLSTEPS_PER_REV/4), rpm=10.0, cw=False)
        while threading.active_count() > 1:
            time.sleep(0.1)

        time.sleep(0.3)
        print("Half-step finer motion (both CW 180°)...")
        motorA.mode = "half"
        motorB.mode = "half"
        motorA.start_async_steps(int(HALFSTEPS_PER_REV/2), rpm=8.0, cw=True)
        motorB.start_async_steps(int(HALFSTEPS_PER_REV/2), rpm=8.0, cw=True)
        while threading.active_count() > 1:
            time.sleep(0.1)

    finally:
        motorA.stop_async()
        motorB.stop_async()
        motorA.release()
        motorB.release()


if __name__ == "__main__":
    try:
        demo_two_motors()
    except KeyboardInterrupt:
        pass
    finally:
        # Always leave outputs HIGH (off) and clean up
        GPIO.cleanup()
        print("GPIO cleaned up.")
