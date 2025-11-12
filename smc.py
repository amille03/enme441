#!/usr/bin/env python3
import multiprocessing as mp
import time
from RPi import GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)

# ---------------- SHIFTER ----------------
class Shifter:
    def __init__(self, data, clock, latch):
        self.dataPin = data
        self.clockPin = clock
        self.latchPin = latch
        GPIO.setup(data, GPIO.OUT)
        GPIO.setup(clock, GPIO.OUT)
        GPIO.setup(latch, GPIO.OUT)

    def ping(self, pin):
        GPIO.output(pin, 1)
        sleep(0)
        GPIO.output(pin, 0)

    def shiftword(self, dataword, num_bits):
        # pad to byte boundary
        for _ in range((num_bits + 1) % 8):
            GPIO.output(self.dataPin, 0)
            self.ping(self.clockPin)

        for i in range(num_bits):
            GPIO.output(self.dataPin, dataword & (1 << i))
            self.ping(self.clockPin)

        self.ping(self.latchPin)

    def shiftByte(self, databyte):
        self.shiftword(databyte, 8)


# -------------- GLOBAL SHIFT REGISTER BYTE --------------
SR_STATE = mp.Value('i', 0, lock=True)

# Two-phase-on, full-step sequence
FULLSTEP = [
    0b0011,
    0b0110,
    0b1100,
    0b1001
]

STEPS_PER_REV = 200
DEG_PER_STEP = 360 / STEPS_PER_REV


# ---------------- STEPPER CLASS ----------------
class Stepper:
    def __init__(self, sh, base_bit):
        self.sh = sh
        self.base_bit = base_bit   # 0–3 for m1, 4–7 for m2
        self.mask = 0b1111 << base_bit

        self.step_delay = 0.01
        self.index = 0
        self.angle = mp.Value('d', 0.0)  # shared angle

        self.proc = None

    def _apply(self, pattern):
        """update ONLY this motor's 4 bits"""
        with SR_STATE.get_lock():
            old = SR_STATE.value & (~self.mask)
            new = old | ((pattern << self.base_bit) & self.mask)
            SR_STATE.value = new
            self.sh.shiftByte(new)

    def _one_step(self, direction):
        if direction > 0:
            self.index = (self.index + 1) % 4
        else:
            self.index = (self.index - 1) % 4

        pattern = FULLSTEP[self.index]
        self._apply(pattern)

        with self.angle.get_lock():
            self.angle.value = (self.angle.value + direction * DEG_PER_STEP) % 360

    def _run(self, steps, direction):
        for _ in range(steps):
            self._one_step(direction)
            sleep(self.step_delay)

    def goAngle(self, deg):
        # ensure previous motion done
        if self.proc:
            self.proc.join()

        with self.angle.get_lock():
            cur = self.angle.value

        tgt = deg % 360
        cur = cur % 360

        delta = (tgt - cur) % 360
        if delta > 180:
            delta -= 360

        if abs(delta) < 1e-6:
            return

        direction = 1 if delta > 0 else -1
        steps = int(round(abs(delta) / DEG_PER_STEP))

        self.proc = mp.Process(target=self._run, args=(steps, direction))
        self.proc.start()

    def zero(self):
        if self.proc:
            self.proc.join()
        with self.angle.get_lock():
            self.angle.value = 0.0

    def wait(self):
        if self.proc:
            self.proc.join()
            self.proc = None


# ---------------- MAIN ----------------
def main():
    # Required pins set HIGH
    for p in (17, 22, 23):
        GPIO.setup(p, GPIO.OUT)
        GPIO.output(p, GPIO.HIGH)

    sh = Shifter(data=16, clock=20, latch=21)

    m1 = Stepper(sh, base_bit=0)  # bits 0–3
    m2 = Stepper(sh, base_bit=4)  # bits 4–7

    try:
        # -------- EXACT LAB COMMAND SEQUENCE --------
        m1.zero()
        m2.zero()

        # these two MUST move simultaneously
        m1.goAngle(90)
        m2.goAngle(-45)
        m1.wait()
        m2.wait()

        # also simultaneous
        m2.goAngle(-90)
        m1.goAngle(45)
        m1.wait()
        m2.wait()

        # Motor 1 final sequence
        m1.goAngle(-135)
        m1.wait()

        m1.goAngle(135)
        m1.wait()

        m1.goAngle(0)
        m1.wait()

    finally:
        with SR_STATE.get_lock():
            SR_STATE.value = 0
        sh.shiftByte(0)
        GPIO.cleanup()


if __name__ == "__main__":
    main()
