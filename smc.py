#!/usr/bin/env python3

import time
import multiprocessing as mp
from RPi import GPIO
from time import sleep

# 4 full-step coil patterns, LSB..MSB = coils 1..4 (two-phase-on)
_FULL_STEP = [
    0b0011,  # 1+2
    0b0110,  # 2+3
    0b1100,  # 3+4
    0b1001,  # 4+1
]

# 28BYJ-48, full-step effective output shaft angle (2048 steps/rev)
STEP_ANGLE_DEG = 360.0 / 2048.0


def _norm_shortest_delta(target_deg, current_deg):
    """Return signed shortest delta in degrees in (-180, 180]."""
    d = (target_deg - current_deg) % 360.0
    if d > 180.0:
        d -= 360.0
    return d


# ---------- 74HC595 SHIFTER ----------

class Shifter:
    def __init__(self, data, clock, latch):
        self.dataPin = data
        self.latchPin = latch
        self.clockPin = clock
        GPIO.setup(self.dataPin, GPIO.OUT)
        GPIO.setup(self.latchPin, GPIO.OUT)
        GPIO.setup(self.clockPin, GPIO.OUT)

    def ping(self, p):
        GPIO.output(p, 1)
        sleep(0)
        GPIO.output(p, 0)

    def shiftWord(self, dataword, num_bits):
        # pad to full byte boundary
        pad_bits = (8 - (num_bits % 8)) % 8
        for _ in range(pad_bits):
            GPIO.output(self.dataPin, 0)
            self.ping(self.clockPin)

        # LSB first
        for i in range(num_bits):
            bit = 1 if (dataword & (1 << i)) else 0
            GPIO.output(self.dataPin, bit)
            self.ping(self.clockPin)

        self.ping(self.latchPin)

    def shiftByte(self, databyte):
        self.shiftWord(databyte, 8)


# ---------- STEPPER CLASS USING SHIFTER ----------

class Stepper:
    def __init__(self, shifter, shared_byte, shared_lock,
                 bit_offset=0, step_delay=0.003):
        """
        shifter: shared Shifter instance (drives the 74HC595)
        shared_byte: multiprocessing.Value('B') shared by all motors
        shared_lock: multiprocessing.Lock shared by all motors
        bit_offset: 0 for QA..QD, 4 for QE..QH
        step_delay: seconds between full-steps (speed control)
        """
        self.s = shifter
        self.shared_byte = shared_byte
        self.shared_lock = shared_lock
        self.bit_offset = bit_offset
        self.step_delay = step_delay
        # Shared logical angle for THIS motor
        self.angle_deg = mp.Value('d', 0.0)
        # Shared current index in the 4-phase sequence
        self.phase = mp.Value('i', 0)

    # write this motor's nibble into the shared 8-bit value
    def _write_phase(self, phase_idx):
        seq_len = len(_FULL_STEP)
        pattern4 = _FULL_STEP[phase_idx % seq_len]

        with self.shared_lock:
            byte = self.shared_byte.value
            mask = 0x0F << self.bit_offset
            byte &= ~mask
            byte |= (pattern4 & 0x0F) << self.bit_offset
            self.shared_byte.value = byte
            self.s.shiftByte(byte)

        with self.phase.get_lock():
            self.phase.value = phase_idx % seq_len

    def _step_worker(self, steps, direction, step_delay):
        seq_len = len(_FULL_STEP)
        with self.phase.get_lock():
            idx = self.phase.value

        for _ in range(abs(steps)):
            idx = (idx + (1 if direction > 0 else -1)) % seq_len
            self._write_phase(idx)
            time.sleep(step_delay)
            with self.angle_deg.get_lock():
                self.angle_deg.value = (
                    self.angle_deg.value + direction * STEP_ANGLE_DEG
                ) % 360.0

        # de-energize only this motor's coils
        with self.shared_lock:
            byte = self.shared_byte.value
            mask = 0x0F << self.bit_offset
            byte &= ~mask
            self.shared_byte.value = byte
            self.s.shiftByte(byte)

    # ---- public APIs ----
    def rotate(self, degrees, speed_hz=200):
        """
        Rotate relative by 'degrees' (non-blocking).
        If m1.rotate(...) and m2.rotate(...) are called sequentially,
        both motors run at the same time in separate processes.
        """
        direction = 1 if degrees >= 0 else -1
        steps = int(round(abs(degrees) / STEP_ANGLE_DEG))
        step_delay = max(1.0 / float(speed_hz), self.step_delay)

        p = mp.Process(target=self._step_worker,
                       args=(steps, direction, step_delay))
        p.start()
        return p

    def goAngle(self, target_deg, speed_hz=200):
        """
        Absolute move to target_deg using the SHORTEST path.
        Uses self.angle_deg (multiprocessing.Value) so the angle
        is shared across processes, as required in the handout.
        """
        with self.angle_deg.get_lock():
            cur = self.angle_deg.value % 360.0
        tgt = target_deg % 360.0
        delta = _norm_shortest_delta(tgt, cur)
        return self.rotate(delta, speed_hz)

    def zero(self):
        with self.angle_deg.get_lock():
            self.angle_deg.value = 0.0


# ---------------- demo / lab harness ----------------
if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)

    # Tie EN pins high: 17, 22, 23
    for pin in (17, 22, 23):
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH)

    DATA, CLK, LATCH = 16, 20, 21
    s = Shifter(DATA, CLK, LATCH)

    # SINGLE shared 8-bit register image + lock for both motors
    shared_byte = mp.Value('B', 0)
    shared_lock = mp.Lock()

    # Motor 1 on QA..QD, Motor 2 on QE..QH
    m1 = Stepper(s, shared_byte, shared_lock, bit_offset=0)
    m2 = Stepper(s, shared_byte, shared_lock, bit_offset=4)

    # ---- EXACT handout command sequence ----
    m1.zero()
    m2.zero()

    m1.goAngle(90)
    m1.goAngle(-45)

    m2.goAngle(-90)
    m2.goAngle(45)

    m1.goAngle(-135)
    m1.goAngle(135)
    m1.goAngle(0)
    # ---------------------------------------

    # Give any still-running processes time to finish
    time.sleep(5)

    GPIO.cleanup()
