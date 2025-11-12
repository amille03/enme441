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


# Shared 8-bit image for the shift register and a lock
SHIFT_BYTE = mp.Value('B', 0)   # unsigned char
SHIFT_LOCK = mp.Lock()

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

        # shift LSB first
        for i in range(num_bits):
            bit = 1 if (dataword & (1 << i)) else 0
            GPIO.output(self.dataPin, bit)
            self.ping(self.clockPin)

        self.ping(self.latchPin)

    def shiftByte(self, databyte):
        self.shiftWord(databyte, 8)


# ---------- STEPPER CLASS USING SHIFTER ----------

class Stepper:
    def __init__(self, shifter, bit_offset=0, step_delay=0.003):
        """
        shifter: shared Shifter instance (drives the 74HC595)
        bit_offset: 0 for QA..QD, 4 for QE..QH
        step_delay: seconds between full-steps (speed control)
        """
        self.s = shifter
        self.bit_offset = bit_offset
        self.step_delay = step_delay
        self.angle_deg = mp.Value('d', 0.0)  # shared logical angle
        self.phase = mp.Value('i', 0)        # shared index in _FULL_STEP
        self.proc = None                     # last motion process for this motor

    def _write_phase(self, phase_idx):
        seq_len = len(_FULL_STEP)
        pattern4 = _FULL_STEP[phase_idx % seq_len]

        # update only our nibble in SHIFT_BYTE
        with SHIFT_LOCK:
            byte = SHIFT_BYTE.value
            mask = 0x0F << self.bit_offset
            byte &= ~mask
            byte |= (pattern4 & 0x0F) << self.bit_offset
            SHIFT_BYTE.value = byte
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

        # de-energize ONLY this motor's coils
        with SHIFT_LOCK:
            byte = SHIFT_BYTE.value
            mask = 0x0F << self.bit_offset
            byte &= ~mask
            SHIFT_BYTE.value = byte
            self.s.shiftByte(byte)

    # ---- public APIs ----
    def rotate(self, degrees, speed_hz=200):
        """
        Rotate relative by 'degrees'.

        - Ensures THIS motor's previous motion finishes before starting a new one
          (so its commands happen in sequence).
        - Still allows the *other* motor to run concurrently.
        """
        # ensure our previous process (if any) is done
        if self.proc is not None and self.proc.is_alive():
            self.proc.join()

        direction = 1 if degrees >= 0 else -1
        steps = int(round(abs(degrees) / STEP_ANGLE_DEG))
        step_delay = max(1.0 / float(speed_hz), self.step_delay)

        p = mp.Process(
            target=self._step_worker,
            args=(steps, direction, step_delay)
        )
        p.start()
        self.proc = p
        return p

    def goAngle(self, target_deg, speed_hz=200):
        """Absolute move to target_deg via the shortest path."""
        with self.angle_deg.get_lock():
            cur = self.angle_deg.value % 360.0
        tgt = target_deg % 360.0
        delta = _norm_shortest_delta(tgt, cur)
        return self.rotate(delta, speed_hz)

    def zero(self):
        with self.angle_deg.get_lock():
            self.angle_deg.value = 0.0

    def wait(self):
        """Wait for this motor's last motion to finish."""
        if self.proc is not None and self.proc.is_alive():
            self.proc.join()


# ---------------- demo / lab harness ----------------
if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)

    # Tie EN pins high: 17, 22, 23
    for pin in (17, 22, 23):
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH)

    DATA, CLK, LATCH = 16, 20, 21
    s = Shifter(DATA, CLK, LATCH)

    m1 = Stepper(s, bit_offset=0)
    m2 = Stepper(s, bit_offset=4)

    # ----- EXACT lab command sequence -----
    m1.zero()
    m2.zero()

    m1.goAngle(90)
    m1.goAngle(-45)

    m2.goAngle(-90)
    m2.goAngle(45)

    m1.goAngle(-135)
    m1.goAngle(135)
    m1.goAngle(0)
    # --------------------------------------

    # wait for both motors to finish their last motion
    m1.wait()
    m2.wait()

    GPIO.cleanup()
