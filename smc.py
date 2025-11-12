#!/usr/bin/env python3

# --- requirements ---
# pip install RPi.GPIO  (already on Pi OS)

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

# shared 8-bit output image for ALL motors (for simultaneous operation)
GLOBAL_OUT = mp.Value('B', 0)   # unsigned char 0–255

# ---------- 74HC595 SHIFTER ----------

class Shifter:
    def __init__(self, data, clock, latch):
        self.dataPin = data
        self.latchPin = latch
        self.clockPin = clock
        GPIO.setup(self.dataPin, GPIO.OUT)
        GPIO.setup(self.latchPin, GPIO.OUT)
        GPIO.setup(self.clockPin, GPIO.OUT)

    def ping(self, p):  # pulse the clock or latch pin
        GPIO.output(p, 1)
        sleep(0)
        GPIO.output(p, 0)

    # Shift all bits in an arbitrary-length word, allowing
    # multiple 8-bit shift registers to be chained:
    def shiftWord(self, dataword, num_bits):
        # pad to full byte boundary with zeros:
        pad_bits = (8 - (num_bits % 8)) % 8
        for _ in range(pad_bits):
            GPIO.output(self.dataPin, 0)
            self.ping(self.clockPin)
        # now shift the actual bits, LSB first
        for i in range(num_bits):
            bit = 1 if (dataword & (1 << i)) else 0
            GPIO.output(self.dataPin, bit)
            self.ping(self.clockPin)
        self.ping(self.latchPin)

    # Shift all bits in a single byte:
    def shiftByte(self, databyte):
        self.shiftWord(databyte, 8)

# ---------- STEPPER CLASS USING SHIFTER ----------

class Stepper:
    def __init__(self, shifter, bit_offset=0, step_delay=0.002):
        """
        shifter: shared Shifter instance (drives the 74HC595)
        bit_offset: 0 for QA..QD, 4 for QE..QH
        step_delay: seconds between full-steps (speed control)
        """
        self.s = shifter
        self.bit_offset = bit_offset
        self.step_delay = step_delay
        # Shared angle across processes (double precision)
        self.angle_deg = mp.Value('d', 0.0)
        # Shared current index in the 4-phase sequence
        self.phase = mp.Value('i', 0)

    # ---- low-level: write this motor's 4 bits into the shared 8-bit byte ----
    def _write_phase(self, phase_idx):
        seq_len = len(_FULL_STEP)
        pattern4 = _FULL_STEP[phase_idx % seq_len]  # 4-bit pattern for this motor

        # update only our nibble inside GLOBAL_OUT, protecting with a lock
        with GLOBAL_OUT.get_lock():
            byte = GLOBAL_OUT.value
            mask = 0x0F << self.bit_offset      # mask for our 4 bits
            byte &= ~mask                       # clear our bits
            byte |= (pattern4 & 0x0F) << self.bit_offset
            GLOBAL_OUT.value = byte
            out_byte = byte

        # actually shift the combined pattern to the 74HC595
        self.s.shiftByte(out_byte)

        with self.phase.get_lock():
            self.phase.value = phase_idx % seq_len

    # ---- worker used by multiprocessing ----
    def _step_worker(self, steps, direction, step_delay):
        seq_len = len(_FULL_STEP)
        # read starting index once
        with self.phase.get_lock():
            idx = self.phase.value

        for _ in range(abs(steps)):
            idx = (idx + (1 if direction > 0 else -1)) % seq_len
            self._write_phase(idx)
            time.sleep(step_delay)
            # update shared angle
            with self.angle_deg.get_lock():
                self.angle_deg.value = (
                    self.angle_deg.value + direction * STEP_ANGLE_DEG
                ) % 360.0

        # de-energize ONLY this motor's coils (leave other motor running)
        with GLOBAL_OUT.get_lock():
            byte = GLOBAL_OUT.value
            mask = 0x0F << self.bit_offset
            byte &= ~mask         # clear our nibble
            GLOBAL_OUT.value = byte
        self.s.shiftByte(byte)

    # ---- public APIs ----
    def rotate(self, degrees, speed_hz=400):
        """
        Rotate relative by 'degrees' (positive = forward),
        speed_hz controls step timing (higher = faster).

        IMPORTANT: this spawns a separate process and returns immediately,
        so two calls like

            m1.rotate(90)
            m2.rotate(-90)

        will cause both motors to move simultaneously.
        """
        direction = 1 if degrees >= 0 else -1
        steps = int(round(abs(degrees) / STEP_ANGLE_DEG))
        step_delay = max(1.0 / float(speed_hz), 0.002)  # clamp for reliability
        p = mp.Process(target=self._step_worker,
                       args=(steps, direction, step_delay))
        p.start()
        return p  # caller can join() if desired

    def goAngle(self, target_deg, speed_hz=400):
        """
        Absolute move to 'target_deg' (0..360), following the SHORTEST path.
        Uses self.angle_deg (a multiprocessing.Value) so angle is shared across
        processes, as required.
        """
        with self.angle_deg.get_lock():
            cur = self.angle_deg.value % 360.0
        delta = _norm_shortest_delta(target_deg % 360.0, cur)
        return self.rotate(delta, speed_hz)

    def zero(self):
        """Set logical angle to zero (does not move)."""
        with self.angle_deg.get_lock():
            self.angle_deg.value = 0.0

# ---------------- demo / lab harness ----------------
if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)

    # Tie EN (or other control) pins high: 17, 22, 23
    for pin in (17, 22, 23):
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH)

    # 74HC595 pins:
    DATA, CLK, LATCH = 16, 20, 21
    s = Shifter(DATA, CLK, LATCH)

    # Motor 1 on QA..QD (bits 0..3), Motor 2 on QE..QH (bits 4..7)
    m1 = Stepper(s, bit_offset=0, step_delay=0.002)
    m2 = Stepper(s, bit_offset=4, step_delay=0.002)

    # --- demo matching the lab instructions conceptually ---

    m1.zero()
    m2.zero()

    # example pair: both move simultaneously
    p1 = m1.goAngle(90)
    p2 = m2.goAngle(-90)
    p1.join(); p2.join()

    p1 = m1.goAngle(-45)
    p2 = m2.goAngle(45)
    p1.join(); p2.join()

    p1 = m1.goAngle(-135)
    p2 = m2.goAngle(135)
    p1.join(); p2.join()

    # home motor 1
    p = m1.goAngle(0)
    p.join()

    GPIO.cleanup()
