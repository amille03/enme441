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
            GPIO.output(self.dataPin, dataword & (1 << i))
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

    # ---- low-level: write 8-bit image to the 595 with our 4 bits in place ----
    def _compose_byte(self, nibble4):
        # Place our 4-bit pattern at bit_offset, leave the other motor's 4 bits zero.
        return (nibble4 & 0x0F) << self.bit_offset

    def _write_phase(self, phase_idx):
        seq_len = len(_FULL_STEP)
        pattern4 = _FULL_STEP[phase_idx % seq_len]  # 4-bit for this motor
        out_byte = self._compose_byte(pattern4)
        self.s.shiftByte(out_byte)  # clocks & latches inside your Shifter
        with self.phase.get_lock():
            self.phase.value = phase_idx % seq_len

    # ---- worker used by multiprocessing ----
    def _step_worker(self, steps, direction, step_delay):
        seq_len = len(_FULL_STEP)
        # read once
        with self.phase.get_lock():
            idx = self.phase.value
        for _ in range(abs(steps)):
            idx = (idx + (1 if direction > 0 else -1)) % seq_len
            self._write_phase(idx)
            time.sleep(step_delay)
            # update shared angle
