#!/usr/bin/env python3
import multiprocessing as mp
import time
from RPi import GPIO
from time import sleep

# ---------------- Shift register class (from handout) ----------------

GPIO.setmode(GPIO.BCM)

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

    # Shift all bits in an arbitrary-length word
    def shiftword(self, dataword, num_bits):
        # Pad to full byte
        for _ in range((num_bits + 1) % 8):
            GPIO.output(self.dataPin, 0)
            self.ping(self.clockPin)
        # Send actual bits LSB first
        for i in range(num_bits):
            GPIO.output(self.dataPin, dataword & (1 << i))
            self.ping(self.clockPin)
        self.ping(self.latchPin)

    def shiftByte(self, databyte):
        self.shiftword(databyte, 8)

# ---------------- Global shift-register state ----------------

# 8-bit shared output image of the shift register (0..255)
SR_STATE = mp.Value('i', 0, lock=True)
SR_NUM_BITS = 8

# Full-step pattern (two-phase on), LSB..MSB = coils 1..4
_FULL_STEP = [
    0b0011,  # 1+2
    0b0110,  # 2+3
    0b1100,  # 3+4
    0b1001,  # 4+1
]

STEPS_PER_REV = 200          # adjust if your motor differs
DEG_PER_STEP  = 360.0 / STEPS_PER_REV

# ---------------- Stepper class ----------------

class Stepper:
    def __init__(self, shifter, base_bit):
        """
        base_bit: starting bit position (0 or 4) for this motor in the 8-bit SR.
        Motor 1 -> bits 0..3, Motor 2 -> bits 4..7.
        """
        self.shifter = shifter
        self.base_bit = base_bit
        self.mask = 0b1111 << base_bit

        self.step_delay = 0.01          # seconds between full-steps
        self._step_index = 0            # which pattern of _FULL_STEP we are on

        # Angle shared across processes
        self.angle = mp.Value('d', 0.0)  # degrees, relative to zero position

        # Worker process for current motion
        self._proc = None

    # ---- low-level stepping ----
    def _apply_pattern(self, pattern):
        """Write this motor's 4-bit pattern into the global 8-bit SR state."""
        with SR_STATE.get_lock():
            word = SR_STATE.value & ~self.mask             # clear this motor's bits
            word |= (pattern << self.base_bit) & self.mask # set new pattern
            SR_STATE.value = word
            self.shifter.shiftByte(word)

    def _step_once(self, direction):
        # choose next pattern index
        if direction > 0:
            self._step_index = (self._step_index + 1) % len(_FULL_STEP)
        else:
            self._step_index = (self._step_index - 1) % len(_FULL_STEP)

        pattern = _FULL_STEP[self._step_index]
        self._apply_pattern(pattern)

        # update angle
        with self.angle.get_lock():
            self.angle.value = (self.angle.value + direction * DEG_PER_STEP) % 360.0

    def _run_steps(self, steps, direction):
        for _ in range(steps):
            self._step_once(direction)
            time.sleep(self.step_delay)

    # ---- public API ----
    def _start_motion(self, steps, direction):
        """Start a new motion, waiting for any previous motion on THIS motor."""
        # Make sure previous motion on this motor has finished
        if self._proc is not None:
            self._proc.join()

        if steps == 0:
            return

        self._proc = mp.Process(target=self._run_steps, args=(steps, direction))
        self._proc.start()

    def zero(self):
        """Set logical zero angle without moving the motor."""
        if self._proc is not None:
            self._proc.join()
            self._proc = None
        with self.angle.get_lock():
            self.angle.value = 0.0

    def goAngle(self, target_deg):
        """
        Move to absolute angle target_deg (deg) relative to zero,
        following the SHORTEST path.
        """
        # Normalize current and target to [0, 360)
        with self.angle.get_lock():
            cur = self.angle.value % 360.0
        tgt = target_deg % 360.0

        # Smallest signed difference in (-180, 180]
        delta = (tgt - cur) % 360.0
        if delta > 180.0:
            delta -= 360.0

        if abs(delta) < 1e-6:
            return  # already there

        direction = 1 if delta > 0 else -1
        steps = int(round(abs(delta) / DEG_PER_STEP))
        self._start_motion(steps, direction)

    def wait(self):
        """Block until this motor is done moving."""
        if self._proc is not None:
            self._proc.join()
            self._proc = None

# ---------------- Main demo ----------------

def main():
    try:
        # Set the requested pins HIGH (e.g., enables for drivers)
        for p in (17, 22, 23):
            GPIO.setup(p, GPIO.OUT, initial=GPIO.HIGH)

        # Create shift register; adjust GPIO pins as wired on your Pi
        s = Shifter(data=16, clock=20, latch=21)

        # Two motors sharing the same shift register:
        m1 = Stepper(s, base_bit=0)  # bits 0..3
        m2 = Stepper(s, base_bit=4)  # bits 4..7

        # ----- Sequence from the lab instructions -----
        m1.zero()
        m2.zero()

        # First pair (should run concurrently)
        m1.goAngle(90)
        m2.goAngle(-45)

        # Wait for both to finish before next pair
        m1.wait()
        m2.wait()

        # Second pair
        m2.goAngle(-90)
        m1.goAngle(45)

        m1.wait()
        m2.wait()

        # Third group for motor 1 only
        m1.goAngle(-135)
        m1.wait()

        m1.goAngle(135)
        m1.wait()

        m1.goAngle(0)
        m1.wait()

    finally:
        # turn everything off and clean up GPIO
        with SR_STATE.get_lock():
            SR_STATE.value = 0
        s.shiftByte(0)
        GPIO.cleanup()

if __name__ == "__main__":
    main()
