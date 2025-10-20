import RPi.GPIO as GPIO
import time
import random
import threading

GPIO.setmode(GPIO.BCM)

class Shifter:
    """
    Handles all shift-register functionality for one 74HC595.
    """
    def __init__(self, serialPin: int, clockPin: int, latchPin: int):
        self.serialPin = serialPin
        self.clockPin  = clockPin
        self.latchPin  = latchPin

        GPIO.setup(self.serialPin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.clockPin,  GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.latchPin,  GPIO.OUT, initial=GPIO.LOW)

    # private
    def __ping(self, p: int):
        GPIO.output(p, GPIO.HIGH)
        time.sleep(0)
        GPIO.output(p, GPIO.LOW)

    # public
    def shiftByte(self, b: int):
        """Send 8 bits (LSB first) and latch output."""
        b &= 0xFF
        for i in range(8):
            GPIO.output(self.serialPin, GPIO.HIGH if (b >> i) & 1 else GPIO.LOW)
            self.__ping(self.clockPin)
        self.__ping(self.latchPin)

    def clear(self):
        self.shiftByte(0x00)

    def cleanup(self):
        self.clear()
        GPIO.cleanup()


class Bug:
    """
    Random-walk LED “bug” using a Shifter object.
    Attributes:
        timestep  – seconds between steps
        x         – current LED index (0–7)
        isWrapOn  – whether movement wraps around
    """
    def __init__(self, shifter: Shifter, timestep: float = 0.1, x: int = 3, isWrapOn: bool = False):
        self.timestep  = float(timestep)
        self.x         = max(0, min(7, int(x)))
        self.isWrapOn  = bool(isWrapOn)
        self.__shifter = shifter   # private composition

        self._running    = False
        self._thread     = None
        self._speed_mult = 1.0

    def _pattern(self) -> int:
        return 1 << self.x

    def _step_once(self):
        move = -1 if random.random() < 0.5 else 1
        new_x = self.x + move

        if self.isWrapOn:
            self.x = new_x % 8
        else:
            if 0 <= new_x <= 7:
                self.x = new_x

        self.__shifter.shiftByte(self._pattern())

    def _run(self):
        self.__shifter.shiftByte(self._pattern())
        while self._running:
            self._step_once()
            time.sleep(max(0.001, self.timestep / self._speed_mult))

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
        self.__shifter.clear()

    def set_speed_multiplier(self, m: float):
        self._speed_mult = max(0.1, float(m))
