import RPi.GPIO as GPIO
import time
from shifter import Shifter, Bug   # both classes are in shifter.py

# --- Pin Configuration (BCM numbering) ---
DATA_PIN  = 14     # serial (DS)
CLOCK_PIN = 15     # clock (SHCP)
LATCH_PIN = 18     # latch (STCP)

S1_PIN = 17        # on/off
S2_PIN = 27        # wrap toggle
S3_PIN = 22        # speed x3
# ------------------------------------------

def main():
    GPIO.setmode(GPIO.BCM)

    # Input setup with pull-down resistors
    for pin in (S1_PIN, S2_PIN, S3_PIN):
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    # Initialize Shifter and Bug objects
    sh = Shifter(serialPin=DATA_PIN, clockPin=CLOCK_PIN, latchPin=LATCH_PIN)
    bug = Bug(shifter=sh, timestep=0.05, x=3, isWrapOn=False)

    s2_prev = GPIO.input(S2_PIN)

    print("Running Bug demo. Use switches:")
    print("S1 = On/Off | S2 = Toggle wrap | S3 = Speed x3")

    try:
        while True:
            s1 = GPIO.input(S1_PIN)
            s2 = GPIO.input(S2_PIN)
            s3 = GPIO.input(S3_PIN)

            # a. S1 controls ON/OFF
            if s1:
                bug.start()
            else:
                bug.stop()

            # b. S2 toggles wrapping on any state change
            if s2 != s2_prev:
                bug.isWrapOn = not bug.isWrapOn
                print(f"Wrap mode {'ON' if bug.isWrapOn else 'OFF'}")
                s2_prev = s2

            # c. S3 increases speed 3x while held
            bug.set_speed_multiplier(3.0 if s3 else 1.0)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        bug.stop()
        sh.cleanup()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
