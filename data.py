import serial
import time
import msvcrt  # Windows-only
import numpy as np
import matplotlib.pyplot as plt   # <<< NEW

PORT = "COM4"
BAUD = 115200

def open_serial_blocking(port=PORT, baud=BAUD, timeout=1):
    """Keep trying to open the serial port until it is usable."""
    while True:
        try:
            ser = serial.Serial(port, baudrate=baud, timeout=timeout)
            # Poke driver; if device isn't ready this will throw.
            _ = ser.in_waiting
            ser.reset_input_buffer()
            print(f"[Python] Opened {port}")
            return ser
        except serial.SerialException as e:
            print(f"[Python] Waiting for {port} to come back: {e}")
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(0.5)

def send_reset_and_reopen(ser):
    """Tell the Feather to reset, close old handle, and return a fresh Serial."""
    try:
        ser.write(b"RESET\n")
        ser.flush()
    except serial.SerialException as e:
        print(f"[Python] Error while sending RESET: {e}")

    try:
        ser.close()
    except Exception:
        pass

    # Give the RP2040 time to reboot and re-enumerate
    time.sleep(1.5)
    new_ser = open_serial_blocking()
    time.sleep(2.0)  # let setup() finish
    print("[Python] Feather reset complete, continuing...\n")
    return new_ser

def read_dump(ser):
    """
    Called right after we've seen 'START_DUMP:'.
    Reads lines until 'END_DUMP' and returns:

      experiment_data: 3xN float array [times; raw_adc; voltage]
      times_str: list of time strings (exactly as sent by Feather)
      vibes_str: list of raw ADC strings (exactly as sent by Feather)
      volts: 1D float array of voltages
    """
    times_str = []
    vibes_str = []
    times_float = []
    vibes_float = []

    while True:
        raw = ser.readline()
        if not raw:
            print("[Python] Dump read timeout / no data.")
            break

        line = raw.decode("utf-8", errors="ignore").strip()

        if line == "END_DUMP":
            # Clean end of dump
            break

        if not line:
            continue  # skip empty lines

        # Assuming lines like: "\t123.45\t0.987"
        parts = line.split()
        if len(parts) < 2:
            print(f"[Python] Skipping malformed dump line: {line!r}")
            continue

        # Preserve exact string tokens as sent by the Feather
        t_str = parts[0]
        v_str = parts[1]
        times_str.append(t_str)
        vibes_str.append(v_str)

        # Also keep numeric versions for math / voltage conversion
        try:
            t_val = float(t_str)
            v_val = float(v_str)
        except ValueError:
            print(f"[Python] Non-numeric dump line: {line!r}")
            # Don't add to numeric arrays if it's not numeric
            continue

        times_float.append(t_val)
        vibes_float.append(v_val)

    if not times_float:
        print("[Python] Warning: saw 'START_DUMP:' but no usable numeric data.")
        return None

    times_arr = np.array(times_float)
    vibes_arr = np.array(vibes_float)

    # Convert raw ADC to voltage: (analog_in.value * 3.3) / 65536
    volts = (vibes_arr * 3.3) / 65536.0

    # 3 x N data array: [time; raw_adc; voltage]
    experiment_data = np.vstack((times_arr, vibes_arr, volts))
    print(f"[Python] Captured dump with shape {experiment_data.shape}")
    return experiment_data, times_str, vibes_str, volts

def main():
    ser = open_serial_blocking()
    time.sleep(2.0)  # initial setup time

    experiment_data = None  # will hold latest 3xN dump

    print("Listening to Feather. SPACE/ENTER = reset; Ctrl+C = quit.")

    try:
        while True:
            # ---- Keyboard handling for reset ----
            if msvcrt.kbhit():
                key = msvcrt.getwch()
                if key in (" ", "\r", "\n"):
                    print("\n[Python] Reset key pressed, resetting Feather...")
                    ser = send_reset_and_reopen(ser)
                    continue

            # ---- Serial reading ----
            try:
                if ser.in_waiting > 0:
                    raw = ser.readline()
                    line = raw.decode("utf-8", errors="ignore").strip()

                    if line == "START_DUMP:":
                        print("[Python] Detected START_DUMP header, reading dump...")
                        dump_result = read_dump(ser)

                        if dump_result is not None:
                            experiment_data, times_str, vibes_str, volts = dump_result

                            # Example: print array (numeric)
                            print("Experiment data array (rows: time, raw_adc, voltage):")
                            print(experiment_data)

                            # Quick stats based on numeric data (optional)
                            times = experiment_data[0, :]
                            vibes = experiment_data[1, :]

                            print(f"Samples: {experiment_data.shape[1]}")
                            print(f"Time range: {times[0]} -> {times[-1]}")
                            print(f"Raw ADC min/max: {vibes.min()} / {vibes.max()}")

                            # === Save to CSV ===
                            # First two columns are EXACT strings from the Feather.
                            # Third column is the computed voltage.
                            with open("experiment_data.csv", "w", newline="") as f:
                                f.write("time,raw_adc,voltage\n")
                                for t_str, v_str, v_volt in zip(times_str, vibes_str, volts):
                                    # t_str and v_str are exactly as received from the Feather
                                    f.write(f"{t_str},{v_str},{v_volt}\n")

                            print("[Python] Saved experiment_data to experiment_data.csv")

                            # === NEW: plot voltage vs time ===
                            plt.figure()
                            plt.plot(times, volts, linewidth=0.5)          # times: numeric timestamps, volts: numeric voltages
                            plt.xlabel("Time")              # label as appropriate: seconds, ms, etc.
                            plt.ylabel("Voltage (V)")
                            plt.title("Vibration Voltage vs Time")
                            plt.grid(True)
                            plt.show()                      # blocks until you close the window
                            # If you want to keep collecting data after closing the plot,
                            # this is fine: the loop will continue.

                        continue  # go back to top of loop

                    # Normal non-dump output:
                    print(f"[FEATHER]: {line}")

            except serial.SerialException as e:
                print(f"[Python] Serial error while reading: {e}")
                print("[Python] Trying to reopen the port...")
                ser = open_serial_blocking()
                time.sleep(2.0)
                continue

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        try:
            ser.close()
        except Exception:
            pass

    if experiment_data is not None:
        print("Final experiment_data:")
        print(experiment_data)

if __name__ == "__main__":
    main()
