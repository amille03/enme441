# import time
# import board
# import neopixel

# pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)

# pixel.brightness = 0.3

# while True:
#     pixel.fill((255, 0, 0))
#     time.sleep(0.5)
#     pixel.fill((0, 255, 0))
#     time.sleep(0.5)
#     pixel.fill((0, 0, 255))
#     time.sleep(0.5)



# import time
# import board
# import busio

# # List of potential I2C busses
# ALL_I2C = ("board.I2C()", "board.STEMMA_I2C()", "busio.I2C(board.GP1, board.GP0)")

# # Determine which busses are valid
# found_i2c = []
# for name in ALL_I2C:
#     try:
#         print("Checking {}...".format(name), end="")
#         bus = eval(name)
#         bus.unlock()
#         found_i2c.append((name, bus))
#         print("ADDED.")
#     except Exception as e:
#         print("SKIPPED:", e)

# # Scan valid busses
# if len(found_i2c):
#     print("-" * 40)
#     print("I2C SCAN")
#     print("-" * 40)
#     while True:
#         for bus_info in found_i2c:
#             name = bus_info[0]
#             bus = bus_info[1]

#             while not bus.try_lock():
#                 pass

#             print(
#                 name,
#                 "addresses found:",
#                 [hex(device_address) for device_address in bus.scan()],
#             )

#             bus.unlock()

#         time.sleep(2)
# else:
#     print("No valid I2C bus found.")



# """CircuitPython I2C Device Address Scan"""
# # If you run this and it seems to hang, try manually unlocking
# # your I2C bus from the REPL with
# #  >>> import board
# #  >>> board.I2C().unlock()

# import time
# import board

# # To use default I2C bus (most boards)
# i2c = board.I2C()  # uses board.SCL and board.SDA
# # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

# # To create I2C bus on specific pins
# # import busio
# # i2c = busio.I2C(board.SCL1, board.SDA1)  # QT Py RP2040 STEMMA connector
# # i2c = busio.I2C(board.GP1, board.GP0)    # Pi Pico RP2040

# while not i2c.try_lock():
#     pass

# try:
#     while True:
#         print(
#             "I2C addresses found:",
#             [hex(device_address) for device_address in i2c.scan()],
#         )
#         time.sleep(2)

# finally:  # unlock the i2c bus when ctrl-c'ing out of the loop
#     i2c.unlock()



# import time
# import board
# import adafruit_bno055

# i2c = board.I2C()  # uses board.SCL and board.SDA
# sensor = adafruit_bno055.BNO055_I2C(i2c)

# last_val = 0xFFFF

# def temperature():
#     global last_val  # noqa: PLW0603
#     result = sensor.temperature
#     if abs(result - last_val) == 128:
#         result = sensor.temperature
#         if abs(result - last_val) == 128:
#             return 0b00111111 & result
#     last_val = result
#     return result


# while True:
#     print(f"Temperature: {sensor.temperature} degrees C")
#     """
#     print(
#         "Temperature: {} degrees C".format(temperature())
#     )  # Uncomment if using a Raspberry Pi
#     """
#     print(f"Accelerometer (m/s^2): {sensor.acceleration}")
#     print(f"Magnetometer (microteslas): {sensor.magnetic}")
#     print(f"Gyroscope (rad/sec): {sensor.gyro}")
#     print(f"Euler angle: {sensor.euler}")
#     print(f"Quaternion: {sensor.quaternion}")
#     print(f"Linear acceleration (m/s^2): {sensor.linear_acceleration}")
#     print(f"Gravity (m/s^2): {sensor.gravity}")
#     print()

#     time.sleep(1)



# import time
# import board
# import adafruit_bno055
# import ulab
# import gc

# gc.collect()
# free_memory = gc.mem_free()
# print("Available memory: {} bytes".format(free_memory))
# alloc_memory = gc.mem_alloc()
# print("Allocated memory: {} bytes".format(alloc_memory))

# print("\nCreating array. . .\n")

# # 76896 total (allegedly)
# test = ulab.numpy.zeros([32000,1], dtype=ulab.numpy.float)

# gc.collect()
# free_memory = gc.mem_free()
# print("Available memory: {} bytes".format(free_memory))
# alloc_memory = gc.mem_alloc()
# print("Allocated memory: {} bytes".format(alloc_memory))



import time
import board
import adafruit_bno055
import ulab
import neopixel
from analogio import AnalogIn

""""
pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
pixel.brightness = 0.3

i2c = board.I2C()  # uses board.SCL and board.SDA
sensor = adafruit_bno055.BNO055_I2C(i2c)

i = 0
L = 32000
rows = 5
L_adj = int(L / rows)
data = ulab.numpy.zeros([L_adj,rows], dtype=ulab.numpy.float)
wait = 0.01

print("Running data storage duration test. . .")
start = time.monotonic()
while i < L_adj:
    # print(f"Accelerometer (m/s^2): {sensor.acceleration}")
    # print(f"Magnetometer (microteslas): {sensor.magnetic}")
    # print(f"Gyroscope (rad/sec): {sensor.gyro}")
    # print(f"Euler angle: {sensor.euler}")
    # print(f"Quaternion: {sensor.quaternion}")
    # print(f"Linear acceleration (m/s^2): {sensor.linear_acceleration}")
    # print(f"Gravity (m/s^2): {sensor.gravity}")
    # print()

    data[i,0] = time.monotonic() - start
    data[i,1] = sensor.quaternion[0]
    data[i,2] = sensor.quaternion[1]
    data[i,3] = sensor.quaternion[2]
    data[i,4] = sensor.quaternion[3]

    if i % 1000 == 0:
        print(f"{i} of {L_adj}")
    i += 1

    time.sleep(wait)

end = time.monotonic()
print(f"Done. {1/wait}Hz; {end - start}s total duration")

filename = "data.csv"

try:
    with open(filename, 'w') as f:
        # f.write("column1,column2,column3\n")
        for row in data:
            row_as_string = ",".join(map(str, row))
            f.write(row_as_string + "\n")
except OSError as e:  # Typically when the filesystem isn't writeable...
    delay = 0.5  # ...blink the LED every half second.
    if e.args[0] == 28:  # If the file system is full...
        delay = 0.25  # ...blink the LED faster!
    while True:
        pixel.value = not pixel.value
        time.sleep(delay)
"""
analog_in = AnalogIn(board.A0)
def get_voltage(pin):
    return (pin.value * 3.3) / 65536


while True:
    print((get_voltage(analog_in),))
    time.sleep(0.1)
