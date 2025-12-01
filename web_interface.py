#!/usr/bin/env python3
import http.server
import socketserver
import urllib.parse
import threading
import time

# -------------------- HARDWARE SETUP --------------------
from RPi import GPIO
from shifter import Shifter
from stepper_class_shiftregister_multiprocessing import Stepper
import multiprocessing

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Shared shift register
s = Shifter(data=17, clock=27, latch=22)

# Lock for multiprocessing-safe updates
lock = multiprocessing.Lock()

# Two motors on the same shift register
# m1 is on Qe–Qh
# m2 is on Qa–Qd
m1 = Stepper(s, lock)
m2 = Stepper(s, lock)

# Start with both motors zeroed
m1.zero()
m2.zero()

print("[SERVER] Motors initialized and zeroed.")


# -------------------- HTML INTERFACE --------------------

PAGE = """
<!DOCTYPE html>
<html>
<head>
<title>Motor Control Interface</title>
<style>
body { font-family: Arial; max-width: 600px; margin:auto; }
fieldset { padding: 15px; margin-top: 20px; }
label { display:inline-block; width:140px; margin-bottom:6px; }
button { padding: 7px 18px; margin-top:10px; }
input[type=number] { width:100px; }
</style>
</head>
<body>

<h1>Manual Motor Control</h1>

<fieldset>
<legend>Motor 1 (Turret)</legend>
<form method="POST" action="/m1">
<label>Rotate degrees:</label>
<input name="deg" type="number" step="0.1"><br><br>
<button name="cmd" value="rotate">Rotate</button>
<button name="cmd" value="zero">Zero Motor 1</button>
</form>
</fieldset>

<fieldset>
<legend>Motor 2 (Globe)</legend>
<form method="POST" action="/m2">
<label>Rotate degrees:</label>
<input name="deg" type="number" step="0.1"><br><br>
<button name="cmd" value="rotate">Rotate</button>
<button name="cmd" value="zero">Zero Motor 2</button>
</form>
</fieldset>

</body>
</html>
"""


# -------------------- REQUEST HANDLER --------------------

class Handler(http.server.BaseHTTPRequestHandler):

    def send_html(self, html):
        encoded = html.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def do_GET(self):
        print("[SERVER] GET request")
        self.send_html(PAGE)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length).decode()
        params = urllib.parse.parse_qs(raw)

        # Motor 1 actions
        if self.path == "/m1":
            cmd = params.get("cmd", [""])[0]
            if cmd == "rotate":
                deg = float(params["deg"][0])
                print(f"[M1] Rotating {deg} degrees")
                threading.Thread(target=m1.rotate, args=(deg,), daemon=True).start()
            elif cmd == "zero":
                print("[M1] Zeroing motor")
                threading.Thread(target=m1.zero, daemon=True).start()

        # Motor 2 actions
        if self.path == "/m2":
            cmd = params.get("cmd", [""])[0]
            if cmd == "rotate":
                deg = float(params["deg"][0])
                print(f"[M2] Rotating {deg} degrees")
                threading.Thread(target=m2.rotate, args=(deg,), daemon=True).start()
            elif cmd == "zero":
                print("[M2] Zeroing motor")
                threading.Thread(target=m2.zero, daemon=True).start()

        # Return page to user
        self.send_html(PAGE)


# -------------------- MAIN SERVER --------------------

PORT = 8000

with socketserver.TCPServer(("", PORT), Handler) as server:
    print(f"[SERVER] Web UI running at: http://<your_pi_ip>:{PORT}")
    server.serve_forever()
