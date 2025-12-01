#!/usr/bin/env python3
import http.server
import socketserver
import urllib.parse
import subprocess
import threading

PORT = 8000

PAGE = """
<!DOCTYPE html>
<html>
<head>
<title>Turret Interface</title>
<style>
body { font-family: Arial; max-width: 600px; margin:auto; }
fieldset { padding: 15px; margin-top: 20px; }
label { display:inline-block; width:150px; }
button { padding: 8px 20px; }
</style>
</head>
<body>

<h1>Turret Motor Interface</h1>

<!-- Run Full Turret Code -->
<fieldset>
<legend>Run Turret Code</legend>
<form method="POST" action="/run">
<label>Enter Turret ID:</label>
<input type="number" name="tid" required><br><br>
<button>Run Turret Program</button>
</form>
</fieldset>

<!-- Manual Motor Control -->
<fieldset>
<legend>Manual Motor Movement</legend>
<form method="POST" action="/move">
<label>Motor:</label>
<select name="motor">
  <option value="1">Motor 1</option>
  <option value="2">Motor 2</option>
</select><br><br>

<label>Degrees:</label>
<input type="number" step="0.1" name="deg" required><br><br>

<button>Rotate</button>
</form>
</fieldset>

</body>
</html>
"""


# ---- Manual motor movement (no import of turretmotors) ----
def rotate_motor(motor, degrees):
    import RPi.GPIO as GPIO
    from shifter import Shifter
    from stepper_class_shiftregister_multiprocessing import Stepper
    import multiprocessing
    import time

    print(f"[MANUAL] Rotating Motor {motor} by {degrees} degrees")

    s = Shifter(data=17, clock=27, latch=22)
    lock = multiprocessing.Lock()

    m1 = Stepper(s, lock)
    m2 = Stepper(s, lock)

    if motor == 1:
        m1.rotate(degrees)
    else:
        m2.rotate(degrees)

    time.sleep(0.1)


class Handler(http.server.BaseHTTPRequestHandler):

    def send_html(self, html):
        data = html.encode()
        self.send_response(200)
        self.send_header("Content-Type","text/html")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        self.send_html(PAGE)

    def do_POST(self):
        length = int(self.headers.get("Content-Length",0))
        raw = self.rfile.read(length).decode()
        params = urllib.parse.parse_qs(raw)

        # ---- Run turret code ----
        if self.path == "/run":
            tid = params["tid"][0]

            def run_motor_program():
                print(f"[SERVER] Running turretmotors.py with ID {tid}")
                subprocess.Popen(["python3", "turretmotors.py", tid])

            threading.Thread(target=run_motor_program, daemon=True).start()
            return self.send_html(PAGE)

        # ---- Manual motor control ----
        if self.path == "/move":
            motor = int(params["motor"][0])
            deg = float(params["deg"][0])

            rotate_motor(motor, deg)
            return self.send_html(PAGE)


# ---- Start server first ----
with socketserver.TCPServer(("", PORT), Handler) as server:
    print(f"\nWeb interface running at: http://<your_pi_ip>:{PORT}\n")
    print("Press CTRL+C to stop.")
    server.serve_forever()
