#!/usr/bin/env python3
import http.server
import socketserver
import urllib.parse
import threading
import time

from RPi import GPIO
from shifter import Shifter
from stepper_class_shiftregister_multiprocessing import Stepper
import multiprocessing

# ================================================================
#       INITIALIZE MOTORS HERE SO WE CAN CONTROL THEM DIRECTLY
# ================================================================

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# One shared shift register for both motors
s = Shifter(data=17, clock=27, latch=22)

# A lock so both motors don’t update the shift register at the same time
lock = multiprocessing.Lock()

# Instantiate the stepper motors
m1 = Stepper(s, lock)   # Turret motor (Qe–Qh)
m2 = Stepper(s, lock)   # Globe motor (Qa–Qd)

# Zero the motors when server starts
try:
    m1.zero()
    m2.zero()
except:
    pass

print("[SERVER] Motors initialized and zeroed.")


# ================================================================
#                     HTML WEB PAGE
# ================================================================

PAGE = """
<!DOCTYPE html>
<html>
<head>
<title>ENME441 Turret Control</title>
<style>
body { font-family: Arial; max-width: 600px; margin:auto; }
fieldset { padding: 15px; margin-top: 20px; }
label { display:inline-block; width:140px; }
button { padding: 8px 20px; }
</style>
</head>
<body>

<h1>ENME441 Manual Turret Control</h1>

<!-- Enter Turret ID -->
<fieldset>
<legend>Set Turret ID</legend>
<form method="POST" action="/id">
<label>Turret ID:</label>
<input type="number" name="tid" required>
<button>Set ID</button>
</form>
</fieldset>

<!-- Manual Motor Movement -->
<fieldset>
<legend>Manual Motor Movement</legend>
<form method="POST" action="/move">
<label>Turret Motor (deg):</label>
<input type="number" name="turret" step="0.1"><br><br>
<label>Globe Motor (deg):</label>
<input type="number" name="globe" step="0.1"><br><br>
<button>Rotate Motors</button>
</form>
</fieldset>

</body>
</html>
"""

current_turret_id = None


# ================================================================
#                     HTTP SERVER HANDLER
# ================================================================

class Handler(http.server.BaseHTTPRequestHandler):

    def send_html(self, html):
        data = html.encode()
        self.send_response(200)
        self.send_header("Content-Type","text/html")
        self.send_header("Content-Length",str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        self.send_html(PAGE)

    def do_POST(self):
        global current_turret_id

        length = int(self.headers.get("Content-Length",0))
        body = self.rfile.read(length).decode()
        params = urllib.parse.parse_qs(body)

        # ---------------- SET ID ----------------
        if self.path == "/id":
            current_turret_id = params.get("tid",[""])[0]
            print(f"[SERVER] Turret ID set to: {current_turret_id}")
            return self.send_html(PAGE)

        # ---------------- MANUAL MOTOR MOVEMENT ----------------
        if self.path == "/move":
            turret_deg = float(params.get("turret",[0])[0])
            globe_deg  = float(params.get("globe", [0])[0])

            print(f"[SERVER] Moving turret m1 by {turret_deg}°")
            print(f"[SERVER] Moving globe  m2 by {globe_deg}°")

            def worker():
                if turret_deg != 0:
                    m1.rotate(turret_deg)
                if globe_deg != 0:
                    m2.rotate(globe_deg)

            threading.Thread(target=worker, daemon=True).start()
            return self.send_html(PAGE)

        self.send_html(PAGE)


# ================================================================
#                     START SERVER
# ================================================================

PORT = 8000
with socketserver.TCPServer(("", PORT), Handler) as server:
    print(f"[SERVER] Web UI running on: http://<your_pi_ip>:{PORT}")
    server.serve_forever()
