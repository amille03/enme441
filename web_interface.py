#!/usr/bin/env python3
import http.server
import socketserver
import urllib.parse
import urllib.request
import json
import threading
import time

from RPi import GPIO

# === import your real motor system and math pipeline ===
import smc                     # ← your full turret + globe + rotation math
import jsontester              # ← your JSON extraction code

# Initialize laser GPIO
LASER_PIN = 5
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(LASER_PIN, GPIO.OUT, initial=GPIO.HIGH)

# Controller wrapper from smc.py
ctrl = smc.MotorController()


# ---------------- HTML ------------------

PAGE = """\
<!DOCTYPE html>
<html>
<head>
<title>Laser Turret System</title>
<style>
body { font-family: Arial; width: 600px; margin:auto; }
fieldset { margin:20px 0; padding:15px; }
label { display:inline-block; width:140px; }
input[type=number], input[type=text] { width:100px; }
</style>
</head>
<body>

<h1>ENME441 Laser Turret</h1>

<!-- Laser Control -->
<fieldset>
<legend>Laser Control</legend>
<form method="POST" action="/laser">
<button name="state" value="on">Laser ON</button>
<button name="state" value="off">Laser OFF</button>
</form>
</fieldset>

<!-- Manual Motor -->
<fieldset>
<legend>Manual Motor Control</legend>
<form method="POST" action="/manual">
<label>Turret (deg):</label>
<input name="turret" type="number" step="0.1"><br><br>
<label>Globe (deg):</label>
<input name="globe" type="number" step="0.1"><br><br>

<button name="mode" value="relative">Rotate (Relative)</button>
<button name="mode" value="absolute">Rotate (Absolute)</button>
<button name="mode" value="zero">Zero Motors</button>
</form>
</fieldset>

<!-- Auto Targeting -->
<fieldset>
<legend>Automated JSON Targeting</legend>
<form method="POST" action="/auto">
<label>JSON URL:</label><br>
<input type="text" name="url" style="width:100%;" placeholder="http://..."><br><br>
<button>Run Auto Targeting</button>
</form>
</fieldset>

</body>
</html>
"""


# ------------- HTTP Handler -------------------

class Handler(http.server.BaseHTTPRequestHandler):

    def html(self, page):
        body = page.encode()
        self.send_response(200)
        self.send_header("Content-Type","text/html")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        self.html(PAGE)

    def do_POST(self):
        size = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(size).decode()
        params = urllib.parse.parse_qs(raw)

        # ---------------- Laser ----------------
        if self.path == "/laser":
            if params.get("state",["off"])[0] == "on":
                ctrl.laser_on()
            else:
                ctrl.laser_off()
            return self.html(PAGE)

        # ---------------- Manual Motor ----------------
        if self.path == "/manual":
            mode = params.get("mode",["relative"])[0]
            t = float(params.get("turret",[0])[0])
            g = float(params.get("globe",[0])[0])

            if mode == "relative":
                ctrl.move_relative(t,g)
            elif mode == "absolute":
                ctrl.move_absolute(t,g)
            elif mode == "zero":
                ctrl.zero_all()

            return self.html(PAGE)

        # ---------------- Automated JSON ----------------
        if self.path == "/auto":
            url = params.get("url",[""])[0]

            def worker():
                print("Downloading JSON:", url)
                with urllib.request.urlopen(url, timeout=5) as f:
                    data = json.load(f)

                # Use your EXACT JSON parsing logic (reusing your script)
                # ----------------------------------------------------------------
                turrets = []
                turret_ids = []
                turret_r = []
                turret_theta = []

                for tid, tinfo in data["turrets"].items():
                    turret_ids.append(int(tid))
                    turret_r.append(tinfo["r"])
                    turret_theta.append(tinfo["theta"])

                globe_r = []
                globe_theta = []
                globe_z = []

                for g in data["globes"]:
                    globe_r.append(g["r"])
                    globe_theta.append(g["theta"])
                    globe_z.append(g["z"])

                print("Parsed Turrets:", turret_ids)
                print("Parsed Globes:", globe_theta)

                # ----------------------------------------------------------------
                # FEED INTO YOUR PIPELINE EXACTLY LIKE YOUR REAL CODE
                # ----------------------------------------------------------------

                full = smc.compute_rotations_from_json(
                    turret_ids,
                    turret_r,
                    turret_theta,
                    globe_r,
                    globe_theta,
                    globe_z
                )

                # full = list of (turret_delta, globe_delta)
                for (dt, dg) in full:
                    ctrl.move_relative(dt, dg)
                    time.sleep(0.15)

            threading.Thread(target=worker, daemon=True).start()
            return self.html(PAGE)

        # default
        self.html("<h1>Error</h1>")


# --------------- SERVER MAIN ------------------

with socketserver.TCPServer(("", 8000), Handler) as server:
    print("Web UI running at http://<pi>:8000")
    server.serve_forever()
