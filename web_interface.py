#!/usr/bin/env python3
import http.server
import socketserver
import urllib.parse
import threading

import turretmotors   # <-- Your motor code module

PORT = 8000

PAGE = """
<!DOCTYPE html>
<html>
<head>
<title>ENME441 Turret Control</title>
<style>
body { font-family: Arial; max-width: 600px; margin: auto; }
fieldset { padding: 15px; margin-top: 20px; }
label { display:inline-block; width: 130px; }
button { padding: 8px 20px; margin-top: 10px; }
</style>
</head>
<body>

<h1>ENME441 Turret Control</h1>

<!-- LASER -->
<fieldset>
<legend>Laser Control</legend>
<form method="POST" action="/laser">
<button name="cmd" value="on">Laser ON</button>
<button name="cmd" value="off">Laser OFF</button>
</form>
</fieldset>

<!-- RUN SEQUENCE -->
<fieldset>
<legend>Run Motor Sequence</legend>
<form method="POST" action="/run">
<label>Turret ID:</label>
<input type="number" name="tid" required><br><br>
<button>Run Sequence</button>
</form>
</fieldset>

</body>
</html>
"""

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
        length = int(self.headers.get("Content-Length",0))
        raw = self.rfile.read(length).decode()
        params = urllib.parse.parse_qs(raw)

        # ===== LASER =====
        if self.path == "/laser":
            cmd = params.get("cmd",["off"])[0]
            if cmd == "on":
                turretmotors.laser_on()
            else:
                turretmotors.laser_off()
            return self.send_html(PAGE)

        # ===== RUN SEQUENCE =====
        if self.path == "/run":
            tid = int(params["tid"][0])

            # run in background
            def worker():
                turretmotors.run_sequence(tid)

            threading.Thread(target=worker, daemon=True).start()
            return self.send_html(PAGE)


# ===== START SERVER FIRST =====
with socketserver.TCPServer(("", PORT), Handler) as server:
    print(f"\nWeb UI running at: http://<your-pi-ip>:{PORT}\n")
    server.serve_forever()
