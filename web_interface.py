#!/usr/bin/env python3
import http.server
import socketserver
import urllib.parse
import threading

import turretmotors   # ← imports your renamed module

PORT = 8000

PAGE = """
<!DOCTYPE html>
<html>
<head>
<title>ENME441 Turret Control</title>
<style>
body { font-family: Arial; max-width: 600px; margin: auto; }
fieldset { padding: 15px; margin-top: 20px; }
label { display:inline-block; width:130px; }
button { padding: 8px 20px; }
</style>
</head>
<body>

<h1>ENME441 Turret Web UI</h1>

<!-- Laser -->
<fieldset>
<legend>Laser Control</legend>
<form method="POST" action="/laser">
<button name="cmd" value="on">Laser ON</button>
<button name="cmd" value="off">Laser OFF</button>
</form>
</fieldset>

<!-- Motors -->
<fieldset>
<legend>Run Motor Sequence</legend>
<form method="POST" action="/run">
<label>Turret ID:</label>
<input type="number" name="tid" required><br><br>
<button>Start Sequence</button>
</form>
</fieldset>

</body>
</html>
"""

class Handler(http.server.BaseHTTPRequestHandler):

    def send_html(self, html):
        html_bytes = html.encode()
        self.send_response(200)
        self.send_header("Content-Type","text/html")
        self.send_header("Content-Length", str(len(html_bytes)))
        self.end_headers()
        self.wfile.write(html_bytes)

    def do_GET(self):
        self.send_html(PAGE)

    def do_POST(self):
        length = int(self.headers.get("Content-Length",0))
        data = self.rfile.read(length).decode()
        params = urllib.parse.parse_qs(data)

        # ---- LASER ----
        if self.path == "/laser":
            cmd = params.get("cmd", ["off"])[0]
            if cmd == "on":
                turretmotors.laser_on()
            else:
                turretmotors.laser_off()
            return self.send_html(PAGE)

        # ---- RUN SEQUENCE ----
        if self.path == "/run":
            tid = int(params["tid"][0])

            def worker():
                turretmotors.run_sequence(tid)

            threading.Thread(target=worker, daemon=True).start()
            return self.send_html(PAGE)


with socketserver.TCPServer(("", PORT), Handler) as server:
    print(f"Web UI running at http://<pi-ip>:{PORT}")
    server.serve_forever()
