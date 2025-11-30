#!/usr/bin/env python3
import http.server
import socketserver
import urllib.parse

PORT = 8000

PAGE = """
<!DOCTYPE html>
<html>
<head>
<title>Test Web UI</title>
<style>
body { font-family: Arial; max-width: 600px; margin:auto; }
fieldset { margin:20px 0; padding:15px; }
</style>
</head>
<body>

<h1>Test Web Interface</h1>

<fieldset>
<legend>Laser Test</legend>
<form method="POST" action="/laser">
<button name="state" value="on">Laser ON</button>
<button name="state" value="off">Laser OFF</button>
</form>
</fieldset>

<fieldset>
<legend>Motor Test</legend>
<form method="POST" action="/move">
<label>Turret (deg):</label>
<input name="turret" type="number"><br><br>
<label>Globe (deg):</label>
<input name="globe" type="number"><br><br>
<button>Submit Move</button>
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
        print("[SERVER] GET request received")
        self.send_html(PAGE)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        data = self.rfile.read(length).decode()
        params = urllib.parse.parse_qs(data)

        print("\n===== POST Received =====")
        print("Path:", self.path)
        print("Parameters:", params)
        print("=========================\n")

        self.send_html(PAGE)

# ---- MAIN SERVER START ----
with socketserver.TCPServer(("", PORT), Handler) as server:
    print(f"Web UI running at: http://<your_pi_ip>:{PORT}")
    print("Press CTRL+C to stop the server.\n")
    server.serve_forever()
