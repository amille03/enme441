#!/usr/bin/env python3
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs

# ---------- Hardware wrapper (PWM on Pi; stub elsewhere) ----------
class LEDPWM:
    def __init__(self, pins=(17, 27, 22)):
        self.levels = [0, 0, 0]  # percent 0..100
        self.on_pi = False
        try:
            from gpiozero import PWMLED
            self.leds = [PWMLED(p) for p in pins]
            self.on_pi = True
            print(f"[PWM] Using gpiozero on pins {pins}")
        except Exception as e:
            self.leds = [None, None, None]
            print(f"[PWM] gpiozero not available ({e}); using stub.")

    def set_percent(self, idx, pct):
        pct = max(0, min(100, int(pct)))
        self.levels[idx] = pct
        if self.on_pi:
            self.leds[idx].value = pct / 100.0  # duty cycle 0..1

    def snapshot(self):
        return list(self.levels)

LED = LEDPWM()

# ---------- Simple HTTP app ----------
class Handler(BaseHTTPRequestHandler):
    def _send_html(self, html):
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(html.encode("utf-8"))

    def do_GET(self):
        self._send_html(self.page())

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        data = parse_qs(self.rfile.read(n).decode())
        led_idx = int(data.get("led", ["1"])[0]) - 1
        value = int(data.get("value", ["0"])[0])
        LED.set_percent(led_idx, value)
        self._send_html(self.page(selected=led_idx, msg=f"Set LED {led_idx+1} to {value}%"))

    def page(self, selected=0, msg=""):
        levels = LED.snapshot()
        radios = []
        for i, lv in enumerate(levels, 1):
            chk = "checked" if (i-1) == selected else ""
            radios.append(f'<label style="display:block;margin:.25rem 0">'
                          f'<input type="radio" name="led" value="{i}" {chk}> LED {i} ({lv}%)</label>')
        return f"""<!doctype html>
<html><head><meta charset="utf-8"><title>LED PWM – Problem 1</title>
<style>
 body{{font-family:system-ui,Arial;margin:2rem}} .card{{border:2px solid #333;border-radius:12px;padding:1rem;width:360px}}
 button{{padding:.5rem 1rem;border-radius:8px}}
</style></head><body>
<h2>Problem 1: Single slider + submit (POST)</h2>
<div class="card">
  <form method="POST" action="/">
    <div><strong>Brightness level:</strong><br>
      <input type="range" min="0" max="100" name="value" value="{levels[selected]}">
    </div><br>
    <div><strong>Select LED:</strong><div>{''.join(radios)}</div></div>
    <button type="submit">Change Brightness</button>
  </form>
  <p style="color:#2b6">{msg}</p>
  <hr>
  <div><em>Current levels</em></div>
  <div>LED1: {levels[0]}% | LED2: {levels[1]}% | LED3: {levels[2]}%</div>
</div>
<p><a href="/v2">Looking for the JS version? (use problem2_rpi.py)</a></p>
</body></html>"""

if __name__ == "__main__":
    HTTPServer(("0.0.0.0", 8000), Handler).serve_forever()
