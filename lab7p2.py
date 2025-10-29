#!/usr/bin/env python3
from http.server import BaseHTTPRequestHandler, HTTPServer
import json

# ---------- Hardware wrapper ----------
class LEDPWM:
    def __init__(self, pins=(17, 27, 22)):
        self.levels = [0, 0, 0]
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
            self.leds[idx].value = pct / 100.0

    def snapshot(self):
        return list(self.levels)

LED = LEDPWM()

# ---------- HTTP app ----------
class Handler(BaseHTTPRequestHandler):
    def _send_html(self, html):
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(html.encode("utf-8"))

    def _send_json(self, obj, status=200):
        b = (json.dumps(obj)).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        self._send_html(self.page())

    def do_POST(self):
        # JSON body: {"led": 0..2, "value": 0..100}
        n = int(self.headers.get("Content-Length", 0))
        try:
            data = json.loads(self.rfile.read(n).decode())
            idx = int(data.get("led", 0))
            val = int(data.get("value", 0))
            LED.set_percent(idx, val)
            self._send_json({"ok": True, "levels": LED.snapshot()})
        except Exception as e:
            self._send_json({"ok": False, "error": str(e)}, status=400)

    def page(self):
        l = LED.snapshot()
        return f"""<!doctype html>
<html><head><meta charset="utf-8"><title>LED PWM – Problem 2</title>
<style>
 body{{font-family:system-ui,Arial;margin:2rem}} .card{{border:2px solid #333;border-radius:12px;padding:1rem;width:420px}}
 .row{{display:flex;align-items:center;justify-content:space-between;margin:.75rem 0;gap:.5rem}}
 .name{{width:48px}} .val{{width:36px;text-align:right;font-weight:600}}
 input[type=range]{{flex:1}}
</style></head><body>
<h2>Problem 2: Three sliders (auto-POST, no reload)</h2>
<div class="card">
  <div class="row"><span class="name">LED1</span>
    <input id="s1" type="range" min="0" max="100" value="{l[0]}"><span id="v1" class="val">{l[0]}</span></div>
  <div class="row"><span class="name">LED2</span>
    <input id="s2" type="range" min="0" max="100" value="{l[1]}"><span id="v2" class="val">{l[1]}</span></div>
  <div class="row"><span class="name">LED3</span>
    <input id="s3" type="range" min="0" max="100" value="{l[2]}"><span id="v3" class="val">{l[2]}</span></div>
  <div id="status" style="color:#2b6;min-height:1.2em"></div>
</div>
<script>
function send(led, value){{
  fetch("/", {{
    method:"POST",
    headers:{{"Content-Type":"application/json"}},
    body:JSON.stringify({{led:led, value:value}})
  }})
  .then(r=>r.json())
  .then(d=>{{
    if(d.ok){{
      document.getElementById("v1").textContent=d.levels[0];
      document.getElementById("v2").textContent=d.levels[1];
      document.getElementById("v3").textContent=d.levels[2];
      document.getElementById("status").textContent="Updated.";
    }} else {{
      document.getElementById("status").textContent="Error: "+(d.error||"");
    }}
  }})
  .catch(()=>{{document.getElementById("status").textContent="Network error";}});
}}
[["s1",0],["s2",1],["s3",2]].forEach(([id,i])=>{
  const el=document.getElementById(id);
  el.addEventListener("input", e=>{
    const val=parseInt(e.target.value)||0;
    document.getElementById("v"+(i+1)).textContent=val;
    send(i, val);
  });
});
</script>
</body></html>"""

if __name__ == "__main__":
    HTTPServer(("0.0.0.0", 8000), Handler).serve_forever()
