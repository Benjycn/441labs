################################################################################
# WEB + STEPPER MOTORS + LASER CONTROL SYSTEM (NO CTYPES VERSION)
#
# Uses:
#   - Two 28BYJ-48 steppers using shift register
#   - TCP Web Interface (HTML sliders + buttons)
#   - Laser ON/OFF
#   - Zero motors
#   - Loads JSON turret angle from remote server
#   - multiprocessing.Manager().dict() instead of ctypes
################################################################################

import RPi.GPIO as GPIO
import multiprocessing
import threading
import socket
import json
import urllib.request
import time
from Shifter import shifter


# ----------------------------- CONFIGURATION ---------------------------------
TEAM_ID = "1"   # <<< CHANGE THIS FOR YOUR TEAM
JSON_URL = "http://192.168.1.254:8000/positions.json"

GPIO.setmode(GPIO.BCM)
LASER_PIN = 22
GPIO.setup(LASER_PIN, GPIO.OUT)
GPIO.output(LASER_PIN, GPIO.LOW)

# Global register (2 motors → 2 integers)
myArray = multiprocessing.Array('i', 2)


# ---------------------------- STEPPER CLASS ----------------------------------
class Stepper:
    seq = [0b0001,0b0011,0b0010,0b0110,
           0b0100,0b1100,0b1000,0b1001]

    delay = 8000
    steps_per_degree = 1024 / 360   # 1:16 gearbox

    def __init__(self, sh, lock, index, shared_angles):
        self.s = sh
        self.lock = lock
        self.index = index
        self.step_state = 0
        self.shared = shared_angles
        self.shifter_bit_start = 4 * index

        # Command queue (each motor runs in background)
        self.q = multiprocessing.Queue()

        self.proc = multiprocessing.Process(target=self._run)
        self.proc.daemon = True
        self.proc.start()

    def _sgn(self, x):
        return 1 if x > 0 else -1 if x < 0 else 0

    def _step(self, direction):
        with self.lock:
            # Update step index
            self.step_state = (self.step_state + direction) % 8
            pattern = Stepper.seq[self.step_state]

            # Update shared nibble
            myArray[self.index] &= ~(0b1111 << self.shifter_bit_start)
            myArray[self.index] |= (pattern << self.shifter_bit_start)

            # Combine full byte
            final = 0
            for v in myArray:
                final |= v

            self.s.shiftByte(final)

            # Update angle (NO ctypes)
            self.shared[self.index] = (self.shared[self.index] +
                                       direction / Stepper.steps_per_degree) % 360

        time.sleep(Stepper.delay / 1e6)

    def _rotate(self, delta):
        steps = int(abs(delta) * Stepper.steps_per_degree)
        direction = self._sgn(delta)
        for _ in range(steps):
            self._step(direction)

    def _run(self):
        while True:
            delta = self.q.get()
            self._rotate(delta)

    def zero(self):
        self.shared[self.index] = 0.0

    def goAngle(self, target):
        curr = self.shared[self.index]
        target %= 360
        diff = target - curr

        # Shortest rotation
        if diff > 180: diff -= 360
        if diff < -180: diff += 360

        self.q.put(diff)


# ---------------------- LOAD TURRET JSON FROM NETWORK -------------------------
def load_json_position():
    """
    Loads JSON from ENME server.
    Extracts theta for this team's turret.
    """
    try:
        with urllib.request.urlopen(JSON_URL, timeout=5) as response:
            data = json.loads(response.read().decode())

        # Example structure:
        # {"turrets":{"1":{"theta":1.23}}}
        theta_rad = data["turrets"][TEAM_ID]["theta"]
        theta_deg = theta_rad * 180 / 3.14159265

        print(f"[JSON] θ(rad)={theta_rad}, θ(deg)={theta_deg}")

        return {"azimuth": theta_deg, "altitude": 0}

    except Exception as e:
        print("[ERROR loading JSON]", e)
        return {"azimuth": 0, "altitude": 0}


# ------------------------------- WEB PAGE ------------------------------------
def build_webpage(az, alt, laser_state):
    return f"""
<html>
<head>
<style>
body {{ font-family: Arial; }}
.container {{
    width: 300px; padding: 12px;
    border: 2px solid #333; border-radius: 8px;
}}
</style>

<script>
function send(cmd) {{
    var x = new XMLHttpRequest();
    x.open("POST", "/", true);
    x.send(cmd);
}}

function setAz(v) {{
    document.getElementById("az_val").innerHTML = v;
    send("azimuth=" + v);
}}

function setAlt(v) {{
    document.getElementById("alt_val").innerHTML = v;
    send("altitude=" + v);
}}

function laserOn()  {{ send("laser=on"); }}
function laserOff() {{ send("laser=off"); }}
function zero()     {{ send("zero=1"); }}
function loadJSON() {{ send("loadjson=1"); }}
</script>
</head>

<body>
<div class="container">

<h2>Turret Control</h2>

<h3>Laser</h3>
<button onclick="laserOn()">ON</button>
<button onclick="laserOff()">OFF</button>
<p>Laser State: <b>{laser_state}</b></p>

<h3>Azimuth ({az}°)</h3>
<input type="range" min="0" max="360" value="{az}" oninput="setAz(this.value)">
<span id="az_val">{az}</span>

<h3>Altitude ({alt}°)</h3>
<input type="range" min="0" max="360" value="{alt}" oninput="setAlt(this.value)">
<span id="alt_val">{alt}</span>

<br><br>
<button onclick="zero()">Zero</button>
<button onclick="loadJSON()">Go to JSON</button>

</div>
</body>
</html>
"""


# ------------------------------ SOCKET PARSER ---------------------------------
def parsePOSTdata(data):
    try:
        body = data.split("\r\n\r\n", 1)[1]
        params = {}
        for pair in body.split("&"):
            if "=" in pair:
                k, v = pair.split("=")
                params[k] = v
        return params
    except:
        return {}


# ------------------------------ WEB SERVER ------------------------------------
def web_server():
    global laser_state

    while True:
        conn, addr = server.accept()
        msg = conn.recv(2048).decode()
        cmd = parsePOSTdata(msg)

        # Laser control
        if "laser" in cmd:
            if cmd["laser"] == "on":
                GPIO.output(LASER_PIN, GPIO.HIGH)
                laser_state = "ON"
            else:
                GPIO.output(LASER_PIN, GPIO.LOW)
                laser_state = "OFF"

        # Manual azimuth
        if "azimuth" in cmd:
            a = int(cmd["azimuth"])
            shared_angles[0] = a
            motor_az.goAngle(a)

        # Manual altitude
        if "altitude" in cmd:
            b = int(cmd["altitude"])
            shared_angles[1] = b
            motor_alt.goAngle(b)

        # Zero
        if "zero" in cmd:
            motor_az.zero()
            motor_alt.zero()
            shared_angles[0] = 0
            shared_angles[1] = 0

        # Load JSON
        if "loadjson" in cmd:
            data = load_json_position()
            shared_angles[0] = data["azimuth"]
            shared_angles[1] = data["altitude"]
            motor_az.goAngle(shared_angles[0])
            motor_alt.goAngle(shared_angles[1])

        # Send web page
        page = build_webpage(shared_angles[0], shared_angles[1], laser_state)
        conn.send(b"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n")
        conn.send(page.encode())
        conn.close()


# ----------------------------------- MAIN -------------------------------------
if __name__ == "__main__":

    # Shared dictionary instead of ctypes
    manager = multiprocessing.Manager()
    shared_angles = manager.dict({0: 0.0, 1: 0.0})

    s = shifter(16, 21, 20)
    lock = multiprocessing.Lock()

    motor_az = Stepper(s, lock, 0, shared_angles)
    motor_alt = Stepper(s, lock, 1, shared_angles)

    laser_state = "OFF"

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("", 80))
    server.listen(3)

    print("Web control interface running on http://<Pi_IP>/")

    threading.Thread(target=web_server, daemon=True).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("\nSystem shut down.")
