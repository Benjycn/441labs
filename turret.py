################################################################################
# WEB + STEPPER MOTORS + LASER CONTROL SYSTEM
# Uses TCP/IP sockets, HTML+JS interface, multiprocessing queue motor control,
# JSON turret position loading, and Raspberry Pi GPIO for laser enable.
################################################################################

import RPi.GPIO as GPIO
import multiprocessing
import threading
import socket
import json
import time
from ctypes import c_double
from Shifter import shifter  # Your custom shift-register class

# ----------------------------- GPIO SETUP ------------------------------------
GPIO.setmode(GPIO.BCM)
LASER_PIN = 22
GPIO.setup(LASER_PIN, GPIO.OUT)
GPIO.output(LASER_PIN, GPIO.LOW)  # Laser OFF by default

# ------------------------- GLOBAL MOTOR MEMORY --------------------------------
myArray = multiprocessing.Array('i', 2)  # two motors = 2 integers


# ------------------------- STEPPER MOTOR CLASS -------------------------------
class Stepper:
    seq = [0b0001,0b0011,0b0010,0b0110,
           0b0100,0b1100,0b1000,0b1001]   # CCW sequence
    delay = 8000                         # us (faster)
    steps_per_degree = 1024 / 360        # 28BYJ-48 but with 1:16 gearbox

    def __init__(self, shifter, lock, index):
        self.s = shifter
        self.lock = lock
        self.index = index
        self.step_state = 0
        self.shifter_bit_start = 4 * index

        self.angle = multiprocessing.Value(c_double, 0.0)
        self.q = multiprocessing.Queue()

        # Motor command processor
        self.proc = multiprocessing.Process(target=self._run)
        self.proc.daemon = True
        self.proc.start()

    def _sgn(self, x):
        return 0 if x == 0 else int(abs(x)/x)

    def _step(self, direction):
        with self.lock:
            self.step_state = (self.step_state + direction) % 8
            pattern = Stepper.seq[self.step_state]

            # Update motor's 4 bits in shared array
            myArray[self.index] &= ~(0b1111 << self.shifter_bit_start)
            myArray[self.index] |= (pattern << self.shifter_bit_start)

            # Combine final register output
            final = 0
            for val in myArray:
                final |= val

            self.s.shiftByte(final)

            # update shared angle
            with self.angle.get_lock():
                self.angle.value = (self.angle.value +
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
        with self.angle.get_lock():
            self.angle.value = 0.0

    def goAngle(self, target):
        with self.angle.get_lock():
            curr = self.angle.value % 360

        target %= 360
        diff = target - curr

        # Shortest path
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360

        self.q.put(diff)


# -------------------------- LOAD JSON LOCATION --------------------------------
def load_turret_json():
    try:
        with open("turret.json", "r") as f:
            return json.load(f)
    except:
        return {"azimuth": 0, "altitude": 0}


# ------------------------------- WEB PAGE ------------------------------------
def build_webpage(az, alt, laser_state):
    return f"""
<html>
<head>
<style>
.container {{
    width: 300px;
    padding: 12px;
    border: 2px solid #333;
    border-radius: 8px;
    font-family: Arial;
}}
.label {{
    font-size: 14px;
    margin-top: 10px;
}}
</style>

<script>
function sendCommand(cmd) {{
    var xhr = new XMLHttpRequest();
    xhr.open("POST", "/", true);
    xhr.send(cmd);
}}

function setAz(val) {{
    document.getElementById("az_val").innerHTML = val;
    sendCommand("azimuth=" + val);
}}

function setAlt(val) {{
    document.getElementById("alt_val").innerHTML = val;
    sendCommand("altitude=" + val);
}}

function laserOn()  {{ sendCommand("laser=on");  }}
function laserOff() {{ sendCommand("laser=off"); }}

function zero() {{ sendCommand("zero=1"); }}

function loadJSON() {{
    sendCommand("loadjson=1");
}}
</script>
</head>

<body>
<div class="container">
<h3>Stepper Turret Control</h3>

<div class="label">Laser:</div>
<button onclick="laserOn()">ON</button>
<button onclick="laserOff()">OFF</button>
<p>Laser State: <b>{laser_state}</b></p>

<div class="label">Azimuth ({az}°)</div>
<input type="range" min="0" max="360" value="{az}"
       oninput="setAz(this.value)">
<span id="az_val">{az}</span>

<div class="label">Altitude ({alt}°)</div>
<input type="range" min="0" max="360" value="{alt}"
       oninput="setAlt(this.value)">
<span id="alt_val">{alt}</span>

<br><br>
<button onclick="zero()">Set Zero</button>
<button onclick="loadJSON()">Go to JSON Location</button>

</div>
</body>
</html>
"""


# ------------------------------ SOCKET SERVER ---------------------------------
def parsePOSTdata(data):
    """Extract key/value pairs manually from raw POST."""
    try:
        body = data.split("\r\n\r\n",1)[1]
        params = {}
        for pair in body.split("&"):
            if "=" in pair:
                k,v = pair.split("=")
                params[k] = v
        return params
    except:
        return {}


def web_server():
    global az, alt, laser_state

    while True:
        conn, addr = server.accept()
        msg = conn.recv(2048).decode()

        cmd = parsePOSTdata(msg)

        # ------------ LASER CONTROL ------------
        if "laser" in cmd:
            if cmd["laser"] == "on":
                GPIO.output(LASER_PIN, GPIO.HIGH)
                laser_state = "ON"
            elif cmd["laser"] == "off":
                GPIO.output(LASER_PIN, GPIO.LOW)
                laser_state = "OFF"

        # ----------- MANUAL ANGLE CONTROL -------
        if "azimuth" in cmd:
            az = int(cmd["azimuth"])
            motor_az.goAngle(az)

        if "altitude" in cmd:
            alt = int(cmd["altitude"])
            motor_alt.goAngle(alt)

        # -------------- ZERO REFERENCE ----------
        if "zero" in cmd:
            motor_az.zero()
            motor_alt.zero()
            az, alt = 0, 0

        # -------------- LOAD JSON LOCATION ------
        if "loadjson" in cmd:
            data = load_turret_json()
            az = data["azimuth"]
            alt = data["altitude"]
            motor_az.goAngle(az)
            motor_alt.goAngle(alt)

        # -------------- SEND PAGE ---------------
        page = build_webpage(az, alt, laser_state)
        conn.send(b"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n")
        conn.send(page.encode())
        conn.close()


# ------------------------------- MAIN -----------------------------------------
if __name__ == "__main__":

    # Shift register + multiprocessing lock
    s = shifter(16, 21, 20)
    lock = multiprocessing.Lock()

    # Two motors: azimuth and altitude
    motor_az  = Stepper(s, lock, 0)
    motor_alt = Stepper(s, lock, 1)

    # Initial turret angles
    az  = 0
    alt = 0
    laser_state = "OFF"

    # Socket server on port 80
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("",80))
    server.listen(3)

    print("Web control server running at http://<Pi_IP>/")

    # Thread for web server
    t = threading.Thread(target=web_server, daemon=True)
    t.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("\nSystem shutting down.")
