#!/usr/bin/env python3
# EMG Angle Control System (Online LDA) + DC Motor Controller (Pi 5 + L298N + Encoder)
# Single file: trains/loads LDA model, predicts joint angle from EMG UDP, and commands DC motor to that angle.
#
# Notes:
# - Requires: lgpio, numpy, joblib, scikit-learn
# - Wiring (BCM): IN1=24, IN2=23, ENA=25, ENC_A=17, ENC_B=27 (edit below if different)

import socket, time, os, math, threading, queue
import numpy as np
from collections import deque
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.preprocessing import StandardScaler
import joblib

# =========================
# ===== LDA CONFIG ========
# =========================
DATA_FILE_X = "emg_history_features.npy"
DATA_FILE_Y = "emg_history_labels.npy"
MODEL_FILE  = "lda_model_pro.joblib"
SCALER_FILE = "scaler_pro.joblib"

UDP_IP   = "0.0.0.0"
UDP_PORT = 5005
FS     = 1000         # sampling frequency (Hz)
WIN_N  = 200          # 200 ms window @ 1 kHz
STEP_N = 50           # 50 ms update

# Output classes (angles in degrees)
CLASSES = ["0", "45", "90", "-45", "-90"]
SECONDS_PER_CLASS = 4

# Prediction gating
CONF_THRESH = 0.40        # only command motor if confidence >= this
MIN_CMD_INTERVAL = 0.10   # seconds between motor commands
SEND_ONLY_ON_CHANGE = True

# =========================
# == FEATURE EXTRACTION ===
# =========================
def extract_features(s1, s2):
    # DC offset removal
    s1 = s1 - np.mean(s1)
    s2 = s2 - np.mean(s2)

    def td_feats(x):
        return [
            np.mean(np.abs(x)),          # MAV
            np.sqrt(np.mean(x**2)),      # RMS
            np.sum(np.abs(np.diff(x))),  # WL
            np.var(x)                    # VAR
        ]
    return td_feats(s1) + td_feats(s2)

# =========================
# ===== CONTROLLER ========
# =========================
# Wrapped from your controller.py into functions, so ML output can call the motor controller.

import lgpio as lg

# ---- Pins (BCM) ----
IN1, IN2, ENA = 24, 23, 25
ENC_A, ENC_B  = 17, 27

# ---- Encoder params ----
CPR_OUT    = 1968     # counts per output shaft revolution (after gearbox). Calibrate if needed.
ENC_INVERT = False    # True if angle grows in wrong direction

# ---- Allowed angle range ----
MIN_ANGLE = -90.0
MAX_ANGLE =  90.0

# ---- PWM & motor ----
PWM_FREQ  = 1800      # 1.8 kHz
DEADBAND  = 0.06

# ---- Control loop ----
DT        = 0.002     # 500 Hz
STOP_BAND = 0.5       # deg
HOLD_TIME = 0.50      # s inside band
MAX_SECONDS = 8.0     # safety timeout

# Timed move option: None = hold-at-target, value = move for N seconds then stop
MOVE_FOR_SECONDS = 1.0

# ---- STSMC tuning ----
LAMBDA = 6.0
K1 = 0.020
K2 = 0.005
PHI = 5.0

U_MAX, U_MIN = 1.0, -1.0
U_GLOBAL_GAIN = 1.0

CTRL_INVERT = True

# smoothing / limiting
U_LIMIT = 1.0
DU_MAX = 2.5
SOFTSTART_SEC = 0.35

# optional assists
K_GRAV = 0.40
KICK_MIN = 0.25
KICK_E_DEG = 2.0
KICK_DE_DPS = 15.0

# ---- Controller runtime globals (initialized in controller_init) ----
_ctrl = {
    "h": None,
    "pwm": None,
    "enc": None,
    "worker": None,
    "cmd_queue": queue.Queue(maxsize=8),
    "abort": threading.Event(),
    "ready": False,
}

# Quadrature transition table
_delta_table = [
    [ 0, -1, +1,  0],
    [+1,  0,  0, -1],
    [-1,  0,  0, +1],
    [ 0, +1, -1,  0],
]

class PWMThread(threading.Thread):
    def __init__(self, handle, pin, freq):
        super().__init__(daemon=True)
        self.h = handle
        self.pin = pin
        self.T = 1.0 / float(freq)
        self.duty = 0.0
        self._lock = threading.Lock()
        self._stop = False
        self._next = time.monotonic()

    def set_duty(self, duty):
        if not isinstance(duty, (int, float)) or duty != duty or math.isinf(duty):
            duty = 0.0
        duty = 0.0 if duty < 0 else 1.0 if duty > 1.0 else float(duty)
        with self._lock:
            self.duty = duty

    def stop(self):
        self._stop = True

    def run(self):
        on = False
        while not self._stop:
            now = time.monotonic()
            if now < self._next:
                time.sleep(0.0002)
                continue
            with self._lock:
                d = self.duty
            T = self.T

            if d <= 0.0:
                lg.gpio_write(self.h, self.pin, 0)
                on = False
                self._next = now + T
            elif d >= 1.0:
                lg.gpio_write(self.h, self.pin, 1)
                on = True
                self._next = now + T
            else:
                if not on:
                    lg.gpio_write(self.h, self.pin, 1)
                    on = True
                    self._next = now + d * T
                else:
                    lg.gpio_write(self.h, self.pin, 0)
                    on = False
                    self._next = now + (1.0 - d) * T

class EncoderThread(threading.Thread):
    def __init__(self, handle, a_pin, b_pin, invert=False):
        super().__init__(daemon=True)
        self.h, self.a, self.b = handle, a_pin, b_pin
        a0 = lg.gpio_read(handle, a_pin)
        b0 = lg.gpio_read(handle, b_pin)
        self.state = (a0 << 1) | b0
        self.count = 0
        self.sign = -1 if invert else 1
        self._lock = threading.Lock()
        self._stop = False

    def reset(self, value=0):
        with self._lock:
            self.count = int(value)

    def get_count(self):
        with self._lock:
            return int(self.count)

    def stop(self):
        self._stop = True

    def run(self):
        while not self._stop:
            a = lg.gpio_read(self.h, self.a)
            b = lg.gpio_read(self.h, self.b)
            s = (a << 1) | b
            d = _delta_table[self.state][s]
            if d:
                with self._lock:
                    self.count += self.sign * d
            self.state = s
            time.sleep(0)

def _counts_to_deg(c):
    return (float(c) * 360.0) / float(CPR_OUT)

def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def _sat(x, phi):
    if phi <= 1e-9:
        return 1.0 if x > 0 else (-1.0 if x < 0 else 0.0)
    if x >  phi: return  1.0
    if x < -phi: return -1.0
    return x / phi

def _stsmc_step(s, dt, v_int):
    sigma = _sat(s, PHI)
    v_int += (K2 * sigma) * dt
    u = -(K1 * math.sqrt(abs(s) + 1e-12) * sigma + v_int)
    u *= U_GLOBAL_GAIN
    if not isinstance(u, (int, float)) or u != u or math.isinf(u):
        u = 0.0
    u = max(U_MIN, min(U_MAX, u))
    return u, v_int

def _set_motor(u):
    h = _ctrl["h"]
    pwm = _ctrl["pwm"]
    if h is None or pwm is None:
        return

    if not isinstance(u, (int, float)) or u != u or math.isinf(u):
        u = 0.0
    u = max(-1.0, min(1.0, float(u)))

    if abs(u) < DEADBAND:
        lg.gpio_write(h, IN1, 0)
        lg.gpio_write(h, IN2, 0)
        pwm.set_duty(0.0)
        return

    if u >= 0:
        lg.gpio_write(h, IN1, 1)
        lg.gpio_write(h, IN2, 0)
        pwm.set_duty(u)
    else:
        lg.gpio_write(h, IN1, 0)
        lg.gpio_write(h, IN2, 1)
        pwm.set_duty(-u)

def controller_init():
    """Initialize GPIO, PWM thread, encoder thread, and command worker."""
    if _ctrl["ready"]:
        return

    h = lg.gpiochip_open(0)
    lg.gpio_claim_output(h, IN1, 0)
    lg.gpio_claim_output(h, IN2, 0)
    lg.gpio_claim_output(h, ENA, 0)
    lg.gpio_claim_input(h, ENC_A)
    lg.gpio_claim_input(h, ENC_B)

    pwm = PWMThread(h, ENA, PWM_FREQ)
    pwm.start()

    enc = EncoderThread(h, ENC_A, ENC_B, invert=ENC_INVERT)
    enc.start()

    _ctrl["h"] = h
    _ctrl["pwm"] = pwm
    _ctrl["enc"] = enc

    _ctrl["worker"] = _CommandWorker()
    _ctrl["worker"].start()

    _ctrl["ready"] = True

def controller_zero_here():
    """Set current encoder position as 0 degrees."""
    if not _ctrl["ready"]:
        controller_init()
    _ctrl["enc"].reset(0)

def controller_shutdown():
    """Stop motor and release GPIO cleanly."""
    try:
        _ctrl["abort"].set()
        if _ctrl["worker"] is not None:
            _ctrl["worker"].stop()
        _set_motor(0.0)
        if _ctrl["pwm"] is not None:
            _ctrl["pwm"].stop()
        if _ctrl["enc"] is not None:
            _ctrl["enc"].stop()
        time.sleep(0.05)
        h = _ctrl["h"]
        if h is not None:
            lg.gpio_write(h, IN1, 0)
            lg.gpio_write(h, IN2, 0)
            lg.gpio_write(h, ENA, 0)
            lg.gpiochip_close(h)
    finally:
        _ctrl["ready"] = False

def controller_command_angle(target_deg):
    """Public function: receive ML output (angle) and send it to the motor controller."""
    if not _ctrl["ready"]:
        controller_init()

    # Clear queue and push latest command (so motor always follows newest intent)
    q = _ctrl["cmd_queue"]
    try:
        while not q.empty():
            q.get_nowait()
    except Exception:
        pass

    try:
        q.put_nowait(("angle", float(target_deg)))
    except queue.Full:
        # if queue is full, just drop older command; next loop will try again
        pass

def _move_to_angle_abs(target_deg_any):
    """Blocking move: move to any reference angle in [-90, +90]."""
    target = _clamp(float(target_deg_any), MIN_ANGLE, MAX_ANGLE)
    T_ref = target

    enc = _ctrl["enc"]
    if enc is None:
        return

    e_prev = 0.0
    t_prev = time.monotonic()
    start  = t_prev
    in_band_since = None

    v_int = 0.0
    u_prev = 0.0
    _ctrl["abort"].clear()

    while True:
        if _ctrl["abort"].is_set():
            break

        now = time.monotonic()
        if (now - t_prev) < DT:
            time.sleep(0.0002)
            continue

        dt = now - t_prev
        if dt <= 0: dt = DT
        if dt > 0.05: dt = DT

        theta = _counts_to_deg(enc.get_count())
        e  = (T_ref - theta)
        de = (e - e_prev) / dt
        s  = de + LAMBDA * e

        u_cmd, v_int = _stsmc_step(s, dt, v_int)

        if CTRL_INVERT:
            u_cmd = -u_cmd

        if K_GRAV != 0.0:
            u_cmd += K_GRAV * math.sin(math.radians(theta))

        if abs(e) > KICK_E_DEG and abs(de) < KICK_DE_DPS and abs(u_cmd) < KICK_MIN:
            u_cmd = math.copysign(KICK_MIN, e)

        # soft-start + slew-rate
        elapsed = now - start
        lim = U_LIMIT * (min(1.0, elapsed / SOFTSTART_SEC) if SOFTSTART_SEC > 1e-6 else 1.0)
        u_cmd = max(-lim, min(lim, u_cmd))

        du = u_cmd - u_prev
        max_du = DU_MAX * dt
        if du >  max_du: u_cmd = u_prev + max_du
        if du < -max_du: u_cmd = u_prev - max_du
        u_prev = u_cmd

        u_cmd = max(U_MIN, min(U_MAX, u_cmd))
        _set_motor(u_cmd)

        # stop logic
        if MOVE_FOR_SECONDS is not None:
            if (now - start) >= MOVE_FOR_SECONDS:
                break
        else:
            if abs(e) <= STOP_BAND:
                if in_band_since is None:
                    in_band_since = now
                elif (now - in_band_since) >= HOLD_TIME:
                    break
            else:
                in_band_since = None

            if (now - start) > MAX_SECONDS:
                break

        e_prev = e
        t_prev = now

    _set_motor(0.0)

class _CommandWorker(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self._stop = False

    def stop(self):
        self._stop = True
        _ctrl["abort"].set()

    def run(self):
        q = _ctrl["cmd_queue"]
        while not self._stop:
            try:
                kind, payload = q.get(timeout=0.2)
            except queue.Empty:
                continue
            if kind == "angle":
                try:
                    _move_to_angle_abs(payload)
                except Exception as e:
                    print(f"[controller] move error: {e}")

# =========================
# ===== UDP SOCKET =========
# =========================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(1.0)

# =========================
# ===== TRAIN ONLINE =======
# =========================
def train_online():
    # Load history if exists
    if os.path.exists(DATA_FILE_X) and os.path.exists(DATA_FILE_Y):
        X_history = list(np.load(DATA_FILE_X))
        y_history = list(np.load(DATA_FILE_Y))
        print(f"[INFO] Loaded {len(X_history)} samples from history.")
    else:
        X_history, y_history = [], []
        print("[INFO] No history found. Starting fresh.")

    X_session, y_session = [], []
    print("\n--- Starting Angle Calibration ---")

    for idx, angle in enumerate(CLASSES):
        print(f"\n[GET READY] Hold Angle: {angle} degrees")
        for countdown in range(3, 0, -1):
            print(f"Recording in {countdown}...")
            time.sleep(1)

        print(f">>> RECORDING: {angle}!")
        start_t = time.time()
        buf1, buf2 = deque(maxlen=WIN_N), deque(maxlen=WIN_N)
        captured_count = 0

        while time.time() - start_t < SECONDS_PER_CLASS:
            try:
                data, _ = sock.recvfrom(4096)
                text = data.decode("utf-8", errors="ignore")

                # Parse batch UDP: "id;v1,v2;v1,v2..."
                for line in text.splitlines():
                    parts = line.strip().split(';')
                    if len(parts) < 2:
                        continue

                    for p in parts[1:]:
                        vv = p.split(',')
                        if len(vv) != 2:
                            continue
                        buf1.append(float(vv[0]))
                        buf2.append(float(vv[1]))

                        if len(buf1) == WIN_N:
                            X_session.append(extract_features(np.array(buf1), np.array(buf2)))
                            y_session.append(idx)
                            captured_count += 1
                            # overlap shift
                            for _ in range(STEP_N):
                                if buf1: buf1.popleft()
                                if buf2: buf2.popleft()
            except socket.timeout:
                continue

        print(f"[DONE] Captured {captured_count} samples for {angle}°")

    if len(X_session) == 0 and len(X_history) == 0:
        print("[ERROR] No data collected. Check UDP stream.")
        return None, None

    X_combined = np.array(X_history + X_session)
    y_combined = np.array(y_history + y_session)
    np.save(DATA_FILE_X, X_combined)
    np.save(DATA_FILE_Y, y_combined)

    print("\n[PROCESS] Training LDA Model...")
    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X_combined)
    lda = LinearDiscriminantAnalysis()
    lda.fit(X_scaled, y_combined)

    joblib.dump(lda, MODEL_FILE)
    joblib.dump(scaler, SCALER_FILE)
    print(f"[SUCCESS] Total Dataset Size: {len(X_combined)} samples.")
    return lda, scaler

# =========================
# ========= MAIN ==========
# =========================
def main():
    print("--- EMG Angle Control System (LDA) + Motor Controller ---")
    print("1) Train/Improve Model")
    print("2) Real-time Predict + Move Motor")
    choice = input("Select Option: ").strip()

    if choice == "1":
        lda, scaler = train_online()
        if lda is None:
            return
        print("[OK] Training done. You can run option 2 now.")
        return

    # choice 2
    try:
        lda = joblib.load(MODEL_FILE)
        scaler = joblib.load(SCALER_FILE)
        print("[OK] Loaded trained model.")
    except Exception:
        print("[ERROR] No model found. Run Option 1 first.")
        return

    # Start controller
    controller_init()
    print("\nPlace joint at physical 0° then press ENTER to zero encoder (recommended).")
    input("")
    controller_zero_here()
    print("[OK] Encoder zeroed. Real-time control active. Ctrl+C to stop.\n")

    buf1, buf2 = deque(maxlen=WIN_N), deque(maxlen=WIN_N)
    samples_count = 0
    last_cmd_time = 0.0
    last_sent_angle = None

    try:
        while True:
            try:
                data, _ = sock.recvfrom(4096)
                text = data.decode("utf-8", errors="ignore")

                for line in text.splitlines():
                    parts = line.strip().split(';')
                    if len(parts) < 2:
                        continue

                    for p in parts[1:]:
                        vv = p.split(',')
                        if len(vv) != 2:
                            continue

                        buf1.append(float(vv[0]))
                        buf2.append(float(vv[1]))
                        samples_count += 1

                        if len(buf1) == WIN_N and samples_count >= STEP_N:
                            feat = extract_features(np.array(buf1), np.array(buf2))
                            feat_scaled = scaler.transform([feat])

                            pred = int(lda.predict(feat_scaled)[0])
                            prob = float(np.max(lda.predict_proba(feat_scaled)))
                            angle_str = CLASSES[pred]

                            if prob >= CONF_THRESH:
                                angle_deg = float(angle_str)
                                print(f"Angle: {angle_str:>4}° | Conf: {prob:.1%}")

                                now = time.time()
                                allow_time = (now - last_cmd_time) >= MIN_CMD_INTERVAL
                                allow_change = (not SEND_ONLY_ON_CHANGE) or (last_sent_angle != angle_deg)

                                if allow_time and allow_change:
                                    controller_command_angle(angle_deg)
                                    last_cmd_time = now
                                    last_sent_angle = angle_deg
                            else:
                                print(f"Angle: Uncertain | Conf: {prob:.1%}")

                            samples_count = 0

            except socket.timeout:
                continue

    except KeyboardInterrupt:
        print("\n[EXIT] Stopping...")

    finally:
        controller_shutdown()

if __name__ == "__main__":
    try:
        main()
    finally:
        try:
            sock.close()
        except Exception:
            pass
