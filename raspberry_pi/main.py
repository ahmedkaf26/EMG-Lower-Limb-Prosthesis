#!/usr/bin/env python3
# EMG Angle Control System (Online LDA) + DC Motor Position Controller (Pi 5 + L298N + Quadrature Encoder)
# Single-file workflow:
#  1) Option 1: collect EMG calibration data (UDP), update history, train LDA + scaler
#  2) Option 2: real-time predict angle and continuously track it with the motor controller
#
# Requirements:
#   pip3 install numpy scikit-learn joblib
#   sudo apt install python3-lgpio   (or: pip3 install lgpio)
#
# Wiring (BCM):
#   L298N: IN1=24, IN2=23, ENA=25
#   Encoder: A=17, B=27

import socket, time, os, math, threading
from collections import deque

import numpy as np
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.preprocessing import StandardScaler
import joblib

# =========================
# ===== LDA CONFIG ========
# =========================
DATA_FILE_X  = "emg_history_features.npy"
DATA_FILE_Y  = "emg_history_labels.npy"
MODEL_FILE   = "lda_model_pro.joblib"
SCALER_FILE  = "scaler_pro.joblib"

UDP_IP   = "0.0.0.0"
UDP_PORT = 5005

FS     = 1000         # sampling frequency (Hz)
WIN_N  = 200          # 200 ms window @ 1 kHz
STEP_N = 50           # 50 ms update (overlap)

# Output classes (angles in degrees)
CLASSES = ["0", "45", "90", "-45", "-90"]
SECONDS_PER_CLASS = 4

# Prediction gating
CONF_THRESH = 0.75

# =========================
# == FEATURE EXTRACTION ===
# =========================
def extract_features(s1, s2):
    # DC offset removal
    s1 = s1 - np.mean(s1)
    s2 = s2 - np.mean(s2)

    def td_feats(x):
        return [
            float(np.mean(np.abs(x))),          # MAV
            float(np.sqrt(np.mean(x**2))),      # RMS
            float(np.sum(np.abs(np.diff(x)))),  # WL
            float(np.var(x))                    # VAR
        ]
    return td_feats(s1) + td_feats(s2)

# =========================
# ===== CONTROLLER ========
# =========================
# ML output updates a shared target; a fast control loop tracks it continuously.
# If the motor still feels slow, increase U_LIMIT (<= 1.0) and/or reduce STOP_BAND slightly.

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
PWM_FREQ  = 5000      # Hz (software PWM thread). 5k is a good compromise for Pi load.
DEADBAND  = 0.05      # |u| below this -> motor off
MIN_PWM   = 0.18      # minimum duty to overcome stiction (try 0.12..0.25)

# ---- Control loop ----
DT        = 0.005     # 200 Hz control loop
STOP_BAND = 0.6       # deg (inside this, reduce drive)
U_LIMIT   = 0.60      # max absolute duty command (0..1). Increase if motor is slow.
DU_MAX    = 2.5       # max duty change per second (slew-rate)

# ---- Reference shaping ----
MAX_REF_SPEED = 240.0  # deg/s ramp (90..360 typical)

# ---- STSMC tuning (start point) ----
LAMBDA = 3.0
K1 = 0.030
K2 = 0.012
PHI = 6.0

# If motor direction is reversed, toggle this:
CTRL_INVERT = True

# ---- Controller runtime globals ----
_ctrl = {"h": None, "pwm": None, "enc": None, "ctl": None, "ready": False, "abort": threading.Event()}
_target = {"deg": 0.0}
_target_lock = threading.Lock()
_ref = {"deg": 0.0, "t": None}

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
    if not isinstance(u, (int, float)) or u != u or math.isinf(u):
        u = 0.0
    return u, v_int

def _apply_deadzone_and_min_pwm(u):
    u = max(-U_LIMIT, min(U_LIMIT, float(u)))
    if abs(u) < DEADBAND:
        return 0.0
    if u > 0:
        return max(u, MIN_PWM)
    return min(u, -MIN_PWM)

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

def set_target_angle(deg):
    """ML -> Controller: set desired angle in degrees."""
    d = _clamp(float(deg), MIN_ANGLE, MAX_ANGLE)
    with _target_lock:
        _target["deg"] = d

def get_target_angle():
    with _target_lock:
        return float(_target["deg"])

class _ControlLoop(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self._stop = False

    def stop(self):
        self._stop = True

    def _update_ref_ramp(self, target):
        now = time.monotonic()
        if _ref["t"] is None:
            _ref["t"] = now
        dt = now - _ref["t"]
        _ref["t"] = now
        if dt <= 0:
            return _ref["deg"]
        step = MAX_REF_SPEED * dt
        if target > _ref["deg"] + step:
            _ref["deg"] += step
        elif target < _ref["deg"] - step:
            _ref["deg"] -= step
        else:
            _ref["deg"] = target
        _ref["deg"] = _clamp(_ref["deg"], MIN_ANGLE, MAX_ANGLE)
        return _ref["deg"]

    def run(self):
        enc = _ctrl["enc"]
        if enc is None:
            return

        e_prev = 0.0
        t_prev = time.monotonic()
        v_int = 0.0
        u_prev = 0.0

        while not self._stop and not _ctrl["abort"].is_set():
            now = time.monotonic()
            if (now - t_prev) < DT:
                time.sleep(0.0002)
                continue

            dt = now - t_prev
            if dt <= 0:
                dt = DT
            if dt > 0.05:
                dt = DT

            theta = _counts_to_deg(enc.get_count())

            target = get_target_angle()
            ref = self._update_ref_ramp(target)

            e = (ref - theta)
            de = (e - e_prev) / dt
            s = de + LAMBDA * e

            u_cmd, v_int = _stsmc_step(s, dt, v_int)
            if CTRL_INVERT:
                u_cmd = -u_cmd

            # soften drive near target to reduce jitter
            if abs(e) <= STOP_BAND and STOP_BAND > 1e-6:
                u_cmd *= (abs(e) / STOP_BAND)

            # slew-rate limit
            du = u_cmd - u_prev
            max_du = DU_MAX * dt
            if du >  max_du: u_cmd = u_prev + max_du
            if du < -max_du: u_cmd = u_prev - max_du
            u_prev = u_cmd

            u_cmd = _apply_deadzone_and_min_pwm(u_cmd)
            _set_motor(u_cmd)

            e_prev = e
            t_prev = now

        _set_motor(0.0)

def controller_init():
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

    _ref["deg"] = 0.0
    _ref["t"] = time.monotonic()
    _ctrl["abort"].clear()

    ctl = _ControlLoop()
    ctl.start()
    _ctrl["ctl"] = ctl
    _ctrl["ready"] = True

def controller_zero_here():
    if not _ctrl["ready"]:
        controller_init()
    _ctrl["enc"].reset(0)
    set_target_angle(0.0)
    _ref["deg"] = 0.0
    _ref["t"] = time.monotonic()

def controller_shutdown():
    try:
        _ctrl["abort"].set()
        if _ctrl["ctl"] is not None:
            _ctrl["ctl"].stop()
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
def realtime_predict_and_control(lda, scaler):
    controller_init()
    print("\nPlace joint at physical 0° then press ENTER to zero encoder (recommended).")
    input("")
    controller_zero_here()
    print("[OK] Encoder zeroed. Real-time control active. Ctrl+C to stop.\n")

    buf1, buf2 = deque(maxlen=WIN_N), deque(maxlen=WIN_N)
    samples_count = 0

    # ML -> motor bridging (fast ML, slower motor)
    CMD_PERIOD = 0.02          # seconds (try 0.15 .. 0.30)
    SEND_IF_CHANGE_DEG = 1    # only update target if change >= this
    STABLE_N = 1                 # require same class this many times before updating target

    last_set_t = 0.0
    last_target = None
    stable_pred = None
    stable_count = 0

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

                                # stability filter (avoid rapid toggling)
                                if stable_pred == angle_deg:
                                    stable_count += 1
                                else:
                                    stable_pred = angle_deg
                                    stable_count = 1

                                print(f"Angle: {angle_str:>4}° | Conf: {prob:.1%} | stable {stable_count}/{STABLE_N}")

                                now = time.time()
                                if stable_count >= STABLE_N and (now - last_set_t) >= CMD_PERIOD:
                                    allow_change = (last_target is None) or (abs(angle_deg - last_target) >= SEND_IF_CHANGE_DEG)
                                    if allow_change:
                                        set_target_angle(angle_deg)   # <<< ML output becomes motor reference
                                        last_target = angle_deg
                                        last_set_t = now
                            else:
                                stable_pred = None
                                stable_count = 0
                                print(f"Angle: Uncertain | Conf: {prob:.1%}")

                            samples_count = 0

            except socket.timeout:
                continue

    except KeyboardInterrupt:
        print("\n[EXIT] Stopping...")

    finally:
        controller_shutdown()

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

    realtime_predict_and_control(lda, scaler)

if __name__ == "__main__":
    try:
        main()
    finally:
        try:
            sock.close()
        except Exception:
            pass
