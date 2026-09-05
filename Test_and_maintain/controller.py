#!/usr/bin/env python3
# Pi5 + L298N + quadrature encoder — STSMC position controller (SMOOTH/LIMITED)
# Manual: reference angle entered from terminal by user (NO UDP).
# Range mode: any reference angle between -90 and +90 (no 360 wrap).

import time, math, threading, queue
import lgpio as lg

# ========= USER CONFIG =========
# Pins (BCM numbering)
IN1, IN2, ENA = 24, 23, 25           # L298N
ENC_A, ENC_B = 17, 27                # Encoder

# Encoder parameters (output shaft)
CPR_OUT    = 1968     # counts per revolution after gearbox (calibrate via menu k/c)
ENC_INVERT = False    # True if angle grows in wrong direction

# Allowed reference range (pendulum)
MIN_ANGLE = -90.0
MAX_ANGLE =  90.0

# PWM
PWM_FREQ  = 1800      # 1 kHz
DEADBAND  = 0.06#0.08

# Control loop
DT          = 0.002   # 500 Hz
STOP_BAND   = 0.5     # deg
HOLD_TIME   = 0.50    # s inside band
MAX_SECONDS = 8.0     # hard failsafe

# ======== TIMED MOVE OPTION ========
# None → normal (hold-at-target). Value (e.g., 1.0) → move for N 6.0seconds then stop.
MOVE_FOR_SECONDS = 3.0

# ========= STSMC TUNING (calmer) =========
# Sliding surface: s = de + LAMBDA * e
LAMBDA = 6#8.5
K1 = 0.020#0.028
K2 = 0.005#0.010
PHI = 5.0


# Super-Twisting gains (start values; tune on your setup)
# K1 = 0.028
# K2 = 0.010

# Boundary layer to reduce chattering
# PHI = 6.0

# Control saturation
U_MAX, U_MIN = 1.0, -1.0
U_GLOBAL_GAIN = 1.0
6.0
# Optional assists
#K_GRAV = 0.5172
K_GRAV = 0.40#0.156.0
KICK_MIN = 0.25
KICK_E_DEG = 2#3.0
KICK_DE_DPS = 15.0

# If motor direction is opposite, flip control output
CTRL_INVERT = True

# ===== Speed limiting / smoothing =====
U_LIMIT = 1 #0.80        # max PWM in normal running (0.30..0.60)
DU_MAX = 2.5#1.2          # max change of u per second (0.8..2.0)
SOFTSTART_SEC = 0.35  # ramp limit during first seconds

# ========= LOW LEVEL =========
h = lg.gpiochip_open(0)
lg.gpio_claim_output(h, IN1, 0)
lg.gpio_claim_output(h, IN2, 0)
lg.gpio_claim_output(h, ENA, 0)
lg.gpio_claim_input(h, ENC_A)
lg.gpio_claim_input(h, ENC_B)

# ========= PWM THREAD =========
class PWMThread(threading.Thread):
    def __init__(self, handle, pin, freq):
        super().__init__(daemon=True)
        self.h = handle
        self.pin = pin
        self.T = 1.0/freq
        self.duty = 0.0
        self._lock = threading.Lock()
        self._stop = False
        self._next = time.monotonic()

    def set_duty(self, duty):
        if not isinstance(duty, (int,float)) or duty != duty or math.isinf(duty):
            duty = 0.0
        duty = 0.0 if duty < 0 else 1.0 if duty > 1.0 else float(duty)
        with self._lock:
            self.duty = duty

    def stop(self): self._stop = True

    def run(self):
        on = False
        while not self._stop:
            now = time.monotonic()
            if now < self._next:
                time.sleep(0.0002); continue
            with self._lock: d = self.duty
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
                    self._next = now + d*T
                else:
                    lg.gpio_write(self.h, self.pin, 0)
                    on = False
                    self._next = now + (1.0-d)*T

pwm = PWMThread(h, ENA, PWM_FREQ)
pwm.start()

def set_motor(u):
    """u in [-1,1] → direction pins + duty to PWM thread."""
    if not isinstance(u, (int,float)) or u != u or math.isinf(u):
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

# ========= ENCODER THREAD =========
delta_table = [
    [ 0, -1, +1,  0],
    [+1,  0,  0, -1],
    [-1,  0,  0, +1],
    [ 0, +1, -1,  0],
]

class EncoderThread(threading.Thread):
    def __init__(self, handle, a_pin, b_pin, invert=False):
        super().__init__(daemon=True)
        self.h, self.a, self.b = handle, a_pin, b_pin
        a0 = lg.gpio_read(handle, a_pin)
        b0 = lg.gpio_read(handle, b_pin)
        self.state = (a0<<1) | b0
        self.count = 0
        self.sign = -1 if invert else 1
        self._lock = threading.Lock()
        self._stop = False

    def reset(self, value=0):
        with self._lock:
            self.count = value

    def get_count(self):
        with self._lock:
            return self.count

    def stop(self): self._stop = True

    def run(self):
        while not self._stop:
            a = lg.gpio_read(self.h, self.a)
            b = lg.gpio_read(self.h, self.b)
            s = (a<<1) | b
            d = delta_table[self.state][s]
            if d:
                with self._lock:
                    self.count += self.sign * d
            self.state = s
            time.sleep(0)

enc = EncoderThread(h, ENC_A, ENC_B, invert=ENC_INVERT)
enc.start()

# ========= ANGLE UTILS =========
def counts_to_deg(c):
    return (c * 360.0) / CPR_OUT

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

# ========= STSMC CORE =========
def sat(x, phi):
    if phi <= 1e-9:
        return 1.0 if x > 0 else (-1.0 if x < 0 else 0.0)
    if x >  phi: return  1.0
    if x < -phi: return -1.0
    return x / phi

def stsmc_step(s, dt, v_int):
    sigma = sat(s, PHI)
    v_int += (K2 * sigma) * dt
    u = -(K1 * math.sqrt(abs(s) + 1e-12) * sigma + v_int)
    u *= U_GLOBAL_GAIN
    if not isinstance(u, (int,float)) or u != u or math.isinf(u):
        u = 0.0
    u = max(U_MIN, min(U_MAX, u))
    return u, v_int

# ========= CONTROL CORE =========
ABORT_EVENT = threading.Event()

def move_to_angle_abs(target_deg_any):
    """Move to any reference angle in [-90, +90] (no 360 wrap)."""
    target = clamp(float(target_deg_any), MIN_ANGLE, MAX_ANGLE)
    T_ref = target

    e_prev = 0.0
    t_prev = time.monotonic()
    start  = t_prev
    in_band_since = None

    v_int = 0.0
    u_prev = 0.0

    # encoder sanity check
    last_c = enc.get_count()
    last_check = start

    print(f">> Move to {target:.2f}° (range [{MIN_ANGLE},{MAX_ANGLE}])")
    ABORT_EVENT.clear()

    while True:
        if ABORT_EVENT.is_set():
            print("!! Move canceled.")
            break

        now = time.monotonic()
        if (now - t_prev) < DT:
            time.sleep(0.0002); continue
        dt = now - t_prev
        if dt <= 0: dt = DT
        if dt > 0.05: dt = DT

        theta = counts_to_deg(enc.get_count())
        e = (T_ref - theta)
        de = (e - e_prev)/dt

        s = de + LAMBDA * e

        u_cmd, v_int = stsmc_step(s, dt, v_int)

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
        set_motor(u_cmd)

        # encoder sanity check
        if abs(u_cmd) > 0.5 and (now - last_check) >= 0.2:
            c_now = enc.get_count()
            if c_now == last_c:
                print("!! WARNING: PWM high but encoder count NOT changing -> check encoder wiring/power/level shifting.")
            last_c = c_now
            last_check = now

        # debug @ ~10 Hz
        if int((now-start)/0.1) != int((t_prev-start)/0.1):
            print(f"θ={theta:7.2f}°, e={e:7.2f}, de={de:7.2f}, s={s:7.2f}, u={u_cmd:5.2f}")

        # stop logic
        if MOVE_FOR_SECONDS is not None:
            if (now - start) >= MOVE_FOR_SECONDS:
                print(f"== Elapsed {MOVE_FOR_SECONDS:.2f}s from command, stopping ==")
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
                print("!! Timeout, stopping.")
                break

        e_prev = e
        t_prev = now

    set_motor(0.0)
    final_theta = counts_to_deg(enc.get_count())
    print(f"✓ Done. θ≈{final_theta:.2f}°\n")

# ========= HOMING, CPR CAL, DIAGNOSTICS =========
def home_zero():
    input("Place pendulum at physical 0°, then press ENTER to zero… ")
    enc.reset(0)
    print("Zeroed. θ = 0.00°")

def calibrate_cpr(theta_true_deg):
    global CPR_OUT
    theta_meas = counts_to_deg(enc.get_count())
    if abs(theta_true_deg) < 1e-6:
        print("True angle must be non-zero!")
        return
    scale = theta_meas / float(theta_true_deg)
    CPR_OUT = int(round(CPR_OUT * scale))
    print(f"Measured {theta_meas:.2f}°, true {theta_true_deg:.2f}° → scale {scale:.5f}")
    print(f"New CPR_OUT = {CPR_OUT}")

def quick_cal_0_to_90():
    global CPR_OUT
    print("-- Quick CPR calibration --")
    input("Place exactly at physical 0° (bottom). Press ENTER to zero… ")
    enc.reset(0)
    print("Zeroed. Now rotate to physical 90° (right angle) and press ENTER…")
    input("")
    counts = enc.get_count()
    if counts == 0:
        print("No counts detected; move away from 0° then retry.")
        return
    new_cpr = int(round(abs(counts) * 4))
    print(f"Counts @90° = {counts}, proposed CPR_OUT = {new_cpr}")
    CPR_OUT = new_cpr
    print(f"CPR_OUT set to {CPR_OUT}. Re-zero before testing.")

# Command queue
cmd_queue = queue.Queue(maxsize=8)

class CommandWorker(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self._stop = False

    def stop(self):
        self._stop = True
        ABORT_EVENT.set()

    def run(self):
        while not self._stop:
            try:
                kind, payload = cmd_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if kind == "angle":
                try:
                    move_to_angle_abs(payload)
                except Exception as e:
                    print(f"[worker] move error: {e}")

worker = CommandWorker()
worker.start()

def diag_print():
    c = enc.get_count()
    th = counts_to_deg(c)
    mode_timed = f"Timed=ON ({MOVE_FOR_SECONDS:.2f}s)" if MOVE_FOR_SECONDS is not None else "Timed=OFF"
    print(f"[diag] counts={c}, θ_meas={th:.2f}°, CPR_OUT={CPR_OUT}, {mode_timed}")
    print(f"      Range: [{MIN_ANGLE},{MAX_ANGLE}] | STSMC λ={LAMBDA}, K1={K1}, K2={K2}, φ={PHI}, CTRL_INVERT={CTRL_INVERT}")
    print(f"      Limits: U_LIMIT={U_LIMIT}, DU_MAX={DU_MAX}, SOFTSTART_SEC={SOFTSTART_SEC}")

def menu_text():
    mode_timed = f"Timed=ON ({MOVE_FOR_SECONDS} s)" if MOVE_FOR_SECONDS is not None else "Timed=OFF"
    return f"""
Manual reference from terminal (NO UDP). You can enter ANY angle in [{MIN_ANGLE}, {MAX_ANGLE}].
  Example: -30, -90, 0, 12.5, 45, 89
  r = re-zero
  k = calibrate CPR (single-point: enter TRUE angle of current pose)
  c = quick 0→90 CPR calibration (guided)
  t = set timed-move seconds (enable timed mode)
  x = disable timed-move (return to hold-at-target mode)
  p = cancel current move (if any)
  d = diagnostics
  q = quit

Current: {mode_timed}
"""

def main():
    global MOVE_FOR_SECONDS
    try:
        home_zero()
        while True:
            print(menu_text())
            s = input("> ").strip().lower()
            if s == "q": break
            if s == "r":
                home_zero(); continue
            if s == "k":
                try:
                    atrue = float(input("Enter TRUE angle (deg): "))
                    calibrate_cpr(atrue)
                except Exception:
                    print("Bad number.")
                continue
            if s == "c":
                quick_cal_0_to_90(); continue
            if s == "d":
                diag_print(); continue
            if s == "x":
                MOVE_FOR_SECONDS = None
                print("Timed move disabled. Controller will hold at target.")
                continue
            if s == "t":
                try:
                    secs = float(input("Enter seconds to move (e.g., 1 = 1.0s): ").strip())
                    if secs <= 0:
                        print("Value must be > 0 seconds.")
                    else:
                        MOVE_FOR_SECONDS = secs
                        print(f"Timed move enabled: {MOVE_FOR_SECONDS:.2f} s")
                except Exception:
                    print("Bad number.")
                continue
            if s == "p":
                ABORT_EVENT.set()
                print("Requested cancel of current move.")
                continue

            try:
                val = float(s)
            except ValueError:
                print(f"Enter an angle in [{MIN_ANGLE},{MAX_ANGLE}] or r/k/c/t/x/p/d/q.")
                continue
            try:
                while not cmd_queue.empty():
                    cmd_queue.get_nowait()
                cmd_queue.put_nowait(("angle", float(val)))
            except queue.Full:
                print("Command queue is full; try again.")
    finally:
        try:
            worker.stop()
        except: pass
        set_motor(0.0)
        pwm.stop(); enc.stop()
        time.sleep(0.05)
        lg.gpio_write(h, IN1, 0)
        lg.gpio_write(h, IN2, 0)
        lg.gpio_write(h, ENA, 0)
        lg.gpiochip_close(h)

if __name__ == "__main__":
    main()
