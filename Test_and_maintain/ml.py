import socket, time, json, os
import numpy as np
import pandas as pd
from collections import deque
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.preprocessing import StandardScaler
import joblib

# ========= File & System Configuration =========
DATA_FILE_X = "emg_history_features.npy"
DATA_FILE_Y = "emg_history_labels.npy"
MODEL_FILE = "lda_model_pro.joblib"
SCALER_FILE = "scaler_pro.joblib"

UDP_IP = "0.0.0.0"
UDP_PORT = 5005
FS = 1000          # Sampling Frequency (Hz)
WIN_N = 200        # 200ms Window
STEP_N = 50        # 50ms Update (Overlap)

# Joint angles for the knee/ankle prosthesis
CLASSES = ["0", "45", "90", "-45", "-90"]
SECONDS_PER_CLASS = 4 

# ========= Feature Extraction =========
def extract_features(s1, s2):
    # DC Offset Removal (Mean Subtraction)
    s1, s2 = s1 - np.mean(s1), s2 - np.mean(s2)
    
    def get_td_feats(x):
        return [
            np.mean(np.abs(x)),          # MAV
            np.sqrt(np.mean(x**2)),      # RMS
            np.sum(np.abs(np.diff(x))),  # WL
            np.var(x)                    # VAR
        ]
    return get_td_feats(s1) + get_td_feats(s2)

# ========= Socket Initialization =========
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(1.0)

def train_online():
    # 1. Load historical data for cumulative improvement
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
                
                # Parse Batch UDP: "id;v1,v2;v1,v2..."
                for line in text.splitlines():
                    parts = line.strip().split(';')
                    if len(parts) < 2: continue
                    
                    for p in parts[1:]:
                        vv = p.split(',')
                        if len(vv) != 2: continue
                        buf1.append(float(vv[0]))
                        buf2.append(float(vv[1]))
                        
                        if len(buf1) == WIN_N:
                            X_session.append(extract_features(np.array(buf1), np.array(buf2)))
                            y_session.append(idx)
                            captured_count += 1
                            # Overlap shift
                            for _ in range(STEP_N):
                                if buf1: buf1.popleft()
                                if buf2: buf2.popleft()
            except socket.timeout: continue

        print(f"[DONE] Captured {captured_count} samples for {angle}°")

    if len(X_session) == 0 and len(X_history) == 0:
        print("[ERROR] No data collected. Check UDP stream.")
        return None, None

    # 3. Save Cumulative Data
    X_combined = np.array(X_history + X_session)
    y_combined = np.array(y_history + y_session)
    np.save(DATA_FILE_X, X_combined)
    np.save(DATA_FILE_Y, y_combined)
    
    # 4. Train LDA
    print("\n[PROCESS] Optimizing LDA Model...")
    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X_combined)
    lda = LinearDiscriminantAnalysis()
    lda.fit(X_scaled, y_combined)
    
    joblib.dump(lda, MODEL_FILE)
    joblib.dump(scaler, SCALER_FILE)
    print(f"[SUCCESS] Total Dataset Size: {len(X_combined)} samples.")
    return lda, scaler

# ========= Main Flow =========
print("--- EMG Angle Control System (LDA) ---")
print("1. Train/Improve Model")
print("2. Direct Real-time Control")
choice = input("Select Option: ")

if choice == '1':
    lda, scaler = train_online()
    if lda is None: exit()
else:
    try:
        lda = joblib.load(MODEL_FILE)
        scaler = joblib.load(SCALER_FILE)
        print("[OK] Loaded trained model.")
    except:
        print("[ERROR] No model found. Run Option 1 first.")
        exit()

# ========= Real-time Prediction Loop =========
print("\n=== Real-time Prediction Active ===")
buf1, buf2 = deque(maxlen=WIN_N), deque(maxlen=WIN_N)
samples_count = 0

try:
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            text = data.decode("utf-8", errors="ignore")
            
            for line in text.splitlines():
                parts = line.strip().split(';')
                if len(parts) < 2: continue
                
                for p in parts[1:]:
                    vv = p.split(',')
                    if len(vv) != 2: continue
                    buf1.append(float(vv[0]))
                    buf2.append(float(vv[1]))
                    samples_count += 1

                    if len(buf1) == WIN_N and samples_count >= STEP_N:
                        feat = extract_features(np.array(buf1), np.array(buf2))
                        feat_scaled = scaler.transform([feat])
                        
                        pred = lda.predict(feat_scaled)[0]
                        prob = np.max(lda.predict_proba(feat_scaled))
                        
                        # Display result
                        if prob > 0.75:
                            print(f"Angle: {CLASSES[pred]:>4}° | Conf: {prob:.1%}")
                        else:
                            print(f"Angle: Uncertain | Conf: {prob:.1%}")
                        
                        samples_count = 0
        except socket.timeout: continue
except KeyboardInterrupt:
    print("\n[EXIT] Session ended.")
finally:
    sock.close()