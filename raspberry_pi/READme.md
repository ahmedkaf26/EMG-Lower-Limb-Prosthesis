# EMG-Based Lower-Limb Prosthesis Control System

An intelligent lower-limb prosthesis control system that combines **surface electromyography (sEMG)**, **Linear Discriminant Analysis (LDA)**, and a **Super-Twisting Sliding Mode Controller (STSMC)** to recognize the user's intended joint angle and control a DC motor accordingly.

The system is designed for real-time implementation on a **Raspberry Pi 5** and uses an **Arduino Uno R4 WiFi** for EMG acquisition, an **L298N motor driver**, a **DC geared motor**, and a **quadrature magnetic encoder** for position feedback.

---

## 📌 Overview

The system consists of two main stages:

1. **EMG-based angle classification**
2. **Closed-loop motor position control**

The Arduino acquires two-channel EMG signals and transmits the data to the Raspberry Pi over Wi-Fi using UDP.

The Raspberry Pi:

* Receives the EMG data.
* Segments the signals into sliding windows.
* Extracts time-domain features.
* Standardizes the extracted features.
* Classifies the intended joint angle using LDA.
* Sends the predicted angle to the motor controller.
* Reads the motor position using a quadrature encoder.
* Controls the DC motor using an STSMC algorithm.

### System Pipeline

```text
┌─────────────────────┐
│   EMG Sensors       │
│  MyoWare 2.0        │
│  Channel 1 & 2      │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Arduino Uno R4 WiFi │
│   EMG Acquisition    │
└──────────┬──────────┘
           │
           │ UDP / Wi-Fi
           │ Port 5005
           ▼
┌──────────────────────────┐
│      Raspberry Pi 5      │
│                          │
│  EMG Processing          │
│       ↓                  │
│  Feature Extraction      │
│       ↓                  │
│  StandardScaler          │
│       ↓                  │
│  LDA Classifier          │
└──────────┬───────────────┘
           │
           │ Target Angle
           ▼
┌──────────────────────────┐
│ STSMC Position Controller│
└──────────┬───────────────┘
           │
           ▼
┌──────────────────────────┐
│       L298N Driver       │
└──────────┬───────────────┘
           │
           ▼
┌──────────────────────────┐
│     DC Gear Motor        │
│       GA25-370           │
└──────────┬───────────────┘
           │
           │ Mechanical Motion
           ▼
┌──────────────────────────┐
│   Magnetic Encoder       │
│  Quadrature A / B        │
└──────────┬───────────────┘
           │
           │ Position Feedback
           └──────────────► Raspberry Pi 5
```

---

# ✨ Main Features

* Two-channel EMG acquisition
* Real-time EMG data transmission over Wi-Fi
* UDP-based communication
* Sliding-window EMG processing
* Time-domain feature extraction
* Online dataset collection
* LDA-based EMG classification
* Feature standardization using `StandardScaler`
* Persistent ML model using Joblib
* Quadrature encoder feedback
* Closed-loop DC motor position control
* Super-Twisting Sliding Mode Control (STSMC)
* PWM motor control
* Motor direction control
* Position limits from -90° to +90°
* Confidence-based prediction gating
* Command rate limiting
* Encoder zeroing
* Motor safety timeout
* Soft-start and slew-rate limiting

---

# 🧠 EMG Classification

## EMG Input

The system receives two EMG channels.

The current implementation uses:

* Sampling frequency: **1000 Hz**
* Window length: **200 samples**
* Window duration: **200 ms**
* Step size: **50 samples**
* Approximate overlap: **75%**

## Target Classes

The classifier recognizes five target joint angles:

| Class | Target Angle |
| ----: | -----------: |
|     0 |           0° |
|     1 |          45° |
|     2 |          90° |
|     3 |         -45° |
|     4 |         -90° |

---

# 📊 Feature Extraction

For each EMG channel, four time-domain features are extracted:

### 1. Mean Absolute Value (MAV)

```text
MAV = mean(|x|)
```

### 2. Root Mean Square (RMS)

```text
RMS = sqrt(mean(x²))
```

### 3. Waveform Length (WL)

```text
WL = Σ|x[n] - x[n-1]|
```

### 4. Variance (VAR)

```text
VAR = variance(x)
```

The DC offset is removed before feature extraction.

Since two channels are used, the final feature vector contains:

```text
4 features × 2 channels = 8 features
```

Feature vector:

```text
[MAV1, RMS1, WL1, VAR1,
 MAV2, RMS2, WL2, VAR2]
```

---

# 🤖 Machine Learning

The classification pipeline is:

```text
Raw EMG
   ↓
DC Offset Removal
   ↓
200 ms Sliding Window
   ↓
Feature Extraction
   ↓
StandardScaler
   ↓
LDA Classifier
   ↓
Predicted Angle
```

The system uses:

```python
LinearDiscriminantAnalysis()
```

from Scikit-learn.

Before classification, the features are standardized using:

```python
StandardScaler()
```

The trained classifier and scaler are stored using Joblib.

---

# 🎛️ Motor Control

The predicted EMG angle is used as the reference position for the motor controller.

The system uses a **Super-Twisting Sliding Mode Controller (STSMC)**.

The sliding variable is defined as:

```text
s = de + λe
```

where:

* `e` = position error
* `de` = error derivative
* `λ` = sliding surface parameter

The STSMC control law includes a continuous approximation using the saturation function to reduce chattering.

---

# ⚙️ Controller Parameters

The current implementation uses:

| Parameter          |   Value |
| ------------------ | ------: |
| `LAMBDA`           |     6.0 |
| `K1`               |   0.020 |
| `K2`               |   0.005 |
| `PHI`              |     5.0 |
| `DT`               | 0.002 s |
| Control frequency  |  500 Hz |
| PWM frequency      | 1800 Hz |
| `STOP_BAND`        |    0.5° |
| `HOLD_TIME`        |  0.50 s |
| `MAX_SECONDS`      |     8 s |
| `MOVE_FOR_SECONDS` |   1.0 s |
| `U_MAX`            |     1.0 |
| `U_MIN`            |    -1.0 |
| `DEADBAND`         |    0.06 |
| `DU_MAX`           |     2.5 |
| `SOFTSTART_SEC`    |    0.35 |
| `K_GRAV`           |    0.40 |

These values can be tuned according to the motor, mechanical load, prosthesis dynamics, and experimental requirements.

---

# 🔌 Hardware

The system is designed around the following components:

* Raspberry Pi 5
* Arduino Uno R4 WiFi
* MyoWare EMG sensors
* L298N motor driver
* GA25-370 DC geared motor
* Quadrature magnetic encoder
* External motor power supply

---

# 🔗 Raspberry Pi GPIO Connections

The GPIO numbering used by the Python program is **BCM numbering**.

| Raspberry Pi Physical Pin | BCM GPIO | Function   | Connection        |
| ------------------------: | -------: | ---------- | ----------------- |
|                    Pin 18 |   GPIO24 | Output     | L298N IN1         |
|                    Pin 16 |   GPIO23 | Output     | L298N IN2         |
|                    Pin 22 |   GPIO25 | PWM Output | L298N ENA         |
|                    Pin 11 |   GPIO17 | Input      | Encoder Channel A |
|                    Pin 13 |   GPIO27 | Input      | Encoder Channel B |
|                    Pin 14 |      GND | Ground     | Common Ground     |

### Motor Driver

```text
Raspberry Pi GPIO24 ───────► L298N IN1
Raspberry Pi GPIO23 ───────► L298N IN2
Raspberry Pi GPIO25 ───────► L298N ENA

L298N OUT1 ────────────────► Motor terminal 1
L298N OUT2 ────────────────► Motor terminal 2

External +12 V ────────────► L298N VS / Motor Supply
External GND ──────────────► L298N GND
```

The Raspberry Pi ground and motor-driver ground must share a common reference.

### Encoder

```text
Encoder A ────────────────► Raspberry Pi GPIO17
Encoder B ────────────────► Raspberry Pi GPIO27
Encoder GND ──────────────► Raspberry Pi GND
Encoder VCC ──────────────► Appropriate encoder supply
```

> ⚠️ **Important:** The encoder supply voltage must be verified from the encoder's datasheet. Raspberry Pi GPIO inputs use 3.3 V logic. If the encoder outputs 5 V logic, appropriate level shifting must be used before connecting its A/B outputs to the Raspberry Pi.

---

# 📡 Arduino to Raspberry Pi Communication

The Arduino Uno R4 WiFi sends EMG data to the Raspberry Pi using:

```text
Protocol: UDP
Port: 5005
```

The Raspberry Pi listens on:

```python
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
```

Therefore, the Arduino and Raspberry Pi communicate through **Wi-Fi**, not through the Raspberry Pi GPIO pins.

The expected UDP data format is:

```text
id;v1,v2;v1,v2;v1,v2;...
```

Example:

```text
15;512,487;514,489;518,491;...
```

where each pair represents the two EMG channels.

---

# 📁 Project Structure

The current project is intentionally kept as a simple single-file implementation:

```text
EMG-Lower-Limb-Prosthesis/
│
├── README.md
├── main.py
├── requirements.txt
├── .gitignore
│
└── images/
    ├── system_architecture.png
    ├── hardware_connection.png
    └── prototype.jpg
```

The generated model and dataset files are created locally during operation and should generally not be committed to GitHub unless intentionally included.

Generated files include:

```text
emg_history_features.npy
emg_history_labels.npy
lda_model_pro.joblib
scaler_pro.joblib
```

---

# 💻 Software Requirements

## Operating System

Recommended:

```text
Raspberry Pi OS
```

on:

```text
Raspberry Pi 5
```

## Python

Python 3 is required.

Recommended environment:

```text
Python 3.11+
```

The project uses the following Python packages:

```text
numpy
scikit-learn
joblib
lgpio
```

---

# 📦 Installation

Clone the repository:

```bash
git clone https://github.com/YOUR_USERNAME/EMG-Lower-Limb-Prosthesis.git
```

Enter the project directory:

```bash
cd EMG-Lower-Limb-Prosthesis
```

Create a virtual environment:

```bash
python3 -m venv venv
```

Activate it:

```bash
source venv/bin/activate
```

Install the required packages:

```bash
pip install -r requirements.txt
```

---

# ▶️ Running the System

Run:

```bash
python3 main.py
```

The program displays:

```text
--- EMG Angle Control System (LDA) + Motor Controller ---
1) Train/Improve Model
2) Real-time Predict + Move Motor
```

---

# 🧪 Option 1 — Train / Improve Model

Select:

```text
1
```

The system starts the EMG calibration procedure.

The five classes are recorded sequentially:

```text
0°
45°
90°
-45°
-90°
```

For every class:

1. The user prepares the required joint position.
2. A countdown is displayed.
3. EMG data is recorded.
4. Sliding windows are generated.
5. Features are extracted.
6. The new samples are added to the historical dataset.
7. The LDA model is trained.
8. The scaler and classifier are saved.

The historical dataset is stored in:

```text
emg_history_features.npy
emg_history_labels.npy
```

The trained model is stored in:

```text
lda_model_pro.joblib
```

The feature scaler is stored in:

```text
scaler_pro.joblib
```

---

# 🎯 Option 2 — Real-Time Prediction and Motor Control

Select:

```text
2
```

The program loads:

```text
lda_model_pro.joblib
scaler_pro.joblib
```

The encoder is then initialized.

The user should place the joint at its physical zero position and press Enter.

The program executes:

```text
controller_zero_here()
```

which sets the current encoder position to:

```text
0°
```

The system then starts real-time EMG classification and motor control.

---

# 🔄 Real-Time Control Pipeline

```text
EMG acquisition
       ↓
UDP packet
       ↓
Sliding window
       ↓
Feature extraction
       ↓
Feature scaling
       ↓
LDA prediction
       ↓
Prediction probability
       ↓
Confidence threshold
       ↓
Target angle
       ↓
STSMC
       ↓
PWM + Direction
       ↓
DC Motor
       ↓
Encoder feedback
       ↓
Current angle
       ↓
Position error
       ↺
```

---

# 🛡️ Safety Features

Several software mechanisms are included to reduce unsafe motor behavior.

### Angle Limitation

The target angle is restricted to:

```text
-90° ≤ target ≤ +90°
```

### Confidence Threshold

A motor command is only generated when:

```text
confidence >= 0.40
```

### Command Interval

Commands are limited by:

```text
MIN_CMD_INTERVAL = 0.10 s
```

### Direction Deadband

Very small control commands are ignored:

```text
DEADBAND = 0.06
```

### Soft Start

The control signal gradually increases during startup.

### Slew Rate Limiting

Rapid changes in the control signal are limited using:

```text
DU_MAX = 2.5
```

### Motor Stop

The motor is stopped when the movement command ends.

The system also supports a maximum controller timeout:

```text
MAX_SECONDS = 8.0
```

---

# 🎚️ Encoder

The motor position is measured using a quadrature magnetic encoder.

The current configuration assumes:

```text
CPR_OUT = 1968
```

counts per output shaft revolution.

The encoder count is converted to degrees using:

```text
Angle = Count × 360 / CPR_OUT
```

The encoder can be inverted using:

```python
ENC_INVERT = True
```

if the measured angle changes in the opposite direction to the expected mechanical direction.

---

# 🔧 Configuration

Most system parameters can be modified directly near the beginning of `main.py`.

Important parameters include:

### EMG

```python
FS = 1000
WIN_N = 200
STEP_N = 50
```

### Classification

```python
CLASSES = ["0", "45", "90", "-45", "-90"]
CONF_THRESH = 0.40
```

### UDP

```python
UDP_PORT = 5005
```

### Motor

```python
PWM_FREQ = 1800
DEADBAND = 0.06
```

### Encoder

```python
CPR_OUT = 1968
ENC_INVERT = False
```

### Controller

```python
LAMBDA = 6.0
K1 = 0.020
K2 = 0.005
PHI = 5.0
```

---

# 🧰 Troubleshooting

## No EMG data

If the program displays:

```text
[ERROR] No data collected. Check UDP stream.
```

check:

* Arduino Wi-Fi connection.
* Raspberry Pi Wi-Fi connection.
* Raspberry Pi IP address.
* UDP port configuration.
* Arduino destination IP.
* UDP port `5005`.
* EMG sensor connections.

---

## No model found

If the program displays:

```text
[ERROR] No model found. Run Option 1 first.
```

run:

```text
Option 1
```

to create:

```text
lda_model_pro.joblib
scaler_pro.joblib
```

Then run:

```text
Option 2
```

---

## Motor moves in the wrong direction

First check:

```python
CTRL_INVERT = True
```

You can change it to:

```python
CTRL_INVERT = False
```

Alternatively, verify the motor-driver wiring and encoder direction.

---

## Encoder angle moves in the wrong direction

Change:

```python
ENC_INVERT = False
```

to:

```python
ENC_INVERT = True
```

---

## Encoder angle is incorrect

Verify:

```python
CPR_OUT = 1968
```

The actual encoder count per output shaft revolution should be experimentally calibrated if necessary.

---

## GPIO permission / access problems

Make sure the program is running on the Raspberry Pi and that the `lgpio` package is installed correctly.

Test:

```bash
python3 -c "import lgpio; print('lgpio OK')"
```

---

# ⚠️ Hardware Safety

This project controls a physical DC motor and is intended for research and prototyping.

Before connecting the motor:

* Test GPIO signals without the motor connected.
* Verify the motor-driver wiring.
* Verify common ground.
* Verify encoder signals.
* Confirm the motor supply voltage.
* Start with a low mechanical load.
* Keep the emergency power disconnect accessible.
* Do not place a person inside the mechanical motion range during initial testing.
* Verify the mechanical angle limits independently of software limits.

The software angle limits should **not** be considered a substitute for physical safety mechanisms.

---

# 📈 Experimental Workflow

A recommended experimental workflow is:

```text
1. Hardware inspection
       ↓
2. Verify encoder
       ↓
3. Verify motor direction
       ↓
4. Verify PWM
       ↓
5. Test motor without EMG
       ↓
6. Acquire EMG data
       ↓
7. Train LDA model
       ↓
8. Evaluate classification
       ↓
9. Test real-time predictions
       ↓
10. Enable motor control
       ↓
11. Tune STSMC parameters
       ↓
12. Evaluate tracking performance
```

---

# 📊 Evaluation

For research and thesis purposes, the system can be evaluated using:

### Classification

* Accuracy
* Precision
* Recall
* F1-score
* Confusion matrix
* Classification confidence
* Prediction latency

### Motor Control

* Tracking error
* RMSE
* MAE
* Maximum error
* Settling time
* Rise time
* Overshoot
* Control effort
* Torque/control-signal variation
* Chattering indicators

---

# 🔬 Research Context

This project investigates the integration of:

```text
Surface EMG
+
Machine Learning
+
Embedded Systems
+
DC Motor Control
+
Sliding Mode Control
+
Prosthetic Joint Control
```

The overall objective is to develop an intelligent control architecture capable of interpreting muscle activity and translating the user's intended joint angle into real-time prosthetic movement.

---

# 📚 Technologies

* Python
* NumPy
* Scikit-learn
* Joblib
* lgpio
* Raspberry Pi 5
* Arduino Uno R4 WiFi
* MyoWare EMG
* L298N
* DC geared motor
* Magnetic quadrature encoder
* UDP/Wi-Fi communication
* Linear Discriminant Analysis
* StandardScaler
* Super-Twisting Sliding Mode Control

---

# 📂 Generated Files

During operation, the following files may be generated:

| File                       | Description                    |
| -------------------------- | ------------------------------ |
| `emg_history_features.npy` | Historical EMG feature dataset |
| `emg_history_labels.npy`   | Corresponding class labels     |
| `lda_model_pro.joblib`     | Trained LDA classifier         |
| `scaler_pro.joblib`        | Feature standardization model  |

These files contain experiment-specific data and may be excluded from version control.

---

# 🚫 Git Ignore

A recommended `.gitignore` is:

```gitignore
# Python
__pycache__/
*.py[cod]
*.so

# Virtual environment
venv/
.venv/
env/

# IDE
.vscode/
.idea/

# Python cache
.pytest_cache/

# EMG datasets
emg_history_features.npy
emg_history_labels.npy

# Trained models
lda_model_pro.joblib
scaler_pro.joblib

# Logs
*.log

# OS files
.DS_Store
Thumbs.db
```

---

# 📜 License

This project is intended primarily for academic and research purposes.

If you plan to make the repository publicly reusable, add an appropriate open-source license such as MIT, Apache-2.0, or another license that matches your intended use.

---

# 👤 Author

**ahmed**


Research interests:

* Prosthetic Robotics
* EMG Signal Processing
* Machine Learning
* Intelligent Control
* Sliding Mode Control
* Lower-Limb Prostheses
* Embedded Control Systems
* Human–Machine Interfaces

---

# ⭐ Acknowledgment

This project was developed as part of research on intelligent EMG-based control of lower-limb prosthetic systems.

---

# ⚠️ Disclaimer

This system is an experimental research prototype and is **not a certified medical device**.

It must not be used for clinical or therapeutic applications without appropriate validation, risk assessment, mechanical safety testing, and regulatory approval.

---

## 📌 Project Status

**Development / Research Prototype**

The current implementation supports:

* EMG acquisition
* Online dataset collection
* LDA training
* Real-time angle classification
* Encoder-based position feedback
* STSMC motor control
* Raspberry Pi 5 implementation
* Arduino Uno R4 WiFi communication
