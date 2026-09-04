# Dual-Channel EMG UDP Sender

Arduino Uno R4 WiFi program for:

* Reading 2 EMG channels (`A0`, `A1`)
* Sampling at approximately **1 kHz**
* Sending data via **UDP**
* Sending 20 samples per packet
* Receiving data on a Raspberry Pi or PC

## Data Format

```text
packet_id;CH1,CH2;CH1,CH2;...
```

## Configuration

Set your own WiFi and receiver settings in the code:

```cpp
SSID
PASSWORD
RECEIVER_IP
UDP_PORT
```

## Requirements

* Arduino Uno R4 WiFi
* 2 EMG sensors
* Raspberry Pi or PC
* `WiFiS3` and `WiFiUdp` libraries

**Note:** Do not upload real WiFi passwords or private network information to GitHub.
