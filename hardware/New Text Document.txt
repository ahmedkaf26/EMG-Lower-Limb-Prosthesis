| Raspberry Pi 5 Physical Pin |   BCM GPIO | Direction | Connected Component | Component Pin | Function                 |
| --------------------------: | ---------: | --------- | ------------------- | ------------- | ------------------------ |
|                  **Pin 11** | **GPIO17** | Input     | Magnetic Encoder    | Channel A     | Encoder Channel A        |
|                  **Pin 13** | **GPIO27** | Input     | Magnetic Encoder    | Channel B     | Encoder Channel B        |
|                  **Pin 16** | **GPIO23** | Output    | L298N Motor Driver  | IN2           | Motor Direction Control  |
|                  **Pin 18** | **GPIO24** | Output    | L298N Motor Driver  | IN1           | Motor Direction Control  |
|                  **Pin 22** | **GPIO25** | Output    | L298N Motor Driver  | ENA           | PWM Motor Speed Control  |
|                  **Pin 14** |    **GND** | —         | L298N / Encoder     | GND           | Common Ground            |
|              **Pin 2 or 4** |     **5V** | Power     | Encoder*            | VCC           | 5V Supply, if required   |
|             **Pin 1 or 17** |   **3.3V** | Power     | Encoder*            | VCC           | 3.3V Supply, if required |

   




                RASPBERRY PI 5
               ┌──────────────────────┐
               │                      │
Encoder A ────►│ GPIO17   Pin 11      │
Encoder B ────►│ GPIO27   Pin 13      │
               │                      │
L298N IN2 ◄────│ GPIO23   Pin 16      │
L298N IN1 ◄────│ GPIO24   Pin 18      │
L298N ENA ◄────│ GPIO25   Pin 22      │
               │                      │
GND ───────────│ GND      Pin 14      │
               │                      │
               └──────────────────────┘