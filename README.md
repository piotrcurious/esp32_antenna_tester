# Quad ESP32-WROOM RF Antenna Testing System

An automated 2.4 GHz 4-radio Wi-Fi antenna characterization system using four ESP32-WROOM microcontrollers. The system performs synchronized frequency sweeps across 802.11 Wi-Fi channels (1–13), measuring Received Signal Strength Indicator (**RSSI**), Packet Error Rates (**PER**), and complex Channel State Information (**CSI**) subcarrier magnitudes ($H(f)$) across dual polarizations (Vertical and Horizontal).

The system outputs structured measurement streams over USB Serial from the Reference Node Master to a Linux computer for data logging, analysis, and visualization.

---

## 1. System Architecture & Topology

The system consists of two physical nodes positioned in a fixed RF geometry (e.g., 2 meters apart):

```
       REFERENCE NODE                                        TEST NODE
 (Fixed Known References)                              (Device Under Test)

┌────────────────────────┐                            ┌────────────────────────┐
│  ESP32 MASTER (Ref-V)  │                            │  ESP32 MASTER (Test-A) │
│  1/4-Wave Vertical Ant │                            │    TEST ANTENNA        │
└───────────┬────────────┘                            └───────────┬────────────┘
            │                                                     │
     Serial1 (UART)                                        Serial1 (UART)
     460800 baud                                           460800 baud
            │                                                     │
┌───────────┴────────────┐                            ┌───────────┴────────────┐
│  ESP32 SLAVE (Ref-H)   │                            │  ESP32 SLAVE (Test-R)  │
│ 1/4-Wave Horizontal Ant│                            │ 1/4-Wave Reference Ant │
└────────────────────────┘                            └────────────────────────┘

            ▲                                                     ▲
            │                                                     │
            └────────────── ESP-NOW Control Link ─────────────────┘
                           (Primary Control Channel 1)

            ~~~~~~~~~~~~~~~~ Raw 802.11 RF Test ~~~~~~~~~~~~~~~~
                         (Test Channels 1–13 / CSI)
```

### Node Functions
1. **Reference Node Master (`ref_node_master/ref_node_master.ino`)**:
   - Attached to a standard 1/4-wave Vertical polarized reference antenna.
   - Acts as the **global experiment orchestrator**.
   - Interfaces via USB Serial (115200 baud) with the Linux PC.
   - Controls local Reference Slave via Serial1 UART.
   - Controls remote Test Node Master via ESP-NOW.

2. **Reference Node Slave (`ref_node_slave/ref_node_slave.ino`)**:
   - Attached to a standard 1/4-wave Horizontal polarized reference antenna.
   - Operates strictly under Serial1 UART commands from Reference Master.

3. **Test Node Master (`test_node_master/test_node_master.ino`)**:
   - Attached to the **Antenna Under Test (Test-A)**.
   - Receives orchestration commands over ESP-NOW from Reference Master.
   - Controls local Test Node Slave via Serial1 UART.

4. **Test Node Slave (`test_node_slave/test_node_slave.ino`)**:
   - Attached to a 1/4-wave reference antenna on the test jig (**Test-R**).
   - Operates strictly under Serial1 UART commands from Test Master.

---

## 2. 4-Phase Directional Measurement Matrix

For each target Wi-Fi channel (1 through 13), the system executes a 4-phase matrix to measure all transmit-receive antenna combinations:

| Phase | Phase Name | Transmitting Radio | Receiving Radios | Purpose |
| :---: | :--- | :--- | :--- | :--- |
| **0** | `REF_V_TX` | **Ref Master** (1/4 λ V-Pol) | Test Master, Test Slave | Measure V-pol reference transmit to Test & Ref antennas |
| **1** | `REF_H_TX` | **Ref Slave** (1/4 λ H-Pol) | Test Master, Test Slave | Measure H-pol reference transmit to Test & Ref antennas |
| **2** | `TEST_A_TX` | **Test Master** (Test Antenna) | Ref Master, Ref Slave | Measure Test Antenna transmit to V & H reference antennas |
| **3** | `TEST_R_TX` | **Test Slave** (1/4 λ Ref Ant) | Ref Master, Ref Slave | Measure Test-node Reference transmit to V & H reference antennas |

This yields a complete $2 \times 2$ directional channel matrix per channel:

$$\mathbf{H}(f) = \begin{bmatrix} H_{V \to A}(f) & H_{V \to R}(f) \\ H_{H \to A}(f) & H_{H \to R}(f) \end{bmatrix}$$

---

## 3. Synchronized Protocol & Execution Flow

To ensure accurate packet counts and uncorrupted CSI measurements, **ESP-NOW and Serial1 control traffic are completely silent during the RF test window**.

### Protocol Timing Flow

```
CONTROL CHANNEL (Ch 1)            TEST CHANNEL (Ch N)             CONTROL CHANNEL (Ch 1)
─────────────────────             ───────────────────             ─────────────────────

  1. SYNC_REQ / RESP
     (Measure Clock Offset)
              │
  2. PREPARE  │
     (Send params to all nodes)
              │
  3. READY    │
     (Acknowledge armed state)
              │
  4. START_AT(t_start) ─────────► [Channel Switch]
     (Scheduled start timestamp)          │
                                 5. RF Window (e.g. 400ms)
                                    - Fixed TX rate (500 pkts/s)
                                    - Raw 802.11 Action Frames
                                    - Promiscuous CSI Capture
                                    - NO ESP-NOW / UART
                                          │
                                 6. Return Guard (100ms) ────────► [Channel Switch]
                                                                         │
                                                                   7. RESULT Exchange
                                                                      (Serial1 & ESP-NOW)
                                                                         │
                                                                   8. Log output to USB PC
```

1. **Clock Sync**: Reference Master computes microsecond clock offsets ($\Delta t$) using 4-timestamp RTT exchanges over ESP-NOW and Serial1.
2. **PREPARE**: Orchestrator sends test parameters (run ID, test ID, channel, phase, window duration, target MAC).
3. **READY**: Radios arm themselves and respond on Channel 1.
4. **START_AT**: Reference Master calculates a future start timestamp ($t_{\text{start}} = t_{\text{local}} + 300\text{ ms}$) translated to each node's local clock domain.
5. **Channel Switch**: All 4 radios switch to the target RF test channel during the guard window.
6. **RF Measurement Window**: Transmitter fires custom raw 802.11 vendor-action frames at a fixed rate (500 frames/sec). Receivers capture RSSI, packet statistics, and 32 OFDM CSI complex subcarrier bins via low-level ESP-IDF Wi-Fi callbacks.
7. **Return & Result Exchange**: Radios switch back to Channel 1. Slaves return `RFResultSummary` over Serial1; Test Master aggregates both test node results into `ResultPacket` and sends it via ESP-NOW to Reference Master.
8. **PC Stream**: Reference Master formats all four radio results into single-line CSV data streams over USB Serial.

---

## 4. Hardware Assembly & Connection Guide

### Pin Connections

For each Node pair (Reference Master $\leftrightarrow$ Reference Slave, and Test Master $\leftrightarrow$ Test Slave):

| ESP32 Master Pin | ESP32 Slave Pin | Signal Function | Baud Rate |
| :---: | :---: | :---: | :---: |
| **GPIO 17 (TX1)** | **GPIO 16 (RX1)** | Master TX $\to$ Slave RX | 460800 |
| **GPIO 16 (RX1)** | **GPIO 17 (TX1)** | Master RX $\leftarrow$ Slave TX | 460800 |
| **GND** | **GND** | Common Ground | — |

*Note: Ensure both ESP32 boards in a node share a solid, low-impedance ground connection.*

---

## 5. Software Setup & Flashing Instructions

### Prerequisites
- [Arduino IDE](https://www.arduino.cc/en/software) or `arduino-cli` / `platformio`
- ESP32 Board Support Package (version 2.x or 3.x)

### 1. Configure Hardware MAC Addresses
Obtain the MAC addresses of your ESP32 boards (printed to Serial at boot). Update the MAC definitions in the master firmwares:

- In `ref_node_master/ref_node_master.ino`:
  ```cpp
  static uint8_t TEST_MASTER_MAC[6] = { 0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC }; // Test Master MAC
  ```
- In `test_node_master/test_node_master.ino`:
  ```cpp
  static uint8_t REF_MASTER_MAC[6] = { 0x24, 0x6F, 0x28, 0x11, 0x22, 0x33 }; // Reference Master MAC
  ```

### 2. Flash Firmware Sketches
Flash the four sketches to their respective boards:
1. `ref_node_master/ref_node_master.ino` $\to$ Reference Master (Ref V-Pol)
2. `ref_node_slave/ref_node_slave.ino` $\to$ Reference Slave (Ref H-Pol)
3. `test_node_master/test_node_master.ino` $\to$ Test Master (Test Antenna)
4. `test_node_slave/test_node_slave.ino` $\to$ Test Slave (Test Reference)

---

## 6. Linux PC Integration & Data Processing

The Reference Master connects to the Linux computer via USB Serial (115200 baud).

### Serial Output Format
The Reference Master streams measurement records in structured CSV lines:

```csv
DATA,run=1,test=1,ch=1,phase=0,node=REF_MASTER,ant=REF_V_ANT,mode=2,frames=0,pkts=200,match=200,rssi_avg=0.00,rssi_std=0.00,min=127,max=-127,csi_i=[0;0;...],csi_q=[0;0;...]
DATA,run=1,test=1,ch=1,phase=0,node=TEST_MASTER,ant=TEST_ANT,mode=1,frames=195,pkts=200,match=195,rssi_avg=-58.40,rssi_std=1.10,min=-62,max=-55,csi_i=[10;12;14;...],csi_q=[4;6;8;...]
```

### Running the Integration Python Script

Install dependencies:
```bash
pip install pyserial matplotlib
```

#### Live Collection Mode:
Connect the Reference Master via USB and start capturing:
```bash
python3 scripts/process_results.py --port /dev/ttyUSB0 --baud 115200 --csv sweep_results.csv --json antenna_results.json --plot
```

#### Log File Processing Mode:
If you saved raw terminal output to a text file:
```bash
python3 scripts/process_results.py --file raw_output.log --csv results.csv --json results.json --plot
```

### Outputs
- **CSV Data (`sweep_results.csv`)**: Contains PER, average RSSI, min/max RSSI, RSSI standard deviation, and semicolon-separated CSI real ($I$) and imaginary ($Q$) subcarrier arrays.
- **JSON Export (`antenna_results.json`)**: Structured dataset containing calculated CSI magnitudes ($\sqrt{I^2 + Q^2}$) per subcarrier bin.
- **Plots (`rf_antenna_test_rssi_vs_channel.png`, `rf_antenna_test_csi_spectrum.png`)**:
  - Average Received RSSI across Channels 1–13.
  - Subcarrier CSI Frequency Response spectrum for the Antenna Under Test.
