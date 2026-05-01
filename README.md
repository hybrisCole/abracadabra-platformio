# abracadabra-platformio

PlatformIO firmware for **Seeed Studio XIAO nRF52840 Sense** (`board = xiaoblesense`, Nordic nRF52 Arduino mbed core).

## What this firmware does

1. **Hardware double-tap** on the onboard **LSM6DS3TR-C** raises **INT1** (pin routed per board). The MCU wakes from sleep (`__WFI()`), confirms `DOUBLE_TAP` in **`TAP_SRC`**, then proceeds.

2. **Settled IMU read** (not at the instant of impact): after confirmation, firmware waits **120 ms**, then averages **10** accel + gyro samples **5 ms** apart. That reduces tap shock and favors a **gravity / tilt** estimate for wearable pose.

3. **Pose gate (“accepted orientation”)**  
   Command acceptance requires **both**:
   - **Accelerometer** (sensor frame, settled average in **g**): each axis inside the box below (constants `kAcceptedAccel*G` in `src/main.cpp`).
   - **Gyroscope** (“still enough”): absolute value on **each** axis ≤ **`kStillGyroMaxDps`** (°/s).

   **Acceptance windows (current defaults):**

   | Quantity | Range |
   |----------|--------|
   | Accel **X** | **−0.25 g … +0.25 g** |
   | Accel **Y** | **+0.60 g … +0.90 g** |
   | Accel **Z** | **−0.75 g … −0.35 g** |
   | Gyro **X, Y, Z** (each) | **≤ 40 °/s** (absolute value) |

   In plain language: the accepted pose is an **orientation relative to gravity** where X is near neutral, Y is moderately positive, and Z is moderately negative—tuned for one wrist-held pose; change the constants if your mounting differs.

4. **RGB LED cue**  
   The playful multi-color sequence runs **only** when the double tap is **accepted** by the pose gate. Rejected double taps still emit the **Serial** snapshot (with `Command gate: REJECTED`) but **no** LED animation.

5. **Post-accept IMU recording (4 seconds)**  
   After an **accepted** double tap, firmware plays the playful LED cue, then captures a **4 second** window of raw accel + gyro (**not** including that cue in the clip—the recording starts **after** the animation). While capturing, the LED shows a **flashing green** “REC” pattern (`recordingLedTick`).

   - **Rate:** nominally **~200 Hz** (`kRecordSamplePeriodMs = 5`). See [Sample rate vs BLE](#sample-rate-vs-ble) below.
   - **Constants:** `kRecordDurationMs`, `kRecordSamplePeriodMs`, buffer sizing in `src/main.cpp`.

6. **Serial dump of recordings (bring-up)**  
   After each recording, firmware prints **human CSV**:

   - Header metadata (`window_id`, nominal sample rate, row count) plus lines:  
     `t_ms,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw`  
     (`t_ms` is nominal: sample index × period, for a stable ML timeline.)

   **BLE framing helper:** `packImuSampleBleLittleEndian()` writes **14 bytes/sample** (little-endian `uint16_t t_ms`, then six `int16_t` raw axes). Reserved for GATT notifications—no hex mirror on Serial.

7. **Serial**  
   **115200 baud** (`platformio.ini` → `monitor_speed`). On boot, firmware waits briefly for USB Serial so logs appear when a monitor is open without blocking forever when USB is unplugged. BLE setup also logs success/failure (`BLE: advertising as "…"` or `advertise() failed`).

8. **Bluetooth Low Energy (ArduinoBLE)**  
   The board runs as a **connectable peripheral** so centrals (e.g. the companion **abracadabra-rnapp** React Native project) can discover it.

   - **Friendly name:** `kBleDeviceName` in `src/main.cpp` → `BLE.setDeviceName()` (GAP Device Name; mbed defaults to `"Arduino"` if omitted per [ArduinoBLE `setDeviceName`](https://github.com/arduino-libraries/ArduinoBLE/blob/master/docs/api.md)) plus advertising / scan response via `BLEAdvertisingData` and `BLE.setLocalName()`.
   - **Advertising:** Primary payload carries **flags + Complete Local Name** so the name fits in **31 bytes**. A **128-bit service UUID is not broadcast** in ADV (it would crowd out the name); the custom service still exists **on GATT** after connect.
   - **Custom GATT (placeholder):** Service **`ADAB0001-0000-1000-8000-00805F9B34FB`**, read-only byte characteristic **`ADAB0002-0000-1000-8000-00805F9B34FB`** — extend when streaming IMU.
   - **Main loop:** `BLE.poll()` runs with IMU idle (`__WFI()`), recording waits, and between tap handling so the stack stays serviced.

   Dependencies: [`arduino-libraries/ArduinoBLE`](https://github.com/arduino-libraries/ArduinoBLE) in `platformio.ini`. **Notify/streaming IMU over BLE** is still the next step after this advertising bring-up.

## IMU (LSM6DS3TR-C)

This board’s onboard **6-axis IMU is the STMicroelectronics LSM6DS3TR-C** (accelerometer + gyroscope). Double-tap uses the accelerometer path per ST application guidance; gyro is enabled at **416 Hz**, **±2000 °/s**, alongside **±2 g** accel for snapshots and scaling consistency with the Seeed driver settings.

Seeed documents that same part on the **XIAO nRF54L15 Sense** in:

**[Seeed Studio XIAO nRF54L15 Sense built-in Sensor](https://wiki.seeedstudio.com/xiao_nrf54l15_sense_built_in_sensor/)**

That page applies to the **nRF54** variant (different MCU and toolchain examples—e.g. Zephyr samples there), but the **sensor IC is identical** (LSM6DS3TR-C). Conceptual notes on axes, accel/gyro roles, and motion use cases carry over when you read the IMU on **nRF52840 Sense**.

For this **nRF52840** project, driver code uses the **`Seeed Arduino LSM6DS3`** library (see `platformio.ini` → `lib_deps`), matching the same chip.

### Related Seeed docs (nRF52840 Sense)

- [Getting Started with Seeed Studio XIAO nRF52840 Series](https://wiki.seeedstudio.com/XIAO_BLE/) — pin map lists **LSM6DS3TR-C** on the Sense variant.

## Gesture capture model

The accepted double tap is the **human start signal**, not part of the ML clip: pose gate → playful LEDs → **then** a fresh **4 s** IMU window for whatever motion follows.

Recommended logical columns when you export or label data:

```csv
session_id,window_id,t_ms,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw
```

`window_id` increments per accepted recording on device; `t_ms` starts at **0** at the beginning of that post-cue window.

### Sample rate vs BLE

| Topic | Notes |
|--------|--------|
| **Default rate (this project)** | Firmware records at **~200 Hz** (`kRecordSamplePeriodMs = 5`). That sharpens tap edges versus **100 Hz** while staying below the IMU’s **416 Hz** ODR. |
| **Hardware headroom** | IMU ODR is **416 Hz**. **`kRecordSamplePeriodMs = 5`** targets **~200 Hz**; shorter periods raise the rate further but need enough CPU + I²C time per tick and a larger `kRecordMaxSamples` buffer. |
| **Payload size** | Each stored sample is **14 bytes**: **2 bytes** nominal `t_ms` (`uint16_t` LE) + **12 bytes** raw axes (**six** `int16_t` LE). **4 s @ 200 Hz ≈ 800 × 14 ≈ 11.2 KB**. **4 s @ 416 Hz** would be roughly double that (~23 KB)—possible but heavier on BLE airtime. |
| **BLE transmission time** | Transfer time depends on **connection interval**, **ATT MTU**, and **how many bytes per notification**. Larger MTU (e.g. 185–247) and packing multiple samples per packet reduce overhead. At **~11 KB** per window, streaming in chunks is still practical; **~20+ KB** takes proportionally longer—plan **~1–3× real-time** for naive notify-per-line unless you optimize. |

**Practical note:** **200 Hz** is a good default for wrist taps and rotations before chasing full **416 Hz** capture (more samples, more BLE time, diminishing returns for many classifiers).

**Peripheral advertising + GATT placeholder** are implemented; **streaming recording windows over notify** is still **planned**. The **14-byte LE** packing helper lives in `src/main.cpp`.

## Build and upload

From the project directory:

```bash
pio run -e xiaoblesense_arduinocore_mbed -t upload
pio device monitor -b 115200
```

## Tuning

Edit the constants at the top of **`src/main.cpp`**: settle time, sample count/spacing, accel acceptance bands, gyro stillness threshold, and **`kRecordDurationMs` / `kRecordSamplePeriodMs`** for the recording window. For stricter pose matching later, consider replacing axis-aligned boxes with a **normalized gravity vector + dot-product** threshold against a calibrated reference pose.
