# abracadabra-platformio

PlatformIO firmware for **Seeed Studio XIAO nRF52840 Sense** (`board = xiaoblesense`, Nordic nRF52 Arduino mbed core).

Companion mobile app: **`abracadabra-rnapp`** (React Native) implements scan/link, **META + GATT pull** recording transfer, and tabbed timeline charts.

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

5. **Post-accept IMU recording (up to ~4 s wall clock)**  
   After an **accepted** double tap, firmware plays the playful LED cue, then captures raw accel + gyro (**not** including that cue—the recording starts **after** the animation). The loop stops when **`millis()` elapsed ≥ `kRecordDurationMs`** (default **4000 ms**) or the ring buffer max count is reached. While capturing, the LED shows a **flashing green** “REC” pattern (`recordingLedTick`).

   - **Nominal timeline:** each stored sample sets **`t_ms = sample_index × kRecordSamplePeriodMs`** (default period **5 ms** → ~200 Hz **grid**). That timestamp is for plotting / ML alignment, **not** guaranteed wall spacing if the main loop is busy (BLE polling, I²C). You may see **fewer samples** than `kRecordDurationMs / 5` over the same wall window; the app’s **Δt** label reflects **`max(t_ms) − min(t_ms)`** on received samples, not “exactly 4000 ms”.
   - **Constants:** `kRecordDurationMs`, `kRecordSamplePeriodMs`, `kRecordMaxSamples` in `src/main.cpp`.

6. **Serial during recordings**  
   Human-readable **CSV dumps after each capture are disabled** (Serial noise). Inspect captures via the phone app (GATT pull after **META**). Serial still logs tap snapshots and **`BLE: META …`** / **`BLE: META sent — central should GATT-pull payload …`** tracing.

   **Packing:** `packImuSampleBleLittleEndian()` writes **14 bytes/sample** (LE **`uint16_t t_ms`**, then six LE **`int16_t`** raw axes). The entire packed buffer lives in RAM until the central finishes pulling it or the next capture overwrites it.

7. **Serial**  
   **115200 baud** (`platformio.ini` → `monitor_speed`). On boot, firmware waits briefly for USB Serial so logs appear when a monitor is open without blocking forever when USB is unplugged. BLE setup also logs success/failure (`BLE: advertising as "…"` or `advertise() failed`).

8. **Bluetooth Low Energy (ArduinoBLE)**  
   The board runs as a **connectable peripheral** so centrals (e.g. the companion **abracadabra-rnapp** React Native project) can discover it.

   - **Friendly name:** `kBleDeviceName` in `src/main.cpp` → `BLE.setDeviceName()` (GAP Device Name; mbed defaults to `"Arduino"` if omitted per [ArduinoBLE `setDeviceName`](https://github.com/arduino-libraries/ArduinoBLE/blob/master/docs/api.md)) plus advertising / scan response via `BLEAdvertisingData` and `BLE.setLocalName()`.
   - **Advertising:** Primary payload carries **flags + Complete Local Name** so the name fits in **31 bytes**. A **128-bit service UUID is not broadcast** in ADV (it would crowd out the name); the custom service still exists **on GATT** after connect.
   - **Custom GATT:** Service **`ADAB0001-0000-1000-8000-00805F9B34FB`**
     - Read-only byte **`ADAB0002-…`** (status placeholder).
     - **`ADAB0003-…` (NOTIFY / write response channel):** After each **accepted** recording, firmware notifies **one framed META packet** only (no bulk payload in notify). Frame header on every transmission: magic **`0xADAB`** (LE `uint16`), **`pkt`** byte (`META = 1`), reserved **`0`**. **META payload (16 bytes LE):** `window_id` u16, `sample_count` u16, `total_bytes` u32 (`sample_count × 14`), `proto_ver` u8 + 3 reserved bytes, **`crc_ieee_u32`** over the **full packed payload** the central must pull.
     - **`ADAB0004-…` (write):** Central writes **4-byte little-endian byte offset** into the staged packed recording. Peripheral prepares **`ADAB0005`** read data for the **next** GATT read (pull model in `src/main.cpp`).
     - **`ADAB0005-…` (read):** Central reads the slice staged after the last **`ADAB0004`** write. Repeat until **`total_bytes`** are read; verify CRC against **META**.
     - Sample packing in RAM / over the wire: LE **`uint16_t t_ms`**, then six LE **`int16_t`** accel + gyro (`packImuSampleBleLittleEndian`).
   - **Link-up cue:** When a central **first connects** (GAP link established after “pairing” from the phone), firmware runs **`bleHandshakeLedCue()`** — three quick **cyan ↔ magenta** bursts on the RGB LED (distinct from the double-tap rainbow). Serial prints `BLE: central connected (link up).` Disconnect clears the latch so the next connection flashes again.
   - **Main loop:** **`blePollServicing()`** wraps **`BLE.poll()`** and is used everywhere the stack is serviced so connection edges are detected while waiting for double-tap (`idleUntilDoubleTapInterrupt`), during LED cues, recording, and NOTIFY chunk pacing.

   Dependencies: [`arduino-libraries/ArduinoBLE`](https://github.com/arduino-libraries/ArduinoBLE) in `platformio.ini`. The React Native app handles **META** on **`ADAB0003`**, pulls bytes via **`ADAB0004`**/**`ADAB0005`**, and validates **CRC** (failed pulls discarded).

## IMU (LSM6DS3TR-C)

This board’s onboard **6-axis IMU is the STMicroelectronics LSM6DS3TR-C** (accelerometer + gyroscope). Double-tap uses the accelerometer path per ST application guidance; gyro is enabled at **416 Hz**, **±2000 °/s**, alongside **±2 g** accel for snapshots and scaling consistency with the Seeed driver settings.

Seeed documents that same part on the **XIAO nRF54L15 Sense** in:

**[Seeed Studio XIAO nRF54L15 Sense built-in Sensor](https://wiki.seeedstudio.com/xiao_nrf54l15_sense_built_in_sensor/)**

That page applies to the **nRF54** variant (different MCU and toolchain examples—e.g. Zephyr samples there), but the **sensor IC is identical** (LSM6DS3TR-C). Conceptual notes on axes, accel/gyro roles, and motion use cases carry over when you read the IMU on **nRF52840 Sense**.

For this **nRF52840** project, driver code uses the **`Seeed Arduino LSM6DS3`** library (see `platformio.ini` → `lib_deps`), matching the same chip.

### Related Seeed docs (nRF52840 Sense)

- [Getting Started with Seeed Studio XIAO nRF52840 Series](https://wiki.seeedstudio.com/XIAO_BLE/) — pin map lists **LSM6DS3TR-C** on the Sense variant.

## Gesture capture model

The accepted double tap is the **human start signal**, not part of the ML clip: pose gate → playful LEDs → **then** a fresh IMU window (**up to ~4 s** wall clock per **`kRecordDurationMs`**) for whatever motion follows.

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
| **Payload size** | Each stored sample is **14 bytes**: **2 bytes** nominal `t_ms` (`uint16_t` LE) + **12 bytes** raw axes (**six** `int16_t` LE). A **full** 4 s capture at **200 Hz** would be **≈ 800 × 14 ≈ 11.2 KB**; **416 Hz** would be roughly double (~23 KB). Real captures may be smaller if fewer samples were stored in **`kRecordDurationMs`**. |
| **Nominal vs actual sample count** | **`t_ms`** is **`index × period`** (ideal grid). Busy loops (BLE polling, I²C) can yield **fewer samples** in **`kRecordDurationMs`** wall time than `duration / period`; the companion app’s timeline shows the resulting **`t_ms`** span. |
| **BLE pull throughput** | Payload is read in **GATT read** slices after **META**. Transfer time depends on **connection interval**, **ATT MTU**, and **central read scheduling**. Larger MTU reduces overhead per round-trip; the RN app requests **MTU 247** on Android when supported. |

**Practical note:** **200 Hz** nominal grid is a good default for wrist taps and rotations before chasing full **416 Hz** capture (more samples, more airtime, diminishing returns for many classifiers).

**Status:** Custom service, **META** notify, **GATT pull** (`ADAB0004` / `ADAB0005`), **CRC**, and **14-byte LE** packing are implemented in `src/main.cpp` and consumed by **`abracadabra-rnapp`**.

## Build and upload

From the project directory:

```bash
pio run -e xiaoblesense_arduinocore_mbed -t upload
pio device monitor -b 115200
```

## Tuning

Edit the constants at the top of **`src/main.cpp`**: settle time, sample count/spacing, accel acceptance bands, gyro stillness threshold, and **`kRecordDurationMs` / `kRecordSamplePeriodMs`** for the recording window. For stricter pose matching later, consider replacing axis-aligned boxes with a **normalized gravity vector + dot-product** threshold against a calibrated reference pose.
