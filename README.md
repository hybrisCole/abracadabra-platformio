# abracadabra-platformio

PlatformIO firmware for **Seeed Studio XIAO nRF52840 Sense** (`board = xiaoblesense`, Nordic nRF52 Arduino mbed core).

## IMU (LSM6DS3TR-C)

This board’s onboard **6-axis IMU is the STMicroelectronics LSM6DS3TR-C** (accelerometer + gyroscope).

Seeed documents that same part on the **XIAO nRF54L15 Sense** in:

**[Seeed Studio XIAO nRF54L15 Sense built-in Sensor](https://wiki.seeedstudio.com/xiao_nrf54l15_sense_built_in_sensor/)**

That page applies to the **nRF54** variant (different MCU and toolchain examples—e.g. Zephyr samples there), but the **sensor IC is identical** (LSM6DS3TR-C). Conceptual notes on axes, accel/gyro roles, and motion use cases carry over when you read the IMU on **nRF52840 Sense**.

For this **nRF52840** project, driver code typically uses the **`Seeed Arduino LSM6DS3`** library (see `platformio.ini` → `lib_deps`), matching the same chip.

### Related Seeed docs (nRF52840 Sense)

- [Getting Started with Seeed Studio XIAO nRF52840 Series](https://wiki.seeedstudio.com/XIAO_BLE/) — pin map lists **LSM6DS3TR-C** on the Sense variant.
