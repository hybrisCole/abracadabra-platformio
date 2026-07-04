#include <Arduino.h>
#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <math.h>
#include <string.h>

/** Shown in BLE scan lists. Also call BLE.setDeviceName — mbed defaults GAP name to "Arduino" if unset (ArduinoBLE docs). */
static constexpr char kBleDeviceName[] = "XA_Abracadabra";

/**
 * Minimal GATT so Nordic stacks reliably advertise; RN app scans by name for now.
 * Service UUID reserved for future Abracadabra data — change characteristics when streaming IMU.
 */
BLEService abracadabraService("ADAB0001-0000-1000-8000-00805F9B34FB");
BLEUnsignedCharCharacteristic abracadabraStatusChar(
    "ADAB0002-0000-1000-8000-00805F9B34FB", BLERead);
/** Framed notify channel: status, META, and iOS-optimized chunk transfer. */
BLECharacteristic abracadabraStreamChar(
    "ADAB0003-0000-1000-8000-00805F9B34FB", BLENotify, 244);
/** Compatibility fallback: central writes uint32 LE byte offset, then reads ADAB0005 for that slice. */
BLECharacteristic abracadabraPullCtrlChar(
    "ADAB0004-0000-1000-8000-00805F9B34FB", BLEWrite, 4);
BLECharacteristic abracadabraPullDataChar(
    "ADAB0005-0000-1000-8000-00805F9B34FB", BLERead, 244);

static constexpr uint16_t kBleFrameMagic = 0xADAB;
static constexpr uint8_t kBlePktMeta = 1;
static constexpr uint8_t kBlePktChunk = 2;
static constexpr uint8_t kBlePktCommit = 3;
/** Payload 4 B: window_id u16 LE, proto_ver u8, reserved u8 — sent immediately before IMU capture starts (central UI aligns with data). */
static constexpr uint8_t kBlePktRecordingPending = 4;
static constexpr uint8_t kBleProtoVer = 1;
/** iOS commonly exposes ATT_MTU 185, so notify values must stay <= 182 bytes (MTU - 3). */
static constexpr size_t kBleNotifyFrameMaxBytes = 182;
static constexpr size_t kBleGattReadMaxBytes = 244;
static constexpr size_t kBleFrameHeaderBytes = 4;
static constexpr size_t kBleChunkPayloadHeaderBytes = 8;  // window_id u16 + offset u32 + data_len u16
static constexpr size_t kBleChunkDataMaxBytes =
    kBleNotifyFrameMaxBytes - kBleFrameHeaderBytes - kBleChunkPayloadHeaderBytes;
/** Give iOS/CoreBluetooth time to drain each notify; tune down only after on-device testing. */
static constexpr uint8_t kBleNotifyChunkPaceMs = 15;
/** COMMIT is tiny; repeat it so the app sees an end marker even if one notify is dropped. */
static constexpr uint8_t kBleCommitRepeatCount = 3;

// RGB LEDs — active LOW on Seeed XIAO BLE Sense (mbed core)
#define LED_R LEDR
#define LED_G LEDG
#define LED_B LEDB

#ifndef PIN_LSM6DS3TR_C_INT1
#error This sketch expects Seeed XIAO nRF52840 Sense (LSM6DS3 INT1 routed to D17).
#endif

LSM6DS3 imu;

static volatile bool imuIntPending;

static constexpr uint16_t kPostTapSettleMs = 120;
static constexpr uint8_t kSnapshotSamples = 10;
static constexpr uint8_t kSnapshotSampleSpacingMs = 5;

static constexpr float kAcceptedAccelXMinG = -0.25f;
static constexpr float kAcceptedAccelXMaxG = 0.25f;
static constexpr float kAcceptedAccelYMinG = 0.60f;
static constexpr float kAcceptedAccelYMaxG = 0.90f;
static constexpr float kAcceptedAccelZMinG = -0.75f;
static constexpr float kAcceptedAccelZMaxG = -0.35f;
static constexpr float kStillGyroMaxDps = 40.0f;

/** Post-acceptance IMU capture for ML / BLE (see README). */
static constexpr uint16_t kRecordDurationMs = 4000;
static constexpr uint8_t kRecordSamplePeriodMs = 5;  // ~200 Hz

static constexpr uint16_t kRecordMaxSamples =
    static_cast<uint16_t>(kRecordDurationMs / kRecordSamplePeriodMs + 1);

struct ImuRecordSample {
  uint16_t t_ms;
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
};

static ImuRecordSample sRecordBuf[kRecordMaxSamples];
static uint16_t sRecordingWindowId;

static void imuInterruptHandler() {
  imuIntPending = true;
}

static void ledOff() {
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
}

static void ledSet(uint8_t r, uint8_t g, uint8_t b) {
  digitalWrite(LED_R, r ? LOW : HIGH);
  digitalWrite(LED_G, g ? LOW : HIGH);
  digitalWrite(LED_B, b ? LOW : HIGH);
}

/** Tracks BLE central connection for edge-triggered “handshake” LED cue. */
static bool sBleCentralWasConnected = false;

static void blePollServicing();

static void serviceBlePullRequests();

/** Short cyan/magenta bursts — distinct from double-tap rainbow; runs during link-up. */
static void bleHandshakeLedCue() {
  constexpr uint16_t stepMs = 70;
  for (int i = 0; i < 3; i++) {
    ledSet(0, 1, 1);  // cyan
    delay(stepMs);
    blePollServicing();
    ledSet(1, 0, 1);  // magenta
    delay(stepMs);
    blePollServicing();
  }
  ledOff();
}

/** Process BLE events and flash LEDs once when a central connects (link established). */
static void blePollServicing() {
  BLE.poll();
  serviceBlePullRequests();
  BLEDevice central = BLE.central();
  const bool connected = central && central.connected();
  if (connected && !sBleCentralWasConnected) {
    sBleCentralWasConnected = true;
    Serial.println(F("BLE: central connected (link up)."));
    bleHandshakeLedCue();
  } else if (!connected) {
    sBleCentralWasConnected = false;
  }
}

/** ST AN5130 §5.5.5 — double-tap to INT1. Gyro at same ODR as XL for snapshot reads. */
static bool configureDoubleTapHardware() {
  if (imu.beginCore() != IMU_SUCCESS) {
    return false;
  }

  uint8_t whoAmI = 0;
  imu.readRegister(&whoAmI, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
  imu.settings.tempSensitivity =
      (whoAmI == LSM6DS3_C_ACC_GYRO_WHO_AM_I) ? 256 : 16;  // TR-C vs LSM6DS3
  imu.settings.accelRange = 2;   // must match CTRL1_XL ±2 g so readFloatAccel is scaled correctly
  imu.settings.gyroRange = 2000; // must match CTRL2_G ±2000 dps

  // Gyro: ODR = 416 Hz, FS = ±2000 dps (library calcGyro matches 2000)
  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x6C);

  // XL: ODR_XL = 416 Hz, FS = ±2 g  (matches AN5130 tap examples)
  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x60);

  // Block data update — cleaner reads when polling status
  uint8_t ctrl3 = 0;
  imu.readRegister(&ctrl3, LSM6DS3_ACC_GYRO_CTRL3_C);
  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, static_cast<uint8_t>(ctrl3 | 0x40));  // BDU

  imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x8E);      // interrupts + XYZ tap axis enable
  imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x8C);     // tap threshold (AN double-tap example)
  imu.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, 0x7F);       // shock / quiet / duration windows
  imu.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80);    // enable single + double tap recognition
  imu.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x08);      // double-tap → INT1 (same bit map as INT1_TAP)

  uint8_t tapSrc = 0;
  imu.readRegister(&tapSrc, LSM6DS3_ACC_GYRO_TAP_SRC);      // clear latched status after config
  (void)tapSrc;

  return true;
}

static void attachImuInterrupt() {
  imuIntPending = false;
  attachInterrupt(digitalPinToInterrupt(PIN_LSM6DS3TR_C_INT1), imuInterruptHandler, RISING);
}

static void detachImuInterrupt() {
  detachInterrupt(digitalPinToInterrupt(PIN_LSM6DS3TR_C_INT1));
}

/** Reads TAP_SRC; sets *tapSrcOut to full register if non-null. */
static bool readWasDoubleTap(uint8_t* tapSrcOut) {
  uint8_t tapSrc = 0;
  imu.readRegister(&tapSrc, LSM6DS3_ACC_GYRO_TAP_SRC);
  if (tapSrcOut != nullptr) {
    *tapSrcOut = tapSrc;
  }
  return (tapSrc & 0x10) != 0;  // DOUBLE_TAP bit in TAP_SRC
}

static bool isInsideRange(float value, float minimum, float maximum) {
  return value >= minimum && value <= maximum;
}

/** Human-readable IMU snapshot after tap shock settles (Serial Monitor). */
static bool serialPrintImuSnapshot(uint8_t tapSrc) {
  int32_t axSum = 0;
  int32_t aySum = 0;
  int32_t azSum = 0;
  int32_t gxSum = 0;
  int32_t gySum = 0;
  int32_t gzSum = 0;

  delay(kPostTapSettleMs);

  for (uint8_t i = 0; i < kSnapshotSamples; ++i) {
    axSum += imu.readRawAccelX();
    aySum += imu.readRawAccelY();
    azSum += imu.readRawAccelZ();
    gxSum += imu.readRawGyroX();
    gySum += imu.readRawGyroY();
    gzSum += imu.readRawGyroZ();

    if (i + 1 < kSnapshotSamples) {
      delay(kSnapshotSampleSpacingMs);
    }
  }

  const float ax = static_cast<float>(axSum) / kSnapshotSamples;
  const float ay = static_cast<float>(aySum) / kSnapshotSamples;
  const float az = static_cast<float>(azSum) / kSnapshotSamples;
  const float gx = static_cast<float>(gxSum) / kSnapshotSamples;
  const float gy = static_cast<float>(gySum) / kSnapshotSamples;
  const float gz = static_cast<float>(gzSum) / kSnapshotSamples;
  const float axG = imu.calcAccel(static_cast<int16_t>(ax));
  const float ayG = imu.calcAccel(static_cast<int16_t>(ay));
  const float azG = imu.calcAccel(static_cast<int16_t>(az));
  const float gxDps = imu.calcGyro(static_cast<int16_t>(gx));
  const float gyDps = imu.calcGyro(static_cast<int16_t>(gy));
  const float gzDps = imu.calcGyro(static_cast<int16_t>(gz));
  const bool acceptedPose =
      isInsideRange(axG, kAcceptedAccelXMinG, kAcceptedAccelXMaxG) &&
      isInsideRange(ayG, kAcceptedAccelYMinG, kAcceptedAccelYMaxG) &&
      isInsideRange(azG, kAcceptedAccelZMinG, kAcceptedAccelZMaxG);
  const bool stillEnough =
      fabsf(gxDps) <= kStillGyroMaxDps &&
      fabsf(gyDps) <= kStillGyroMaxDps &&
      fabsf(gzDps) <= kStillGyroMaxDps;
  const bool acceptedCommand = acceptedPose && stillEnough;

  Serial.println();
  Serial.println(F("======== Double tap — settled IMU snapshot ========"));
  Serial.print(F("Snapshot timing: waited "));
  Serial.print(kPostTapSettleMs);
  Serial.print(F(" ms, then averaged "));
  Serial.print(kSnapshotSamples);
  Serial.print(F(" samples spaced "));
  Serial.print(kSnapshotSampleSpacingMs);
  Serial.println(F(" ms apart."));
  Serial.println(acceptedCommand ? F("Command gate: ACCEPTED (pose inside range and wrist settled).")
                                 : F("Command gate: REJECTED (pose outside range or wrist still moving)."));

  Serial.println(F("--- TAP_SRC (tap detection flags, Register TAP_SRC) ---"));
  Serial.print(F("  Raw TAP_SRC byte: 0x"));
  Serial.println(tapSrc, HEX);
  Serial.println(F("  Bit Z_TAP (1): tap impulse mainly on Z axis of sensor."));
  Serial.println(F("  Bit Y_TAP (2): tap impulse mainly on Y axis."));
  Serial.println(F("  Bit X_TAP (4): tap impulse mainly on X axis."));
  Serial.println(F("  Bit TAP_SIGN (8): direction/sign of tap acceleration pulse."));
  Serial.println(F("  Bit DOUBLE_TAP (0x10): this interrupt was a recognized double tap."));
  Serial.println(F("  Bit SINGLE_TAP (0x20): single-tap event latched here."));
  Serial.println(F("  Bit TAP_EV_STATUS (0x40): tap event detected."));

  Serial.println(F("--- Accelerometer (linear acceleration + gravity, sensor frame) ---"));
  Serial.println(F("  Scale: ±2 g @ 416 Hz; averaged after tap shock settles."));
  Serial.print(F("  Accel X raw avg: "));
  Serial.print(ax, 1);
  Serial.print(F("  | Accel X (g): "));
  Serial.println(axG, 4);
  Serial.print(F("  Accel Y raw avg: "));
  Serial.print(ay, 1);
  Serial.print(F("  | Accel Y (g): "));
  Serial.println(ayG, 4);
  Serial.print(F("  Accel Z raw avg: "));
  Serial.print(az, 1);
  Serial.print(F("  | Accel Z (g): "));
  Serial.println(azG, 4);
  Serial.println(F("  Meaning: quieter gravity/tilt estimate for classifying the wearable pose."));
  Serial.print(F("  Accepted ranges: X "));
  Serial.print(kAcceptedAccelXMinG, 2);
  Serial.print(F(".."));
  Serial.print(kAcceptedAccelXMaxG, 2);
  Serial.print(F(" g, Y "));
  Serial.print(kAcceptedAccelYMinG, 2);
  Serial.print(F(".."));
  Serial.print(kAcceptedAccelYMaxG, 2);
  Serial.print(F(" g, Z "));
  Serial.print(kAcceptedAccelZMinG, 2);
  Serial.print(F(".."));
  Serial.print(kAcceptedAccelZMaxG, 2);
  Serial.println(F(" g."));

  Serial.println(F("--- Gyroscope (angular rate, sensor frame) ---"));
  Serial.println(F("  Scale: ±2000 dps @ 416 Hz; averaged after tap shock settles."));
  Serial.print(F("  Gyro X raw avg: "));
  Serial.print(gx, 1);
  Serial.print(F("  | Gyro X (deg/s): "));
  Serial.println(gxDps, 3);
  Serial.print(F("  Gyro Y raw avg: "));
  Serial.print(gy, 1);
  Serial.print(F("  | Gyro Y (deg/s): "));
  Serial.println(gyDps, 3);
  Serial.print(F("  Gyro Z raw avg: "));
  Serial.print(gz, 1);
  Serial.print(F("  | Gyro Z (deg/s): "));
  Serial.println(gzDps, 3);
  Serial.println(F("  Meaning: residual wrist rotation after the tap; closer to 0 means more settled."));
  Serial.print(F("  Still threshold: each gyro axis must be <= "));
  Serial.print(kStillGyroMaxDps, 1);
  Serial.println(F(" deg/s."));

  Serial.println(F("--- Temperature (die / sensor internal) ---"));
  Serial.print(F("  Temp (C approx): "));
  Serial.println(imu.readTempC(), 2);

  Serial.println(F("===================================================="));
  Serial.println();

  return acceptedCommand;
}

/**
 * Packed BLE payload for one IMU row (little-endian):
 * uint16_t t_ms + int16_t ax, ay, az, gx, gy, gz — **14 bytes** total.
 * Use when streaming via GATT; not printed on Serial for now.
 */
static constexpr size_t kBlePackedSampleBytes = 14;

static void packImuSampleBleLittleEndian(const ImuRecordSample& s,
                                         uint8_t out[kBlePackedSampleBytes]) {
  out[0] = static_cast<uint8_t>(s.t_ms & 0xFF);
  out[1] = static_cast<uint8_t>((s.t_ms >> 8) & 0xFF);
  const int16_t* axes[] = {&s.ax, &s.ay, &s.az, &s.gx, &s.gy, &s.gz};
  uint8_t* p = out + 2;
  for (const int16_t* axis : axes) {
    const uint16_t u = static_cast<uint16_t>(*axis);
    *p++ = static_cast<uint8_t>(u & 0xFF);
    *p++ = static_cast<uint8_t>((u >> 8) & 0xFF);
  }
}

static uint8_t sBlePackedRecording[kRecordMaxSamples * kBlePackedSampleBytes];

/** Fallback pull transfer: valid after META notify succeeds until the next capture overwrites the buffer. */
static bool sBlePullReady = false;
static uint32_t sBlePullTotalBytes = 0;
static uint8_t* sBlePullPackedPtr = nullptr;

/**
 * Central writes 4-byte LE offset to pull-ctrl; we stage pull-data for the following READ.
 */
static void serviceBlePullRequests() {
  if (!abracadabraPullCtrlChar.written()) {
    return;
  }
  uint32_t off = 0;
  if (abracadabraPullCtrlChar.valueLength() >= 4) {
    const uint8_t* v = abracadabraPullCtrlChar.value();
    off = static_cast<uint32_t>(v[0]) | (static_cast<uint32_t>(v[1]) << 8) |
          (static_cast<uint32_t>(v[2]) << 16) | (static_cast<uint32_t>(v[3]) << 24);
  }

  uint8_t sliceBuf[kBleGattReadMaxBytes];
  size_t n = 0;
  if (sBlePullReady && sBlePullPackedPtr != nullptr && off < sBlePullTotalBytes) {
    const uint32_t remain = sBlePullTotalBytes - off;
    const size_t maxLen = sizeof(sliceBuf);
    n = static_cast<size_t>(remain < maxLen ? remain : maxLen);
    memcpy(sliceBuf, sBlePullPackedPtr + off, n);
  }
  abracadabraPullDataChar.writeValue(sliceBuf, static_cast<int>(n));
}

static void putU16Le(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

static void putU32Le(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

/** IEEE CRC-32 (same polynomial as PNG / Ethernet); matches app assembler. */
static uint32_t crc32Ieee(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc >> 1U) ^ (0xEDB88320UL & (uint32_t)-(int32_t)(crc & 1U));
    }
  }
  return crc ^ 0xFFFFFFFFUL;
}

/**
 * Notify one framed packet: magic LE, pkt type, reserved, payload.
 * Returns false if peripheral disconnected or notify fails.
 */
static bool bleNotifyFramed(uint8_t pktType, const uint8_t* payload, size_t payloadLen) {
  BLEDevice central = BLE.central();
  if (!central || !central.connected()) {
    return false;
  }
  if (payloadLen + kBleFrameHeaderBytes > static_cast<size_t>(abracadabraStreamChar.valueSize())) {
    return false;
  }
  uint8_t buf[kBleNotifyFrameMaxBytes];
  putU16Le(buf, kBleFrameMagic);
  buf[2] = pktType;
  buf[3] = 0;
  if (payloadLen > 0 && payload != nullptr) {
    memcpy(buf + kBleFrameHeaderBytes, payload, payloadLen);
  }
  const int n = static_cast<int>(kBleFrameHeaderBytes + payloadLen);
  blePollServicing();
  const bool ok = abracadabraStreamChar.writeValue(buf, n);
  blePollServicing();
  return ok;
}

/** Lets the central show “recording” in sync with the IMU window (capture follows immediately after notify). */
static bool bleNotifyRecordingPending(uint16_t windowId) {
  uint8_t payload[4];
  putU16Le(payload + 0, windowId);
  payload[2] = kBleProtoVer;
  payload[3] = 0;
  if (!bleNotifyFramed(kBlePktRecordingPending, payload, sizeof(payload))) {
    Serial.println(F("BLE: RECORDING_PENDING notify failed."));
    return false;
  }
  Serial.println(F("BLE: RECORDING_PENDING sent."));
  return true;
}

static bool bleNotifyRecordingChunks(uint16_t windowId, const uint8_t* packed, uint32_t totalBytes, uint32_t crc) {
  uint8_t payload[kBleChunkPayloadHeaderBytes + kBleChunkDataMaxBytes];
  uint32_t offset = 0;
  uint16_t chunkCount = 0;
  uint32_t nextLogBytes = 1024;

  while (offset < totalBytes) {
    const uint32_t remain = totalBytes - offset;
    const uint16_t n = static_cast<uint16_t>(
        remain < kBleChunkDataMaxBytes ? remain : kBleChunkDataMaxBytes);
    putU16Le(payload + 0, windowId);
    putU32Le(payload + 2, offset);
    putU16Le(payload + 6, n);
    memcpy(payload + kBleChunkPayloadHeaderBytes, packed + offset, n);
    if (!bleNotifyFramed(kBlePktChunk, payload, kBleChunkPayloadHeaderBytes + n)) {
      Serial.println(F("BLE: CHUNK notify failed."));
      return false;
    }
    offset += n;
    chunkCount++;
    if (offset >= nextLogBytes || offset == totalBytes) {
      Serial.print(F("BLE: CHUNK progress chunk="));
      Serial.print(chunkCount);
      Serial.print(F(" bytes="));
      Serial.print(offset);
      Serial.print(F("/"));
      Serial.println(totalBytes);
      nextLogBytes += 1024;
    }
    if (offset < totalBytes) {
      delay(kBleNotifyChunkPaceMs);
      blePollServicing();
    }
  }

  uint8_t commit[12];
  putU16Le(commit + 0, windowId);
  putU32Le(commit + 2, totalBytes);
  putU32Le(commit + 6, crc);
  commit[10] = kBleProtoVer;
  commit[11] = 0;
  for (uint8_t i = 0; i < kBleCommitRepeatCount; ++i) {
    if (!bleNotifyFramed(kBlePktCommit, commit, sizeof(commit))) {
      Serial.println(F("BLE: COMMIT notify failed."));
      return false;
    }
    if (i + 1 < kBleCommitRepeatCount) {
      delay(kBleNotifyChunkPaceMs);
      blePollServicing();
    }
  }

  Serial.print(F("BLE: notify chunks sent chunks="));
  Serial.print(chunkCount);
  Serial.print(F(" bytes="));
  Serial.println(totalBytes);
  return true;
}

/**
 * Notify META (includes CRC), then stream packed bytes as iOS-friendly CHUNK notifies.
 * ADAB0004/0005 pull remains staged as a diagnostic / compatibility fallback.
 */
static void bleTryPushRecording(uint16_t windowId, uint16_t sampleCount) {
  if (sampleCount == 0) {
    return;
  }

  sBlePullReady = false;
  sBlePullPackedPtr = nullptr;
  sBlePullTotalBytes = 0;

  BLEDevice central = BLE.central();
  if (!central || !central.connected()) {
    Serial.println(F("BLE: recording ready but no central — skipping transfer."));
    return;
  }

  uint8_t* packed = sBlePackedRecording;
  const uint32_t totalBytes =
      static_cast<uint32_t>(sampleCount) * static_cast<uint32_t>(kBlePackedSampleBytes);
  if (totalBytes > sizeof(sBlePackedRecording)) {
    Serial.println(F("BLE: packed recording overflow."));
    return;
  }

  for (uint16_t i = 0; i < sampleCount; ++i) {
    packImuSampleBleLittleEndian(sRecordBuf[i], packed + static_cast<size_t>(i) * kBlePackedSampleBytes);
  }

  const uint32_t crc = crc32Ieee(packed, totalBytes);

  Serial.print(F("BLE: META win="));
  Serial.print(windowId);
  Serial.print(F(" samples="));
  Serial.print(sampleCount);
  Serial.print(F(" totalBytes="));
  Serial.print(totalBytes);
  Serial.print(F(" crc=0x"));
  Serial.println(crc, HEX);

  uint8_t meta[16];
  putU16Le(meta + 0, windowId);
  putU16Le(meta + 2, sampleCount);
  putU32Le(meta + 4, totalBytes);
  meta[8] = kBleProtoVer;
  meta[9] = 0;
  meta[10] = 0;
  meta[11] = 0;
  putU32Le(meta + 12, crc);

  sBlePullPackedPtr = packed;
  sBlePullTotalBytes = totalBytes;

  if (!bleNotifyFramed(kBlePktMeta, meta, sizeof(meta))) {
    Serial.println(F("BLE: META notify failed."));
    sBlePullPackedPtr = nullptr;
    sBlePullTotalBytes = 0;
    return;
  }

  sBlePullReady = true;
  Serial.println(F("BLE: META sent — streaming payload by CHUNK notify (ADAB0003)."));
  if (!bleNotifyRecordingChunks(windowId, packed, totalBytes, crc)) {
    Serial.println(F("BLE: notify chunk stream failed — GATT pull fallback remains staged (ADAB0004/0005)."));
    return;
  }
  Serial.println(F("BLE: COMMIT sent — central should CRC/decode streamed payload."));
}

/** Flash green while recording (~4 Hz). */
static void recordingLedTick(uint32_t elapsedMs) {
  constexpr uint16_t kFlashHalfPeriodMs = 125;
  const bool on = ((elapsedMs / kFlashHalfPeriodMs) & 1U) == 0;
  if (on) {
    ledSet(0, 1, 0);
  } else {
    ledOff();
  }
}

/**
 * Captures fixed-rate raw IMU samples for kRecordDurationMs.
 * t_ms is nominal (sample index * period) for ML timeline alignment.
 */
static uint16_t captureImuRecordingWindow() {
  const uint32_t wallStart = millis();
  uint32_t nextDeadline = wallStart;
  uint16_t count = 0;

  while (count < kRecordMaxSamples) {
    const uint32_t now = millis();
    const uint32_t elapsed = now - wallStart;
    recordingLedTick(elapsed);

    if (static_cast<int32_t>(now - nextDeadline) < 0) {
      blePollServicing();
      continue;
    }

    if (elapsed >= kRecordDurationMs) {
      break;
    }

    blePollServicing();

    ImuRecordSample& s = sRecordBuf[count];
    s.t_ms = static_cast<uint16_t>(count * kRecordSamplePeriodMs);
    s.ax = imu.readRawAccelX();
    s.ay = imu.readRawAccelY();
    s.az = imu.readRawAccelZ();
    s.gx = imu.readRawGyroX();
    s.gy = imu.readRawGyroY();
    s.gz = imu.readRawGyroZ();
    count++;

    nextDeadline += kRecordSamplePeriodMs;
    if (static_cast<int32_t>(now - nextDeadline) > static_cast<int32_t>(kRecordSamplePeriodMs)) {
      nextDeadline = now;
    }
  }

  ledOff();
  return count;
}

/** Short animated cue: inward chase + dual sparkle + settle green. */
static void playDoubleTapCue() {
  constexpr uint16_t stepMs = 55;

  ledOff();
  delay(stepMs);
  blePollServicing();

  for (int rep = 0; rep < 2; rep++) {
    ledSet(1, 0, 0);
    delay(stepMs);
    blePollServicing();
    ledSet(1, 1, 0);
    delay(stepMs);
    blePollServicing();
    ledSet(0, 1, 0);
    delay(stepMs);
    blePollServicing();
    ledSet(0, 1, 1);
    delay(stepMs);
    blePollServicing();
    ledSet(0, 0, 1);
    delay(stepMs);
    blePollServicing();
    ledSet(1, 0, 1);
    delay(stepMs);
    blePollServicing();
    ledOff();
    delay(stepMs * 2);
    blePollServicing();
  }

  for (int i = 0; i < 6; i++) {
    const bool on = (i % 2) == 0;
    ledSet(on, on, on);
    delay(40);
    blePollServicing();
  }

  for (int e = 8; e >= 0; e--) {
    const bool pulse = e == 8 || e == 4 || e == 0;
    ledSet(pulse ? 0 : 0, pulse ? 1 : 0, pulse ? 0 : 0);
    delay(35);
    blePollServicing();
  }

  ledOff();
}

static void idleUntilDoubleTapInterrupt() {
  while (!imuIntPending) {
    blePollServicing();
    __WFI();
  }
}

void setup() {
  Serial.begin(115200);
  for (unsigned i = 0; i < 80 && !Serial; ++i) {
    delay(10);  // brief wait for USB Serial (skip if not plugged in)
  }

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  ledOff();

  pinMode(PIN_LSM6DS3TR_C_INT1, INPUT);

  if (!configureDoubleTapHardware()) {
    while (true) {
      ledSet(1, 0, 0);
      delay(150);
      ledOff();
      delay(150);
    }
  }

  attachImuInterrupt();

  if (!BLE.begin()) {
    Serial.println(F("BLE: begin() failed — scan name unavailable."));
  } else {
    // GATT Device Name — defaults to "Arduino" if unset (ArduinoBLE api.md).
    BLE.setDeviceName(kBleDeviceName);
    // Primary ADV must carry the name for many centrals (iOS / react-native-ble-plx): they often
    // never surface scan-response-only names before showing CBPeripheral.name ("Arduino").
    // A 128-bit UUID in ADV + full name does not fit in 31 B; we advertise name here and keep
    // ADAB… services on GATT only (discover after connect). See Peripheral examples:
    // https://github.com/arduino-libraries/ArduinoBLE/tree/master/examples/Peripheral
    BLEAdvertisingData adv;
    adv.setFlags(BLEFlagsGeneralDiscoverable | BLEFlagsBREDRNotSupported);
    adv.setLocalName(kBleDeviceName);
    BLE.setAdvertisingData(adv);
    BLE.setLocalName(kBleDeviceName);  // scan response — stacks that merge SR still get the name

    abracadabraService.addCharacteristic(abracadabraStatusChar);
    abracadabraService.addCharacteristic(abracadabraStreamChar);
    abracadabraService.addCharacteristic(abracadabraPullCtrlChar);
    abracadabraService.addCharacteristic(abracadabraPullDataChar);
    BLE.addService(abracadabraService);
    abracadabraStatusChar.writeValue(0);
    if (!BLE.advertise()) {
      Serial.println(F("BLE: advertise() failed."));
    }
    Serial.print(F("BLE: advertising as \""));
    Serial.print(kBleDeviceName);
    Serial.println(F("\"."));
  }

  ledSet(0, 1, 0);
  delay(120);
  ledOff();
}

void loop() {
  blePollServicing();

  idleUntilDoubleTapInterrupt();
  imuIntPending = false;

  detachImuInterrupt();

  uint8_t tapSrc = 0;
  const bool doubleTap = readWasDoubleTap(&tapSrc);

  if (doubleTap) {
    const bool acceptedCommand = serialPrintImuSnapshot(tapSrc);
    Serial.flush();
    if (acceptedCommand) {
      ++sRecordingWindowId;
      (void)bleNotifyRecordingPending(sRecordingWindowId);
      for (uint8_t i = 0; i < 8; ++i) {
        blePollServicing();
      }
      const uint16_t n = captureImuRecordingWindow();
      bleTryPushRecording(sRecordingWindowId, n);
      playDoubleTapCue();
      Serial.flush();
    }
  }

  attachImuInterrupt();
}
