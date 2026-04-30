#include <Arduino.h>
#include <LSM6DS3.h>
#include <math.h>

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

/** Short animated cue: inward chase + dual sparkle + settle green. */
static void playDoubleTapCue() {
  constexpr uint16_t stepMs = 55;

  ledOff();
  delay(stepMs);

  for (int rep = 0; rep < 2; rep++) {
    ledSet(1, 0, 0);
    delay(stepMs);
    ledSet(1, 1, 0);
    delay(stepMs);
    ledSet(0, 1, 0);
    delay(stepMs);
    ledSet(0, 1, 1);
    delay(stepMs);
    ledSet(0, 0, 1);
    delay(stepMs);
    ledSet(1, 0, 1);
    delay(stepMs);
    ledOff();
    delay(stepMs * 2);
  }

  for (int i = 0; i < 6; i++) {
    const bool on = (i % 2) == 0;
    ledSet(on, on, on);
    delay(40);
  }

  for (int e = 8; e >= 0; e--) {
    const bool pulse = e == 8 || e == 4 || e == 0;
    ledSet(pulse ? 0 : 0, pulse ? 1 : 0, pulse ? 0 : 0);
    delay(35);
  }

  ledOff();
}

static void idleUntilDoubleTapInterrupt() {
  while (!imuIntPending) {
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

  ledSet(0, 1, 0);
  delay(120);
  ledOff();
}

void loop() {
  idleUntilDoubleTapInterrupt();
  imuIntPending = false;

  detachImuInterrupt();

  uint8_t tapSrc = 0;
  const bool doubleTap = readWasDoubleTap(&tapSrc);

  if (doubleTap) {
    const bool acceptedCommand = serialPrintImuSnapshot(tapSrc);
    Serial.flush();
    if (acceptedCommand) {
      playDoubleTapCue();
    }
  }

  attachImuInterrupt();
}
