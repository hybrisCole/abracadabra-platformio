#include <Arduino.h>
#include <LSM6DS3.h>

// RGB LEDs — active LOW on Seeed XIAO BLE Sense (mbed core)
#define LED_R LEDR
#define LED_G LEDG
#define LED_B LEDB

#ifndef PIN_LSM6DS3TR_C_INT1
#error This sketch expects Seeed XIAO nRF52840 Sense (LSM6DS3 INT1 routed to D17).
#endif

LSM6DS3 imu;

static volatile bool imuIntPending;

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

/** ST AN5130 §5.5.5 — double-tap to INT1; gyro off for lower current. */
static bool configureDoubleTapHardware() {
  if (imu.beginCore() != IMU_SUCCESS) {
    return false;
  }

  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00);  // gyro power-down

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

/** Confirms hardware recognized a double tap (not only first tap of pair). */
static bool readWasDoubleTap() {
  uint8_t tapSrc = 0;
  imu.readRegister(&tapSrc, LSM6DS3_ACC_GYRO_TAP_SRC);
  return (tapSrc & 0x10) != 0;  // DOUBLE_TAP bit in TAP_SRC
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

  const bool doubleTap = readWasDoubleTap();

  if (doubleTap) {
    playDoubleTapCue();
  }

  attachImuInterrupt();
}
