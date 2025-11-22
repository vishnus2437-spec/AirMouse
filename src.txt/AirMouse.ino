/* Final AirMouse for MPU6050 clone (WHO_AM_I = 0x70)
   - Raw I2C reads (no Adafruit library)
   - Smooth gyro filtering
   - Double-click, drag, shake-to-recenter
   - Works on ALL MPU clones
*/

#include <Arduino.h>
#include <Wire.h>
#include <BleMouse.h>

// Pins
#define LEFTBUTTON   19
#define RIGHTBUTTON  18
#define RESETBUTTON  15
#define LEDPIN        2

// Motion settings
#define SPEED        18.0f
#define DEADZONE     0.03f
#define MAX_DELTA    127

// Timing
#define DOUBLE_CLICK_MS 250
#define DRAG_HOLD_MS    400

// Shake
#define SHAKE_DELTA_THRESHOLD  4.0f
#define SHAKE_COOLDOWN_MS      600
#define SHAKE_AUTO_RESUME_MS   500

// MPU registers
#define MPU_ADDR        0x68
#define REG_PWR_MGMT1   0x6B
#define REG_WHOAMI      0x75
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define REG_SMPLRT_DIV  0x19
#define REG_ACCEL_XOUT_H 0x3B

// scales
const float GYRO_LSB_PER_DEG = 65.5f; // ±500 dps
const float DEG2RAD = 3.14159265358979323846f / 180.0f; // FIXED NAME

// smoothing
const float GYRO_ALPHA = 0.75f;

BleMouse bleMouse("AirMouse");

// State trackers
bool paused = false;
bool dragging = false;

unsigned long leftPressTime = 0;
unsigned long rightPressTime = 0;

bool leftPendingClick = false;
unsigned long leftPendingTime = 0;

bool rightPendingClick = false;
unsigned long rightPendingTime = 0;

unsigned long lastShake = 0;

// smoothed gyro values
float smooth_gx = 0, smooth_gy = 0, smooth_gz = 0;

static inline int8_t clamp8(float v) {
  if (v >  MAX_DELTA) return  MAX_DELTA;
  if (v < -MAX_DELTA) return -MAX_DELTA;
  return (int8_t)lroundf(v);
}

// ---------------- LOW-LEVEL I2C ----------------
void mpuWriteByte(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

uint8_t mpuReadByte(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);   // FIXED
  if (Wire.available()) return Wire.read();
  return 0;
}

int mpuReadBytes(uint8_t startReg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false);
  uint8_t got = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)len); // FIXED
  for (uint8_t i = 0; i < got; i++) buf[i] = Wire.read();
  return got;
}

// ---------------- MPU SETUP ----------------
void setupMPUClone() {
  mpuWriteByte(REG_PWR_MGMT1, 0x00);  
  delay(50);
  mpuWriteByte(REG_SMPLRT_DIV, 0x07);
  mpuWriteByte(REG_CONFIG, 0x03);       
  mpuWriteByte(REG_GYRO_CFG, 0x08);     
  mpuWriteByte(REG_ACCEL_CFG, 0x08);    
  delay(50);
}

// ------------- READ RAW ACC + GYRO -------------
void readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint8_t data[14];
  if (mpuReadBytes(REG_ACCEL_XOUT_H, data, 14) < 14) return;

  int16_t rawAX = (int16_t)((data[0] << 8) | data[1]);
  int16_t rawAY = (int16_t)((data[2] << 8) | data[3]);
  int16_t rawAZ = (int16_t)((data[4] << 8) | data[5]);

  int16_t rawGX = (int16_t)((data[8]  << 8) | data[9]);
  int16_t rawGY = (int16_t)((data[10] << 8) | data[11]);
  int16_t rawGZ = (int16_t)((data[12] << 8) | data[13]);

  // accel ±4g → 8192 LSB/g
  const float ACC_SCALE = 8192.0f;
  ax = (rawAX / ACC_SCALE) * 9.81f;
  ay = (rawAY / ACC_SCALE) * 9.81f;
  az = (rawAZ / ACC_SCALE) * 9.81f;

  gx = (rawGX / GYRO_LSB_PER_DEG) * DEG2RAD;  
  gy = (rawGY / GYRO_LSB_PER_DEG) * DEG2RAD;  
  gz = (rawGZ / GYRO_LSB_PER_DEG) * DEG2RAD;  
}

// ------------- PENDING SINGLE CLICK HANDLER -------------
void handlePendingClicks() {
  unsigned long now = millis();

  if (leftPendingClick && (now - leftPendingTime >= DOUBLE_CLICK_MS)) {
    bleMouse.click(MOUSE_LEFT);
    Serial.println("Left Click");
    leftPendingClick = false;
  }
  if (rightPendingClick && (now - rightPendingTime >= DOUBLE_CLICK_MS)) {
    bleMouse.click(MOUSE_RIGHT);
    Serial.println("Right Click");
    rightPendingClick = false;
  }
}

/////////////////////// SETUP ///////////////////////
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(LEFTBUTTON, INPUT_PULLUP);
  pinMode(RIGHTBUTTON, INPUT_PULLUP);
  pinMode(RESETBUTTON, INPUT_PULLUP);
  pinMode(LEDPIN, OUTPUT);

  Wire.begin(21, 22);
  delay(50);

  Serial.println("Configuring MPU clone...");
  setupMPUClone();

  uint8_t who = mpuReadByte(REG_WHOAMI);
  Serial.print("WHO_AM_I = 0x");
  Serial.println(who, HEX);

  bleMouse.begin();
  Serial.println("BLE Ready");
}

/////////////////////// LOOP ///////////////////////
void loop() {
  if (!bleMouse.isConnected()) {
    digitalWrite(LEDPIN, LOW);
    delay(80);
    return;
  }
  digitalWrite(LEDPIN, HIGH);

  // Pause toggle
  if (digitalRead(RESETBUTTON) == LOW) {
    paused = !paused;
    Serial.println(paused ? "Paused" : "Resumed");
    delay(300);
  }

  if (paused) {
    handlePendingClicks();
    return;
  }

  // Read sensor
  float ax, ay, az, gx, gy, gz;
  readMPU(ax, ay, az, gx, gy, gz);

  // Smooth gyro
  smooth_gx = GYRO_ALPHA * smooth_gx + (1 - GYRO_ALPHA) * gx;
  smooth_gy = GYRO_ALPHA * smooth_gy + (1 - GYRO_ALPHA) * gy;
  smooth_gz = GYRO_ALPHA * smooth_gz + (1 - GYRO_ALPHA) * gz;

  // Map gyro → mouse
  float dx = -smooth_gz;
  float dy = -smooth_gx;

  if (fabs(dx) < DEADZONE) dx = 0;
  if (fabs(dy) < DEADZONE) dy = 0;

  int8_t mdx = clamp8(dx * SPEED);
  int8_t mdy = clamp8(dy * SPEED);

  if (mdx || mdy) {
    bleMouse.move(mdx, mdy);
    digitalWrite(LEDPIN, LOW); delay(2); digitalWrite(LEDPIN, HIGH);
  }

  unsigned long now = millis();

  // ---- SHAKE ----
  float mag = sqrt(ax*ax + ay*ay + az*az);
  float deltaG = fabs(mag - 9.81f);

  if (deltaG > SHAKE_DELTA_THRESHOLD && (now - lastShake) > SHAKE_COOLDOWN_MS) {
    Serial.println("Shake detected → recenter");
    paused = true;
    lastShake = now;
    digitalWrite(LEDPIN, LOW); delay(100); digitalWrite(LEDPIN, HIGH);
  }

  if (paused && (now - lastShake > SHAKE_AUTO_RESUME_MS)) {
    paused = false;
    Serial.println("Auto-resume");
  }

  // ---- LEFT BUTTON (drag, double-click) ----
  static bool leftDown = false;
  bool leftNow = (digitalRead(LEFTBUTTON) == LOW);

  if (leftNow && !leftDown) {
    leftPressTime = now;
    leftDown = true;
  }

  if (!leftNow && leftDown) {
    unsigned long pressDur = now - leftPressTime;

    if (dragging) {
      bleMouse.release(MOUSE_LEFT);
      dragging = false;
      Serial.println("Drag end");
    }
    else if (pressDur > DRAG_HOLD_MS) {
      Serial.println("Long press but no drag");
    }
    else {
      if (leftPendingClick) {
        bleMouse.click(MOUSE_LEFT);
        delay(70);
        bleMouse.click(MOUSE_LEFT);
        Serial.println("Double Left Click");
        leftPendingClick = false;
      } else {
        leftPendingClick = true;
        leftPendingTime = now;
      }
    }
    leftDown = false;
  }

  if (leftDown && !dragging && (now - leftPressTime > DRAG_HOLD_MS)) {
    bleMouse.press(MOUSE_LEFT);
    dragging = true;
    leftPendingClick = false;
    Serial.println("Drag Start");
  }

  // ---- RIGHT BUTTON ----
  static bool rightDown = false;
  bool rightNow = (digitalRead(RIGHTBUTTON) == LOW);

  if (rightNow && !rightDown) {
    rightPressTime = now;
    rightDown = true;
  }

  if (!rightNow && rightDown) {
    unsigned long pressDur = now - rightPressTime;
    if (pressDur > DRAG_HOLD_MS) {
      // reserved for long-press actions
    } else {
      if (rightPendingClick) {
        bleMouse.click(MOUSE_RIGHT);
        delay(70);
        bleMouse.click(MOUSE_RIGHT);
        Serial.println("Double Right Click");
        rightPendingClick = false;
      } else {
        rightPendingClick = true;
        rightPendingTime = now;
      }
    }
    rightDown = false;
  }

  handlePendingClicks();
  delay(5);
}
