#include <WiFi.h>
#include <Wire.h>
#include <WiFiUdp.h>
#include <MPU6050_6Axis_MotionApps20.h>

// ==== Wi-Fi Config ====  
const char* ssid     = "MIPL";  
const char* password = "a1b2c3d4";

// ==== Pi UDP Config ====  
const char*    PI_IP           = "192.168.1.241";  
const uint16_t PI_IMU_PORT     = 8001;  // IMU → Pi  
const uint16_t PI_STATE_PORT   = 8002;  // Gun state → Pi  

WiFiUDP udp;  
MPU6050   mpu;

// ==== Pins ====  
const int TRIGGER_PIN   = 14;  
const int MAGAZINE_PIN  = 27;  
const int MODE_PIN      = 34;  
const int RELAY_PIN     = 25;  
const int SDA_PIN       = 21;  
const int SCL_PIN       = 22;

// ==== Fire Modes ====  
enum FireMode { SINGLE = 0, BURST = 1, AUTO = 2 };  
FireMode currentMode = SINGLE;

// ==== Gun State ====  
int  remainingBullets = 30;  
bool triggerPressed   = false;  
bool magazineInserted = false;  
bool canFire          = false;  
int  burstCount       = 0;

// ==== Constants ====  
const int MAGAZINE_CAPACITY = 30;  
const int BURST_SHOTS       = 3;  
const int FIRE_DELAY        = 85;   // ms between auto/burst shots  
const int SOLENOID_DURATION = 20;   // ms solenoid on  
const int DEBOUNCE_DELAY    = 50;   // ms

// ==== MPU6050 DMP ====  
bool     dmpReady      = false;  
uint8_t  devStatus;  
Quaternion   q;  
VectorFloat  gravity;  
float    ypr[3];  
uint8_t  fifoBuffer[64];

// ==== EMA for yaw ====  
float smoothedYaw = 0;  
const float alpha = 0.1;  // yaw EMA factor  

// ==== Complementary filter for pitch & roll ====  
float compPitch = 0, compRoll = 0;  
const float compAlpha = 0.98;  // trust gyro 98%  
uint32_t lastMicros;

// ───────────────────────────────────────────────────────────  
// Pack & UDP-send gun state to Pi  
// [byte0] flags: bit0=trigger, bit1=magazine  
// [byte1] remaining bullets  
// [byte2] fire mode (0=SINGLE,1=BURST,2=AUTO)  
// ───────────────────────────────────────────────────────────  
void sendGunStateUDP() {  
  uint8_t flags = (triggerPressed ? 1 : 0) | (magazineInserted ? 2 : 0);  
  uint8_t buf[3] = { flags, (uint8_t)remainingBullets, (uint8_t)currentMode };  
  udp.beginPacket(PI_IP, PI_STATE_PORT);  
  udp.write(buf, sizeof(buf));  
  udp.endPacket();  
}

// ───────────────────────────────────────────────────────────  
// Initialize MPU6050 + DMP  
// ───────────────────────────────────────────────────────────  
void setupMPU() {  
  Wire.begin(SDA_PIN, SCL_PIN);  
  mpu.initialize();  
  devStatus = mpu.dmpInitialize();  
  if (devStatus == 0) {  
    mpu.setDMPEnabled(true);  
    dmpReady = true;  
  }  
}

// ───────────────────────────────────────────────────────────  
// Fire one shot (solenoid)  
// ───────────────────────────────────────────────────────────  
void fire() {  
  if (!canFire || remainingBullets <= 0) {  
    sendGunStateUDP();  // empty-click  
    return;  
  }  
  remainingBullets--;  
  digitalWrite(RELAY_PIN, HIGH);  
  delay(SOLENOID_DURATION);  
  digitalWrite(RELAY_PIN, LOW);  
  sendGunStateUDP();  
}

// ───────────────────────────────────────────────────────────  
// Handle trigger logic per mode  
// ───────────────────────────────────────────────────────────  
void handleTriggerPress() {  
  if (!canFire) return;  
  switch (currentMode) {  
    case SINGLE:  
      fire();  
      break;  
    case BURST:  
      burstCount = min(BURST_SHOTS, remainingBullets);  
      fire();  
      burstCount--;  
      break;  
    case AUTO:  
      fire();  
      break;  
  }  
}

// ───────────────────────────────────────────────────────────  
// Read potentiometer & update mode  
// ───────────────────────────────────────────────────────────  
void updateFireMode() {  
  int v = analogRead(MODE_PIN);  
  FireMode m = (v < 1365 ? SINGLE : (v < 2730 ? BURST : AUTO));  
  if (m != currentMode) {  
    currentMode = m;  
    sendGunStateUDP();  
  }  
}

// ───────────────────────────────────────────────────────────  
// Detect magazine insert/remove  
// ───────────────────────────────────────────────────────────  
void handleMagazine() {  
  bool ins = (digitalRead(MAGAZINE_PIN) == LOW);  
  if (ins != magazineInserted) {  
    magazineInserted = ins;  
    canFire = ins;  
    if (ins) remainingBullets = MAGAZINE_CAPACITY;  
    sendGunStateUDP();  
  }  
}

// ───────────────────────────────────────────────────────────  
// Arduino setup()  
// ───────────────────────────────────────────────────────────  
void setup() {  
  Serial.begin(115200);  
  pinMode(TRIGGER_PIN,  INPUT_PULLUP);  
  pinMode(MAGAZINE_PIN, INPUT_PULLUP);  
  pinMode(MODE_PIN,     INPUT);  
  pinMode(RELAY_PIN,    OUTPUT);  
  digitalWrite(RELAY_PIN, LOW);  

  setupMPU();  

  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED) delay(500);  
  udp.begin(PI_STATE_PORT);  

  // initial state & timestamp  
  sendGunStateUDP();  
  lastMicros = micros();  
}

// ───────────────────────────────────────────────────────────  
// Arduino loop()  
// ───────────────────────────────────────────────────────────  
void loop() {  
  // 1) Gun controls  
  updateFireMode();  
  handleMagazine();  

  // Trigger debounce  
  static int  lastTrigger    = HIGH;  
  static unsigned long lastDebounce = 0;  
  int t = digitalRead(TRIGGER_PIN);  
  if (t != lastTrigger) lastDebounce = millis();  
  if ((millis() - lastDebounce) > DEBOUNCE_DELAY) {  
    if (t == LOW && !triggerPressed) {  
      triggerPressed = true;  
      handleTriggerPress();  
    }  
    else if (t == HIGH && triggerPressed) {  
      triggerPressed = false;  
      burstCount = 0;  
      sendGunStateUDP();  
    }  
  }  
  lastTrigger = t;  

  // Burst‐mode loop  
  if (currentMode == BURST && burstCount > 0 && canFire) {  
    fire();  
    burstCount--;  
    delay(FIRE_DELAY);  
  }  
  // Auto‐mode loop  
  if (currentMode == AUTO && triggerPressed && canFire) {  
    fire();  
    delay(FIRE_DELAY);  
  }  

  // 2) IMU → UDP with complementary filter  
  if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  
    // — DMP yaw for EMA —  
    mpu.dmpGetQuaternion(&q, fifoBuffer);  
    mpu.dmpGetGravity(&gravity, &q);  
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);  
    float rawYaw = ypr[0] * 180.0f / M_PI;  
    smoothedYaw = alpha * rawYaw + (1 - alpha) * smoothedYaw;  

    // — Raw accel & gyro for comp filter —  
    int16_t ax, ay, az, gx, gy, gz;  
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  

    uint32_t now = micros();  
    float dt = (now - lastMicros) * 1e-6f;  
    lastMicros = now;  

    // accel → g’s  
    float accelX = ax / 16384.0f;  
    float accelY = ay / 16384.0f;  
    float accelZ = az / 16384.0f;  

    // gyro → °/s  
    float gyroX = gx / 131.0f;  
    float gyroY = gy / 131.0f;  

    // tilt from accel  
    float rollAcc  = atan2(accelY, accelZ) * 180.0f / M_PI;  
    float pitchAcc = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 180.0f / M_PI;  

    // complementary fusion  
    compRoll  = compAlpha * (compRoll  + gyroX * dt) + (1 - compAlpha) * rollAcc;  
    compPitch = compAlpha * (compPitch + gyroY * dt) + (1 - compAlpha) * pitchAcc;  

    // send filtered data (pitch, roll, yaw)  
    uint8_t buf[12];  
    memcpy(buf + 0, &compPitch, sizeof(float));  
    memcpy(buf + 4, &compRoll,  sizeof(float));  
    memcpy(buf + 8, &smoothedYaw, sizeof(float));  
    udp.beginPacket(PI_IP, PI_IMU_PORT);  
    udp.write(buf, sizeof(buf));  
    udp.endPacket();  
  }  

  delay(2);  
}