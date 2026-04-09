#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <LoRa.h>

// MPU + PID variables
MPU6050 mpu;

int16_t rawAx, rawAy, rawAz;
int16_t gyroX, gyroY, gyroZ;

float ax, ay, az;
float pitch;
float filteredPitch = 0;
float yawAngle = 0;

float Kp = 1.2, Ki = 0.03, Kd = 0.4;
float errorTilt = 0, previousErrorTilt = 0, integralTilt = 0, outputTilt = 0;

const float smoothingFactor = 0.1;
const float sensitivityFactor = 3.0;
const int center = 90;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 not connected!");
    while (1);
  }

  mpu.CalibrateAccel();
  mpu.CalibrateGyro();

  LoRa.setPins(10, 9, 2); // NSS, RESET, DIO0
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    while (1);
  }

  Serial.println("LoRa TX Ready");
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  mpu.getAcceleration(&rawAx, &rawAy, &rawAz);
  mpu.getRotation(&gyroX, &gyroY, &gyroZ);

  ax = (float)rawAx;
  ay = (float)rawAy;
  az = (float)rawAz;

  // Calculate pitch
  pitch = atan2(ay, az) * 180.0 / PI;
  filteredPitch = filteredPitch * (1.0 - smoothingFactor) + pitch * smoothingFactor;
  float adjustedPitch = filteredPitch * sensitivityFactor;

  // PID for Tilt
  errorTilt = adjustedPitch;
  integralTilt += errorTilt * dt;
  integralTilt = constrain(integralTilt, -100, 100);
  float derivativeTilt = (errorTilt - previousErrorTilt) / dt;
  outputTilt = Kp * errorTilt + Ki * integralTilt + Kd * derivativeTilt;
  outputTilt = constrain(outputTilt, -90, 90);
  previousErrorTilt = errorTilt;

  // Yaw from gyro Z
  float gyroZ_deg = (float)gyroZ / 131.0;
  yawAngle += gyroZ_deg * dt;
  yawAngle = constrain(yawAngle, -90, 90);

  // Send outputTilt and yawAngle via LoRa
  LoRa.beginPacket();
  LoRa.print(outputTilt);
  LoRa.print(",");
  LoRa.print(yawAngle);
  LoRa.endPacket();

  // Debug
  Serial.print("Sent Tilt: "); Serial.print(outputTilt);
  Serial.print(" | Yaw: "); Serial.println(yawAngle);

  delay(20);  // 50Hz
}
