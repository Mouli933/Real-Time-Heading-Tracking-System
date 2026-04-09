#include <Servo.h>
#include <SPI.h>
#include <LoRa.h>

Servo tiltServo, panServo;

const int panPin = 7;
const int tiltPin = 8;
const int center = 90;

void setup() {
  Serial.begin(115200);

  // LoRa pins: NSS = 10, RESET = 9, DIO0 = 2
  LoRa.setPins(10, 9, 2);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    while (1);
  }
  Serial.println("LoRa RX Ready");

  // Attach servos
  panServo.attach(panPin);
  tiltServo.attach(tiltPin);

  // Initialize to center
  panServo.write(center);
  tiltServo.write(center);
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String data = "";
    while (LoRa.available()) {
      data += (char)LoRa.read();
    }

    float receivedTilt = 0;
    float receivedYaw = 0;

    // Parse data
    int commaIndex = data.indexOf(',');
    if (commaIndex != -1) {
      receivedTilt = data.substring(0, commaIndex).toFloat();
      receivedYaw  = data.substring(commaIndex + 1).toFloat();

      // Convert to servo angles
      int targetTilt = constrain(center + receivedTilt, 0, 180);
      int targetPan  = constrain(center + receivedYaw, 0, 180);

      // Reverse pan direction
      panServo.write(180 - targetPan);  // ✅ Reversed pan
      tiltServo.write(targetTilt);      // Normal tilt

      // Debug
      Serial.print("Tilt PWM: "); Serial.print(targetTilt);
      Serial.print(" | Pan PWM (reversed): "); Serial.println(180 - targetPan);
    }
  }

  delay(5);  // Fast refresh loop
}