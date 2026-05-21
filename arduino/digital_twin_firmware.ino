// 6-DOF Digital Twin - Arduino Firmware
// Controls 6 servos via PCA9685 I2C driver
// Communicates with ROS 2 bridge via USB serial
//
// Commands:
//   "90 100 50 120 120 100\n" - Move all 6 servos (from bridge)
//   "0 90\n"                  - Move single servo CH0 to 90 degrees
//   "h\n"                     - Home all servos (smooth sequential)
//
// CUSTOMIZE: Change homeAngles[], servoMin[], servoMax[] for your robot

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN 150
#define SERVOMAX 600
#define NUM_SERVOS 6
#define SWEEP_DELAY 15

// CUSTOMIZE THESE FOR YOUR ROBOT:
int servoMin[NUM_SERVOS] = {10, 55, 55, 80, 10, 100};
int servoMax[NUM_SERVOS] = {170, 150, 180, 160, 170, 140};
int homeAngles[NUM_SERVOS] = {90, 100, 50, 120, 120, 100};

int currentAngles[NUM_SERVOS];
String inputString = "";

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void smoothMove(int channel, int targetAngle) {
  targetAngle = constrain(targetAngle, servoMin[channel], servoMax[channel]);
  int current = currentAngles[channel];
  if (current == targetAngle) return;
  int step = (targetAngle > current) ? 1 : -1;
  for (int angle = current; angle != targetAngle; angle += step) {
    pca9685.setPWM(channel, 0, angleToPulse(angle));
    delay(SWEEP_DELAY);
  }
  pca9685.setPWM(channel, 0, angleToPulse(targetAngle));
  currentAngles[channel] = targetAngle;
}

void setServoAngle(uint8_t channel, int angle) {
  angle = constrain(angle, servoMin[channel], servoMax[channel]);
  pca9685.setPWM(channel, 0, angleToPulse(angle));
  currentAngles[channel] = angle;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pca9685.begin();
  pca9685.setPWMFreq(50);
  delay(100);

  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngles[i] = homeAngles[i];
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    smoothMove(i, homeAngles[i]);
    delay(500);
  }

  Serial.println("6DOF_READY");
  Serial.print("HOME");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print(" ");
    Serial.print(homeAngles[i]);
  }
  Serial.println();
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputString.length() > 0) {

        if (inputString == "h") {
          for (int i = 0; i < NUM_SERVOS; i++) {
            smoothMove(i, homeAngles[i]);
            delay(300);
          }
          Serial.print("HOME");
          for (int i = 0; i < NUM_SERVOS; i++) {
            Serial.print(" ");
            Serial.print(currentAngles[i]);
          }
          Serial.println();
          inputString = "";
          return;
        }

        int spaceCount = 0;
        for (int i = 0; i < (int)inputString.length(); i++) {
          if (inputString.charAt(i) == ' ') spaceCount++;
        }

        if (spaceCount == 1) {
          int spaceIdx = inputString.indexOf(' ');
          int channel = inputString.substring(0, spaceIdx).toInt();
          int angle = inputString.substring(spaceIdx + 1).toInt();
          if (channel >= 0 && channel < NUM_SERVOS) {
            angle = constrain(angle, servoMin[channel], servoMax[channel]);
            setServoAngle(channel, angle);
            Serial.print("Moved CH");
            Serial.print(channel);
            Serial.print(" to ");
            Serial.print(angle);
            Serial.println(" degrees");
          }
          inputString = "";
          return;
        }

        if (spaceCount == 5) {
          int angles[NUM_SERVOS];
          int index = 0;
          int startPos = 0;
          for (int i = 0; i <= (int)inputString.length() && index < NUM_SERVOS; i++) {
            if (i == (int)inputString.length() || inputString.charAt(i) == ' ') {
              if (i > startPos) {
                angles[index] = inputString.substring(startPos, i).toInt();
                angles[index] = constrain(angles[index], servoMin[index], servoMax[index]);
                index++;
              }
              startPos = i + 1;
            }
          }
          if (index == NUM_SERVOS) {
            for (int i = 0; i < NUM_SERVOS; i++) {
              setServoAngle(i, angles[i]);
            }
            Serial.print("OK");
            for (int i = 0; i < NUM_SERVOS; i++) {
              Serial.print(" ");
              Serial.print(currentAngles[i]);
            }
            Serial.println();
          }
          inputString = "";
          return;
        }

        inputString = "";
      }
    } else {
      inputString += c;
    }
  }
}
