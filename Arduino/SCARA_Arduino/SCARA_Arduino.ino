#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// -------------------------------------------------------------
//  Servo pulse ranges (calibrate these to your servos!)
// -------------------------------------------------------------
const int SHOULDER_MIN = 85;   // Shoulder (0°)
const int SHOULDER_MAX = 655;  // Shoulder (270°)
const int ELBOW_MIN    = 150;  // Elbow (0°)
const int ELBOW_MAX    = 600;  // Elbow (270°)

// -------------------------------------------------------------
//  Gripper
// -------------------------------------------------------------
const int GRIPPER_PIN = 9;     // Digital pin for gripper

// -------------------------------------------------------------
//  Lead Screw (MG996R - Continuous Rotation)
//  LEAD_STOP = 300 sends a true 1.5ms neutral pulse to stop
// -------------------------------------------------------------
const int LEAD_STOP     = 300; // Neutral (stop)
const int LEAD_UP_MAX   = 300; // Faster UP   (adjust direction if reversed)
const int LEAD_DOWN_MAX = 600; // Faster DOWN

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);          // Analog servos run at ~60 Hz

  pinMode(GRIPPER_PIN, OUTPUT);
  digitalWrite(GRIPPER_PIN, LOW); // Start with gripper open
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int spaceIndex = input.indexOf(' ');
    if (spaceIndex == -1) return; // Skip invalid commands

    int servoIndex = input.substring(0, spaceIndex).toInt();
    int servoValue = input.substring(spaceIndex + 1).toInt();

    switch (servoIndex) {

      case 1: // Shoulder (0–270°)
        pwm.setPWM(0, 0, map(servoValue, 0, 270, SHOULDER_MIN, SHOULDER_MAX));
        break;

      case 2: // Elbow (0–270°)
        pwm.setPWM(1, 0, map(servoValue, 0, 270, ELBOW_MIN, ELBOW_MAX));
        break;

      case 3: // Lead Screw (UP / STOP / DOWN)
        if (servoValue == 0) {
          pwm.setPWM(2, 0, LEAD_STOP);     // STOP
        } else if (servoValue == 1) {
          pwm.setPWM(2, 0, LEAD_UP_MAX);   // UP
        } else if (servoValue == -1) {
          pwm.setPWM(2, 0, LEAD_DOWN_MAX); // DOWN
        }
        break;

      case 4: // Gripper
        digitalWrite(GRIPPER_PIN, servoValue == 1 ? HIGH : LOW);
        break;
    }

    // Debug output
    Serial.print("Servo ");
    Serial.print(servoIndex);
    Serial.print(" set to ");
    Serial.println(servoValue);
  }
}
