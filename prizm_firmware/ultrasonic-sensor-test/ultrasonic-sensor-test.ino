#include <PRIZM.h>

PRIZM prizm;

const int RIGHT_FORWARD_ULTRASONIC_PIN = 3;
const int CLAW_ULTRASONIC_PIN = 4;
const int LEFT_FORWARD_ULTRASONIC_PIN = 5;

void setup()
{
  prizm.PrizmBegin();
  Serial.begin(115200);
}

void loop()
{
  int rightCm = prizm.readSonicSensorCM(RIGHT_FORWARD_ULTRASONIC_PIN);
  delay(60);
  int leftCm = prizm.readSonicSensorCM(LEFT_FORWARD_ULTRASONIC_PIN);
  delay(60);
  int clawCm = prizm.readSonicSensorCM(CLAW_ULTRASONIC_PIN);

  Serial.print("right_forward_D3_cm=");
  Serial.print(rightCm);
  Serial.print(" left_forward_D5_cm=");
  Serial.print(leftCm);
  Serial.print(" claw_D4_cm=");
  Serial.println(clawCm);

  delay(250);
}
