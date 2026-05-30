#include <PRIZM.h>

PRIZM prizm;

const int LEFT_IR_PIN = A2;
const int FRONT_IR_PIN = A3;
const int IR_ANALOG_THRESHOLD = 250;
const bool IR_DETECTED_ABOVE_THRESHOLD = true;

bool isDetected(int raw)
{
  if (IR_DETECTED_ABOVE_THRESHOLD)
  {
    return raw >= IR_ANALOG_THRESHOLD;
  }
  return raw <= IR_ANALOG_THRESHOLD;
}

void setup()
{
  prizm.PrizmBegin();
  Serial.begin(115200);
}

void loop()
{
  int leftRaw = analogRead(LEFT_IR_PIN);
  int frontRaw = analogRead(FRONT_IR_PIN);

  Serial.print("left_A2_raw=");
  Serial.print(leftRaw);
  Serial.print(" left_detected=");
  Serial.print(isDetected(leftRaw) ? 1 : 0);

  Serial.print(" front_A3_raw=");
  Serial.print(frontRaw);
  Serial.print(" front_detected=");
  Serial.print(isDetected(frontRaw) ? 1 : 0);

  Serial.print(" threshold=");
  Serial.print(IR_ANALOG_THRESHOLD);
  Serial.print(" detected_above=");
  Serial.println(IR_DETECTED_ABOVE_THRESHOLD ? 1 : 0);

  delay(250);
}
