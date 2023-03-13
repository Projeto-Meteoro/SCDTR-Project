#include "pid.h"
const int LED_PIN = 15;
//Create a pid controller
pid my_pid {0.001, 1, 0.1, 0.05};

float r {200};
void setup() {
  analogReadResolution(12);
  Serial.begin(9600);
}

void loop() {
  if ( Serial.available() )
    r = Serial.parseInt(SKIP_ALL, '\n');
  float y = analogRead(A0);
  Serial.print("y:");
  Serial.print(y);
  float u = my_pid.compute_control(r, y);
  int pwm = (int)u;
  Serial.print(",pwm:");
  Serial.print(pwm);
  Serial.print(",reference:");
  Serial.println(r);
  analogWrite(LED_PIN, pwm);
  delay(50);
}