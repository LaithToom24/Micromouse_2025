#include <digitalWriteFast.h>
#include <Arduino.h>

int left_dist = 0;
int right_dist = 0;
int front_dist = 0;

int readSensor(int channel)
{
  // Measure ambient light first (emitter off)
  digitalWriteFast(12, LOW);
  delayMicroseconds(100);
  int dark = analogRead(channel);

  // Turn emitter on and measure reflected light
  digitalWriteFast(12, HIGH);
  delayMicroseconds(150);  // allow IR LED to stabilize
  int lit = analogRead(channel);

  // Turn emitter off
  digitalWriteFast(12, LOW);

  // Compute reflection difference (lit - dark)
  int diff = lit - dark;
  if (diff < 0) diff = 0;
  if (diff > 1023) diff = 1023;

  return diff;
}

void readAll(){
  left_dist = readSensor(A2);

  delayMicroseconds(200);

  right_dist = readSensor(A1);

  delayMicroseconds(200);

  front_dist = readSensor(A0);

  delayMicroseconds(200);
}

void printAll(){
  Serial.print(left_dist);
  Serial.print(" ");
  Serial.print(right_dist);
  Serial.print(" ");
  Serial.println(front_dist);
}