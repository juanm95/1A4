#include <Pulse.h>

/*
 * This program waits for a key input and then responds by performing a full motor rotation. 
 * Change the period and pulses to adjust the degree of rotation and the speed of rotation.
 */
int PERIOD = 50; 
int PULSES = 200;
//1ms
void setup() {
  pinMode(3, OUTPUT);
  Serial.begin(9600);
  InitPulse(3, PERIOD);
}

unsigned char TestForKey();
void RespToKey();

bool forward = true;

void loop() {
 if (TestForKey() && IsPulseFinished()) RespToKey();
}

unsigned char TestForKey(void) {
  unsigned char KeyEventOccurred;
  KeyEventOccurred = Serial.available();
  return KeyEventOccurred;
}

void RespToKey(void) {
  Serial.read();
  if (forward) {
    Serial.println("Setting LOW");
    digitalWrite(3, LOW);
    Pulse(PULSES);
  } else {
    Serial.println("Setting HIGH");
    digitalWrite(4, HIGH);
    forward = true;
  }
}
