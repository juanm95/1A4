#include <SM.h>
#include <State.h>

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
  digitalWrite(4, HIGH);
  digitalWrite(3, LOW);
}

unsigned char TestForKey();
void RespToKey();
void ReverseDirection();
void FullRotation();
void DumpChips();
void OneChip();
bool forward = true;
int chips = 12;
void loop() {
 if (TestForKey() && IsPulseFinished()) RespToKey();
}

unsigned char TestForKey(void) {
  unsigned char KeyEventOccurred;
  KeyEventOccurred = Serial.available();
  return KeyEventOccurred;
}

void RespToKey(void) {
  char option = Serial.read();
  switch(option) {
    case ('r'): ReverseDirection(); break;
    case ('f'): FullRotation(); break;
    case ('d'): DumpChips(); break;
    case ('o'): OneChip(); break;
  }
}

void OneChip() {
  chips--;
  Pulse(PULSES/12);
}

void FullRotation() {
  chips = 0;
  Pulse(PULSES);
}
void DumpChips() {
  Pulse(PULSES * chips /12);
  chips = 12;
}

void ReverseDirection() {
  forward = !forward;
  if (forward) digitalWrite(4, HIGH);
  else digitalWrite(4, LOW);
}


