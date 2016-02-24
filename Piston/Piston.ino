#include <Timers.h>

#include <SM.h>
#include <State.h>

#include <Pulse.h>
State Overall_Start();
State Overall_FindRightBinIR();
State Overall_ApproachBinLine();
State Overall_DriveBy();
State Overall_ApproachTheForce();
State Overall_LoadForce();
State Overall_DetermineDumpSide();
State Overall_DumpRightSide();
State Overall_DumpLeftSide();
State DumpRight_ApproachingBins();
State DumpRight_DepositChip();
State DumpRight_ApproachingNextBin();
State DumpRight_DumpAll();
State DumpRight_ReturnToCenter();
SM Overall(Overall_Start);
SM DumpRight(DumpRight_ApproachingBins);
/*
 * This program waits for a key input and then responds by performing a full motor rotation. 
 * Change the period and pulses to adjust the degree of rotation and the speed of rotation.
 */
int PERIOD = 50; 
int PULSES = 200;
int TIME_TO_NEXT_BIN = 3000;// ms
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
  EXEC(Overall);
}
/************************* Overall States ***********************************/
State Overall_Start() {
  Overall.Set(Overall_DetermineDumpSide);
}
bool DeterminedDumpSide = true;
bool ShouldDumpRight = true;
bool atCenter = true;
State Overall_DetermineDumpSide() {
  if (DeterminedDumpSide) {
    if (ShouldDumpRight) {
      atCenter = false;
      Overall.Set(Overall_DumpRight);
    }
  }  
}
State Overall_DumpRight() {
  if (atCenter) {
    
  } else {
    Overall.Set(Overall_DumpRight);
  }
  EXEC(DumpRight);
}

/************************ DumpRight States *********************************/

unsigned char TestForKey(void) {
  unsigned char KeyEventOccurred;
  KeyEventOccurred = Serial.available();
  return KeyEventOccurred;
}

State DumpRight_ApproachingBins() {
  OneChip();
  DumpRight.Set(DumpRight_DepositOneChip);
}
void StartTimer0() {
  TMRArd_InitTimer(0, TIME_TO_NEXT_BIN);
}
void MoveToNextBin() {
  StartTimer0();
}
State DumpRight_DepositOneChip() {
  if (IsPulseFinished()) {
    MoveToNextBin();
    DumpRight.Set(DumpRight_ApproachingNextBin);
  }
}
unsigned char TestTimer0Expired(void) {
  return (unsigned char)(TMRArd_IsTimerExpired(0));
}
bool InFrontOfBin() {
  return TestTimer0Expired();
}
bool BeaconDetected() {
  return false;
}
State DumpRight_ApproachingNextBin() {
  if (InFrontOfBin()) {
    if (BeaconDetected()) {
      DumpChips();
      DumpRight.Set(DumpRight_DumpAll);
    } else {
      OneChip();
      DumpRight.Set(DumpRight_DepositOneChip);
    }
  }
}

State DumpRight_DumpAll() {
  
}

State DumpRight_Debugging() {
  Serial.print("InFrontOfBin");
  char option = Serial.read();
  switch(option) {
    case ('r'): ReverseDirection(); break;
    case ('f'): FullRotation(); break;
    case ('d'): DumpChips(); break;
    case ('o'): OneChip(); break;
  }
}

void OneChip() {
  //Serial.print("One");
  chips--;
  Pulse(PULSES/12);
}

void FullRotation() {
  //Serial.print("Full");
  chips = 0;
  Pulse(PULSES);
}
void DumpChips() {
  //Serial.print("Dumping");
  Pulse(PULSES * chips /12);
  chips = 12;
}

void ReverseDirection() {
  //Serial.print("Reversing");
  forward = !forward;
  if (forward) digitalWrite(4, HIGH);
  else digitalWrite(4, LOW);
}
