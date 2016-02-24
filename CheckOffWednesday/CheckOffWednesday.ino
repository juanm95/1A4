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
State FindRightBinIR_Spinning();
State FindRightBinIR_Sensing1KLight();
State FindRightBinIR_FindingRightMostBeacon();
State ApproachBinLine_ApproachingLine();
State ApproachBinLine_AdjustingToBePerpendicular();
State ApproachBinLine_Centered();
State DriveBy_Start();
State DriveBy_StrafeLeft();
State DriveBy_DepositChip();
State DriveBy_StrafeToCenter();
State DumpRight_ApproachingBins();
State DumpRight_DepositChip();
State DumpRight_ApproachingNextBin();
State DumpRight_DumpAll();
State DumpRight_ReturnToCenter();
SM Overall(Overall_Start);
SM DumpRight(DumpRight_ApproachingBins);
SM DriveBy(DriveBy_Start);
SM FindRightBinIR(FindRightBinIR_Spinning);
SM ApproachBinLine(ApproachBinLine_ApproachingLine);
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
/************************ Actions and Checks *******************************/
void MoveBackwards() {
  
}
bool LightSensed1K() {
  
}
void SpinCCW() {
  
}
void MoveForwards() {
  
}
bool LeftSensorSensesTape() {
  
}
void PivotCCW() {
  
}
void MoveRight() {
  
}
bool RightSensorSensesTape() {
  
}
void ReturnToCenter() {
  StartTimer0();  
}
bool LastBinVisited() {
  
}
bool AtCenter() {
  return true;
}

unsigned char TestForKey(void) {
  unsigned char KeyEventOccurred;
  KeyEventOccurred = Serial.available();
  return KeyEventOccurred;
}
void StartTimer0() {
  TMRArd_InitTimer(0, TIME_TO_NEXT_BIN);
}
void MoveToLeftBin() {
  StartTimer0();
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
/************************* Overall States ***********************************/
State Overall_Start() {
  Overall.Set(Overall_DriveBy);
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

State Overall_DriveBy() {
  EXEC(DriveBy);
}
State Overall_DumpRight() {
  if (atCenter) {
    
  } else {
    Overall.Set(Overall_DumpRight);
  }
  EXEC(DumpRight);
}

/************************ FindRightBinIRStates *****************************/

State FindRightBinIR_Spinning() {
  if (LightSensed1K()) FindRightBinIR.Set(FindRightBinIR_Sensing1KLight);
}

State FindRightBinIR_Sensing1KLight() {
  if (!LightSensed1K()) {
    SpinCCW();
    FindRightBinIR.Set(FindRightBinIR_FindingRightMostBeacon);   
  }
}
State FindRightBinIR_FindingRightMostBeacon() {
  if (LightSensed1K()) {
    MoveForwards();
    FindRightBinIR.Set(FindRightBinIR_Spinning);
    Overall.Set(Overall_ApproachBinLine);
  }
}
/************************ ApproachBinLine States ***************************/

State ApproachBinLine_ApproachingLine() {
  if (LeftSensorSensesTape()) {
    PivotCCW();
    ApproachBinLine.Set(ApproachBinLine_AdjustingToBePerpendicular);
  }
}


//Potential Bug, adjusting and losing sight of the beacon
State ApproachBinLine_AdjustingToBePerpendicular() {
  if (RightSensorSensesTape()) {
    MoveRight();
    ApproachBinLine.Set(ApproachBinLine_Centered);
  }
}
State ApproachBinLine_Centered() {
  if (!LightSensed1K()) {
    ApproachBinLine.Set(ApproachBinLine_ApproachingLine);
    Overall.Set(Overall_DriveBy); 
  }
}
/************************ DriveBy States ***********************************/
State DriveBy_Start() {
  MoveToLeftBin();
  DriveBy.Set(DriveBy_StrafeLeft);
}
State DriveBy_StrafeLeft() {
  if (InFrontOfBin()) {
    OneChip();
    DriveBy.Set(DriveBy_DepositChip);
  }
}

State DriveBy_DepositChip(){
  if (IsPulseFinished()) {
    if (LastBinVisited()) {
      ReturnToCenter();
      DriveBy.Set(DriveBy_StrafeToCenter);
    }
    MoveToLeftBin();
    DriveBy.Set(DriveBy_StrafeLeft);
  }
}

State DriveBy_StrafeToCenter(){
  if(AtCenter()) {
    MoveBackwards();
    DriveBy.Set(DriveBy_StrafeLeft);
    Overall.Set(Overall_ApproachTheForce);
  }
}
/************************ DumpRight States *********************************/

State DumpRight_ApproachingBins() {
  OneChip();
  DumpRight.Set(DumpRight_DepositOneChip);
}

State DumpRight_DepositOneChip() {
  if (IsPulseFinished()) {
    MoveToLeftBin();
    DumpRight.Set(DumpRight_ApproachingNextBin);
  }
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


