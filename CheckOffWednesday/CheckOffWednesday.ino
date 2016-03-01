#include <Timers.h>

#include <SM.h>
#include <State.h>

#include <Pulse.h>
/*
 * This program waits for a key input and then responds by performing a full motor rotation. 
 * Change the period and pulses to adjust the degree of rotation and the speed of rotation.
 */
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
/************ Demos and Prototypes ***************/
State Debugger();
/************ Constants **************/
int PERIOD = 100;   
int PULSES = 90;
int TIME_TO_NEXT_BIN = 3000;// ms
int TAPE_THRESHOLD = 200;
int RETRACT_AMOUNT = 10;
//ANALOG
int FRONT_TS = 1;
int BACK_TS = 2;
int LEFT_TS = 4;
int RIGHT_TS = 3;
int LED = 0;
//DIGITAL
int FR_MOTOR_DIR = 7;
int FL_MOTOR_DIR = 8;
int BR_MOTOR_DIR = 12;
int BL_MOTOR_DIR = 13;
int FR_MOTOR_EN =  6;
int FL_MOTOR_EN = 9;
int BR_MOTOR_EN = 10;
int BL_MOTOR_EN = 11;
int DISPENSER_STEP = 3;
int DISPENSER_DIR = 2;
//1ms
void setupDispenser() {
  pinMode(DISPENSER_STEP, OUTPUT);
  pinMode(DISPENSER_DIR, OUTPUT);
  digitalWrite(DISPENSER_STEP, HIGH); //Instantiate to 
  digitalWrite(DISPENSER_DIR, HIGH);
  InitPulse(DISPENSER_STEP, PERIOD);
}
void setupMotor() {
  pinMode(FR_MOTOR_DIR, OUTPUT);
  pinMode(FL_MOTOR_DIR, OUTPUT);
  pinMode(BR_MOTOR_DIR, OUTPUT);
  pinMode(BL_MOTOR_DIR, OUTPUT);
  pinMode(FR_MOTOR_EN, OUTPUT);
  pinMode(FL_MOTOR_EN, OUTPUT);
  pinMode(BR_MOTOR_EN, OUTPUT);
  pinMode(BL_MOTOR_EN, OUTPUT);
  // Set all motors off
  digitalWrite(FR_MOTOR_EN, LOW);
  digitalWrite(FL_MOTOR_EN, LOW);
  digitalWrite(BR_MOTOR_EN, LOW);
  digitalWrite(BL_MOTOR_EN, LOW);
}
void setupSensors() {
}
void setup() {
  Serial.begin(9600);
  setupDispenser();
  setupMotor();
  setupSensors();
}
unsigned char TestForKey();
void RespToKey();
void RetractDirection();
void FullRotation();
void DumpChips();
void OneChip();
bool forward = true;
int chips = 12;
void loop() {
  EXEC(Overall);
}
/************************ Actions and Checks *******************************/
void FRForward() {
  digitalWrite(FR_MOTOR_DIR, LOW);
}
void FRBack() {
  digitalWrite(FR_MOTOR_DIR, HIGH);
}
void FLForward() {
  digitalWrite(FL_MOTOR_DIR, HIGH);
}
void FLBack() {
  digitalWrite(FL_MOTOR_DIR, LOW);
}
void BRForward() {
  digitalWrite(BR_MOTOR_DIR, LOW);
}
void BRBack() {
  digitalWrite(BR_MOTOR_DIR, HIGH);
}
void BLForward() {
  digitalWrite(BL_MOTOR_DIR, HIGH);
}
void BLBack() {
  digitalWrite(BL_MOTOR_DIR, LOW);
}
void ActivateMotors() {
  digitalWrite(FR_MOTOR_EN, HIGH);
  digitalWrite(FL_MOTOR_EN, HIGH);
  digitalWrite(BR_MOTOR_EN, HIGH);
  digitalWrite(BL_MOTOR_EN, HIGH);
}
void StopMotors() {
  digitalWrite(FR_MOTOR_EN, LOW);
  digitalWrite(FL_MOTOR_EN, LOW);
  digitalWrite(BR_MOTOR_EN, LOW);
  digitalWrite(BL_MOTOR_EN, LOW);
}
void MoveBackwards() {
  ActivateMotors();
  digitalWrite(FR_MOTOR_DIR, LOW);
  digitalWrite(FL_MOTOR_DIR, LOW);
  digitalWrite(BR_MOTOR_DIR, LOW);
  digitalWrite(BL_MOTOR_DIR, LOW);
 }
bool LightSensed1K() {
  if (analogRead(LED) > HIGH) return true;
}
void SpinCCW() {
  
}
void MoveForwards() {
  ActivateMotors();
  FRForward();
  FLForward();
  BRForward();
  BLForward();
}
bool tapeEventOccurred(int sensorVal) {
  return sensorVal > TAPE_THRESHOLD;
}
bool LeftSensorSensesTape() {
  return tapeEventOccurred(analogRead(LEFT_TS));
}
bool RightSensorSensesTape() {
  return tapeEventOccurred(analogRead(RIGHT_TS));
}
bool BackSensorSensesTape() {
  return tapeEventOccurred(analogRead(BACK_TS));
}
bool FrontSensorSensesTape() {
  return tapeEventOccurred(analogRead(FRONT_TS));
}
void togglePin(int pin) {
  static bool toggle = false;
  if (toggle) {
    digitalWrite(pin, HIGH);
  } else {
    digitalWrite(pin, LOW);
  }
  toggle = !toggle;
}
void PivotCCW() {
  
}
void MoveRight() {
  
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
void startBinTimer() {
    TMRArd_InitTimer(0, TIME_TO_NEXT_BIN);
}
void MoveToLeftBin() {
  ActivateMotors();
  digitalWrite(FR_MOTOR_DIR, HIGH);
  digitalWrite(FL_MOTOR_DIR, LOW);
  digitalWrite(BR_MOTOR_DIR, LOW);
  digitalWrite(BL_MOTOR_DIR, HIGH);
  startBinTimer();
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
int stepSize[12] = {18, 5, 5, 6, 4, 5, 4, 4, 5, 5, 5, 26 - RETRACT_AMOUNT};
void BatchDump() {
  ExtendDirection();
  chips -= 4;
  Pulse(stepSize[chips]+stepSize[chips+1]+stepSize[chips+2]+stepSize[chips+3] + RETRACT_AMOUNT);
  if (chips == 0) {
    while (!IsPulseFinished()){}
    RetractDirection();
    FullRotation();
    chips = 12;
  } else {
    while (!IsPulseFinished()){}
    RetractDirection();
    Pulse(RETRACT_AMOUNT);
  }
}
void OneChip() {
  ExtendDirection();
  //Serial.print("One");
  chips--;
  Pulse(stepSize[chips] + RETRACT_AMOUNT);
  if (chips == 0) {    
    while (!IsPulseFinished()){}
    RetractDirection();
    FullRotation();
    chips = 12;
  } else {
    while (!IsPulseFinished()){}
    RetractDirection();
    Pulse(RETRACT_AMOUNT);
  }
}
int stepCounter = 0;
void OneStep() {
  stepCounter++;
  Serial.println(stepCounter);
  Pulse(1);
}
void FullRotation() {
  int numPulses = 0;
  for (int i = 0 ; i < 12 ; i++)numPulses += stepSize[i];
  Pulse(numPulses);
  chips = 12;
}
void DumpChips() {
  Pulse(PULSES * chips /12);
  chips = 12;
}
void RetractDirection() {
  digitalWrite(DISPENSER_DIR, LOW);
}
void ExtendDirection() {
  digitalWrite(DISPENSER_DIR, HIGH);
}
/************************* Overall States ***********************************/
State Overall_Start() {
//  Overall.Set(Overall_DriveBy);
  if (TestForKey() && IsPulseFinished()) {
    Debugger();
  }
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
State Overall_ApproachBinLine() {
  
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

State Debugger() {
//  Serial.print("InFrontOfBin");
  char option = Serial.read();
  switch(option) {
    case ('0'): stepCounter = 0; break;
    case ('r'): RetractDirection(); break;
    case ('e'): ExtendDirection(); break;
    case ('f'): FullRotation(); break;
    case ('d'): DumpChips(); break;
    case ('o'): OneChip(); break;
    case ('B'): BatchDump(); break;
    case ('O'): OneStep(); break;
    case ('l'): MoveToLeftBin(); break;
    case ('F'): MoveForwards(); break;
    case ('b'): MoveBackwards(); break;
    case ('s'): StopMotors(); break;
    case ('1'): togglePin(6); break;
    case ('2'): togglePin(7); break;
    case ('3'): togglePin(8); break;
    case ('4'): togglePin(9); break;
    case ('5'): togglePin(10); break;
    case ('6'): togglePin(11); break;
    case ('7'): togglePin(12); break;
    case ('8'): togglePin(13); break;
  }
}
