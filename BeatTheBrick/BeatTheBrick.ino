#include <Timers.h>

#include <SM.h>
#include <State.h>

bool DEBUGGING = false;
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
/*
 * This program waits for a key input and then responds by performing a full motor rotation. 
 * Change the period and pulses to adjust the degree of rotation and the speed of rotation.
 */
 State Overall_WaitingForOnSignal();
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
SM Overall(Overall_WaitingForOnSignal);
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
int LightBuffer = 100;
//ANALOG
#define FRONT_TS A2
#define BACK_TS A3
#define LEFT_TS  A1
#define RIGHT_TS  A0
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
int GAME_TIME = 120000;
int BUTTON = 4;
int LED = 5;
//1ms
void setupDispenser() {
  pinMode(DISPENSER_STEP, OUTPUT);
  pinMode(DISPENSER_DIR, OUTPUT);
  digitalWrite(DISPENSER_STEP, HIGH); //Instantiate to 
  digitalWrite(DISPENSER_DIR, HIGH);
}
int MOTOR_FULL_PERIOD = 100;
int MOTOR_HALF_SPEED = 200;
//void setupMotorPWM() {
//  digitalWrite(FR_MOTOR_EN, LOW);
//  digitalWrite(FL_MOTOR_EN, LOW);
//  digitalWrite(BR_MOTOR_EN, LOW);
//  digitalWrite(BL_MOTOR_EN, LOW);
//  pinMode(FR_MOTOR_DIR, OUTPUT);
//  pinMode(FL_MOTOR_DIR, OUTPUT);
//  pinMode(BR_MOTOR_DIR, OUTPUT);
//  pinMode(BL_MOTOR_DIR, OUTPUT);
//  pinMode(FR_MOTOR_EN, OUTPUT);
//  pinMode(FL_MOTOR_EN, OUTPUT);
//  pinMode(BR_MOTOR_EN, OUTPUT);
//  pinMode(BL_MOTOR_EN, OUTPUT);
//  InitPulse(FR_MOTOR_EN, MOTOR_FULL_PERIOD);
//  InitPulse(FL_MOTOR_EN, MOTOR_FULL_PERIOD);
//  InitPulse(BR_MOTOR_EN, MOTOR_FULL_PERIOD);
//  InitPulse(BL_MOTOR_EN, MOTOR_FULL_PERIOD);
//}
void setupMotor() {
//  setPwmFrequency(BL_MOTOR_EN, 1);
//  TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequen  // Set all motors off
  delay(300);
  pinMode(FR_MOTOR_DIR, OUTPUT);
  pinMode(FL_MOTOR_DIR, OUTPUT);
  pinMode(BR_MOTOR_DIR, OUTPUT);
  pinMode(BL_MOTOR_DIR, OUTPUT);
  pinMode(FR_MOTOR_EN, OUTPUT);
  pinMode(FL_MOTOR_EN, OUTPUT);
  pinMode(BR_MOTOR_EN, OUTPUT);
  pinMode(BL_MOTOR_EN, OUTPUT);
  digitalWrite(FR_MOTOR_EN, LOW);
  digitalWrite(FL_MOTOR_EN, LOW);
  digitalWrite(BR_MOTOR_EN, LOW);
  digitalWrite(BL_MOTOR_EN, LOW);
}
void setupSensors() {
  pinMode(LED, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);
}
void setup() {
  setupMotor();
  Serial.begin(9600);
  setupDispenser();
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

void Pulse(int pulses) { 
  for(int i = 0; i < pulses ; i++) {
    digitalWrite(DISPENSER_STEP, HIGH);
    delay(5);
    digitalWrite(DISPENSER_STEP, LOW);
    delay(5);
  }
}

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
void AnalogFRSpeed(int speed) {
  analogWrite(FR_MOTOR_EN, speed);
}
void AnalogBRSpeed(int speed) {
  analogWrite(BR_MOTOR_EN, speed);
}
void AnalogFLSpeed(int speed) {
  analogWrite(FL_MOTOR_EN, speed);
}
void AnalogBLSpeed(int speed) {
  analogWrite(BL_MOTOR_EN, speed);
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
  FRBack();
  FLBack();
  BRBack();
  BLBack();
}
bool LightSensed1K() {
  return GetFrequency(LED) > 50;
}
void SpinCCW() {  
  ActivateMotors();
  FRForward();
  BRForward();
  BLBack();
  FLBack();
}
void SpinCW() {
  ActivateMotors();
  FRBack();
  BRBack();
  BLForward();
  FLForward();
}
void PivotCCW() {

//  analogWrite(FR_MOTOR_EN, 255);
//  analogWrite(BR_MOTOR_EN, 255);
//  analogWrite(FL_MOTOR_EN, 230);
//  analogWrite(11, 230);
}
void PivotCW() {

}
int motorSpeed = 0;
void PWMTest() {
  analogWrite(FR_MOTOR_EN, motorSpeed);
}
void SpeedUp() {
  Serial.println(motorSpeed);
  if (motorSpeed >= 255) return;
  motorSpeed += 5;
}
void SlowDown() {
  Serial.println(motorSpeed);
  if (motorSpeed <= 0) return;
  motorSpeed -= 5;
}
void AllMotorsForward() {
  FRForward();
  FLForward();
  BRForward();
  BLForward();
}
void MoveForwards() {
  AllMotorsForward();
  ActivateMotors();
}
bool tapeEventOccurred(int sensorVal) {
  return sensorVal > TAPE_THRESHOLD;
}
bool LeftSensorSensesTape() {
  return digitalRead(LEFT_TS) == LOW;
}
bool RightSensorSensesTape() {
  return digitalRead(RIGHT_TS) == LOW;
}
bool BackSensorSensesTape() {
  return digitalRead(BACK_TS) == LOW;
}
bool FrontSensorSensesTape() {
  return digitalRead(FRONT_TS) == LOW;
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
void MoveRight() {
  
}
void ReturnToCenter() {
//  StartTimer0();  
}
bool LastBinVisited() {
  return false;
}
bool AtCenter() {
  return true;
}

unsigned char TestForKey(void) {
  unsigned char KeyEventOccurred;
  KeyEventOccurred = Serial.available();
  return KeyEventOccurred;
}
void StartTimer0(int time) {
  TMRArd_InitTimer(0, time);
}
void startBinTimer() {
    TMRArd_InitTimer(0, TIME_TO_NEXT_BIN);
}
void StartGameTimer() {
  TMRArd_InitTimer(1, GAME_TIME);
}
void MoveToLeftBin() {
  ActivateMotors();
  FLBack();
  BLForward();
  FRForward();
  BRBack();
  AnalogBRSpeed(200);
  AnalogBLSpeed(200);
  AnalogFRSpeed(200);
  AnalogFLSpeed(200);
}
void RightStrafe() {
  ActivateMotors();
  FLForward();
  BLBack();
  FRBack();
  BRForward();
}
void CCWArcLeft() {
  ActivateMotors();
  FLBack();
  BLForward();
  FRForward();
  BRBack();
  
  AnalogFRSpeed(200);
  AnalogFLSpeed(200);
  
  AnalogBRSpeed(10);
  AnalogBLSpeed(10);
} 
void CWArcLeft() {
  ActivateMotors();
  FLBack();
  BLForward();
  FRForward();
  BRBack();
  
  AnalogBRSpeed(200);
  AnalogBLSpeed(200);
    
  AnalogFLSpeed(10);
  AnalogFRSpeed(10);
}

void MoveForwardLeft() {
  ActivateMotors();
  BLForward();
  FRForward();
  digitalWrite(FL_MOTOR_EN, LOW);
  digitalWrite(BR_MOTOR_EN, LOW);
}

unsigned char Timer0Expired(void) {
  return (unsigned char)(TMRArd_IsTimerExpired(0));
}
bool InFrontOfBin() {
  return Timer0Expired();
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
    //while (!IsPulseFinished()){}
    RetractDirection();
    FullRotation();
    chips = 12;
  } else {
    //while (!IsPulseFinished()){}
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
    //while (!IsPulseFinished()){}
    RetractDirection();
    FullRotation();
    chips = 12;
  } else {
    //while (!IsPulseFinished()){}
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
long GetFrequency(int pin) {
  #define SAMPLES 50
  long period = 0;
  for(unsigned int j=0; j<SAMPLES; j++) period += pulseIn(pin, LOW, 1000);
  return period / SAMPLES;
}
/************************* Overall States ***********************************/
bool ButtonDetected() {
  return digitalRead(BUTTON) == LOW;
}
State Overall_WaitingForOnSignal() {
  Serial.println("Waiting For On");
  StopMotors();
  if (ButtonDetected()) {
    StartGameTimer();
    Overall.Set(Overall_Start);
  }
}
State Overall_Start() {
  if (!DEBUGGING) {
    SpinCW();
    Overall.Set(Overall_FindRightBinIR);
  }
//  Overall.Set(Overall_DriveBy);
  Serial.println(GetFrequency(LED));
   if (TestForKey()) {
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
  EXEC(ApproachBinLine);
}
State Overall_FindRightBinIR() {
  EXEC(FindRightBinIR);
}
/************************ FindRightBinIRStates *****************************/

State FindRightBinIR_Spinning() {
  Serial.println("Spinning");
  if (LightSensed1K()) {
    FindRightBinIR.Set(FindRightBinIR_Sensing1KLight);
  }
}

State FindRightBinIR_Sensing1KLight() {
  Serial.println("Sensing");
  if (!LightSensed1K()) {
    StartTimer0(1000);
    FindRightBinIR.Set(FindRightBinIR_NotSensing1KLight);
  }
}
State FindRightBinIR_NotSensing1KLight() {
  Serial.println("Not");
  if (LightSensed1K()) {
    FindRightBinIR.Set(FindRightBinIR_Sensing1KLight);
  }
  if (Timer0Expired()) {
    SpinCCW();
    FindRightBinIR.Set(FindRightBinIR_FindingRightMostBeacon);
  }
}
State FindRightBinIR_FindingRightMostBeacon() {
  Serial.println("Finding");
  if (LightSensed1K()) {
    SpinCW();
    delay(125);
    MoveForwards();
    FindRightBinIR.Set(FindRightBinIR_Spinning);
    Overall.Set(Overall_ApproachBinLine);
  }
}
/************************ ApproachBinLine States ***************************/

State ApproachBinLine_ApproachingLine() {
  Serial.println("Approaching");
  if (LeftSensorSensesTape()) {
    MoveBackwards();
    delay(50);
    StopMotors();
    delay(100);
    ApproachBinLine.Set(ApproachBinLine_FirstBin);
//    StopMotors();
//    if (IsPulseFinished()) {
//      BatchDump();
//    }
    // ApproachBinLine.Set(ApproachBinLine_AdjustingToBePerpendicular);
  }
}

State ApproachBinLine_InchForward() {
  if (!FrontSensorSensesTape()) {
    StopMotors();
    ApproachBinLine.Set(ApproachBinLine_FirstBin);
  }
}

State ApproachBinLine_FirstBin() {
  BatchDump();
  //while (!IsPulseFinished());
  OneChip();
  //while (!IsPulseFinished());
  OneChip();
  //StartTimer0(300);
  //SpinCCW();
  //delay(100);
  StopMotors();
  Overall.Set(Overall_DriveBy);
  ApproachBinLine.Set(ApproachBinLine_ApproachingLine);
}

State ApproachBinLine_LeftAdjust() {
  if (FrontSensorSensesTape()) {
    CWArcLeft();
  } else if (!LeftSensorSensesTape()) { //Tilted too Clockwise
    CCWArcLeft();
  } else {
    MoveToLeftBin();
  }
  if (Timer0Expired()) {
    Overall.Set(Overall_DriveBy);
    ApproachBinLine.Set(ApproachBinLine_ApproachingLine);
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
  DriveBy.Set(DriveBy_StrafeLeft);
  StartTimer0(600);
}
State DriveBy_StrafeLeft() {

  if(FrontSensorSensesTape() && !LeftSensorSensesTape() && RightSensorSensesTape())
  {
    CWArcLeft();
  }
  else if(FrontSensorSensesTape() && LeftSensorSensesTape() && !RightSensorSensesTape())
  {
    CCWArcLeft();
  }
  else if(FrontSensorSensesTape() && !LeftSensorSensesTape() && !RightSensorSensesTape())
  {
    MoveForwards();
    delay(10);
    //MoveForwardLeft();
  }
  else if(!FrontSensorSensesTape() && !LeftSensorSensesTape() && !RightSensorSensesTape())
  {
    MoveForwards();
    delay(10);
    //MoveForwardLeft();
  }
  else{
    MoveToLeftBin();
  }
  
  if (LightSensed1K() && Timer0Expired()) {
    StopMotors();
    delay(100);
    OneChip();
    DriveBy.Set(DriveBy_DepositChip);
  }

}

State DriveBy_DepositChip(){
  //if (IsPulseFinished()) {
    if (LastBinVisited()) {
      ReturnToCenter();
      DriveBy.Set(DriveBy_StrafeToCenter);
    }
    StartTimer0(500);
    DriveBy.Set(DriveBy_StrafeLeft);
  //}
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
  //if (IsPulseFinished()) {
    MoveToLeftBin();
    DumpRight.Set(DumpRight_ApproachingNextBin);
  //}
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
    case ('+'): SpeedUp(); break;
    case ('-'): SlowDown(); break;
    case ('p'): PivotCCW(); break;
    case ('a'): Overall_FindRightBinIR(); break;
    case ('c'): SpinCW(); break;
    
    case ('q'): CCWArcLeft(); break;
    case ('w'): CWArcLeft(); break; 

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
