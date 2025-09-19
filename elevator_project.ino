#include <LiquidCrystal.h>
// ---------------- LCD ----------------
LiquidCrystal lcd(5, 6, 7, 8, 9, 10); // RS,E,D4..D7

// ---------------- Keypad ----------------
const int colPins[3] = {33, 34, 35};
const int rowPins[4] = {36, 37, 38, 39};
char mappedKey[4][3] = { // already working map
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

// ---------------- Motors / Doors ----------------
const int MOTOR_UP_PIN = 27;
const int MOTOR_DOWN_PIN = 28;
const int DOOR_OPEN_PIN = 29;
const int DOOR_CLOSE_PIN = 30;

// ---------------- Ultrasonic ----------------
const int TRIG_PIN = 31;
const int ECHO_PIN = 32;

// ---------------- Outside call buttons ----------------
const int PIN_GU = 21; // Ground => floor 0
const int PIN_FU = 22;
const int PIN_FD = 23;
const int PIN_SU = 24;
const int PIN_SD = 25;
const int PIN_TD = 26;

// ---------------- Constants ----------------
const int NUM_FLOORS = 4; // 0..3
const int FLOOR_MAX_CM[NUM_FLOORS] = {15,30,45,999};

// Tuned for speed: reduce delays/checks and door times
const unsigned long MOVE_TIMEOUT_MS = 20000UL;
const unsigned long DOOR_RUN_MS = 1000;   // shortened
const unsigned long DOOR_WAIT_MS = 2000;  // shortened
const int STABLE_READS = 2;               // require fewer stable reads
const int MOVE_POLL_DELAY_MS = 10;        // faster polling

// ---------------- FIFO queue ----------------
const int QUEUE_SIZE = 16;
int reqQueue[QUEUE_SIZE];
int qHead=0,qTail=0,qCount=0;
bool enqueueRequest(int floor){
  if(qCount>=QUEUE_SIZE) return false;
  reqQueue[qTail]=floor; qTail=(qTail+1)%QUEUE_SIZE; qCount++;
  return true;
}
int dequeueRequest(){
  if(qCount==0) return -1;
  int v=reqQueue[qHead]; qHead=(qHead+1)%QUEUE_SIZE; qCount--; return v;
}
bool queueEmpty(){ return qCount==0; }

// ---------------- State ----------------
int currentFloor=-1;
int targetFloor=-1;
bool busyMoving=false;
unsigned long lastButtonTime[6]={0};

// ---------------- Software PWM for elevator motor (no pin changes) ----------------
const unsigned long MOTOR_PWM_PERIOD_US = 2000UL; // 2 ms period (~500 Hz)
uint8_t motorDuty = 255; // 0-255, 255 = 100% (max speed)
int motorPWMDir = 0;     // 0=stopped, 1=up, -1=down
unsigned long motorPWMStartUs = 0;
bool motorPWMOutState = false;

void startMotorPWMUp(uint8_t duty){
  motorDuty = duty;
  motorPWMDir = 1;
  motorPWMStartUs = micros();
  motorPWMOutState = false;
}
void startMotorPWMDown(uint8_t duty){
  motorDuty = duty;
  motorPWMDir = -1;
  motorPWMStartUs = micros();
  motorPWMOutState = false;
}
void stopMotorPWM(){
  motorPWMDir = 0;
  motorPWMOutState = false;
  digitalWrite(MOTOR_UP_PIN, LOW);
  digitalWrite(MOTOR_DOWN_PIN, LOW);
}

// call this frequently (non-blocking) — it will toggle motor pins according to duty
void motorPWMTick(){
  if(motorPWMDir == 0) return;
  unsigned long now = micros();
  unsigned long phase = (now - motorPWMStartUs) % MOTOR_PWM_PERIOD_US;
  unsigned long onTime = ((unsigned long)MOTOR_PWM_PERIOD_US * motorDuty) / 255UL;
  bool shouldBeOn = (phase < onTime);

  if(shouldBeOn != motorPWMOutState){
    motorPWMOutState = shouldBeOn;
    if(motorPWMDir == 1){
      digitalWrite(MOTOR_DOWN_PIN, LOW);
      digitalWrite(MOTOR_UP_PIN, shouldBeOn ? HIGH : LOW);
    } else { // down
      digitalWrite(MOTOR_UP_PIN, LOW);
      digitalWrite(MOTOR_DOWN_PIN, shouldBeOn ? HIGH : LOW);
    }
  }
}

// ---------------- Helpers (use PWM versions) ----------------
void stopElevatorMotor(){ stopMotorPWM(); }
void elevatorUp(){ startMotorPWMUp(motorDuty); }
void elevatorDown(){ startMotorPWMDown(motorDuty); }

void stopDoorMotor(){ digitalWrite(DOOR_OPEN_PIN, LOW); digitalWrite(DOOR_CLOSE_PIN, LOW); }
void doorOpen(){ digitalWrite(DOOR_OPEN_PIN, HIGH); digitalWrite(DOOR_CLOSE_PIN, LOW); }
void doorClose(){ digitalWrite(DOOR_OPEN_PIN, LOW); digitalWrite(DOOR_CLOSE_PIN, HIGH); }

// ---------------- Ultrasonic helpers ----------------
long getDistanceCm(){
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if(duration==0) return 400;
  return (long)((duration*0.034)/2.0);
}
int detectFloorFromDistance(long cm){
  if(cm<=0) return -1;
  for(int f=0;f<NUM_FLOORS;f++) if(cm<=FLOOR_MAX_CM[f]) return f;
  return NUM_FLOORS-1;
}
int readFloorMajority(){
  const int N=5;
  int counts[NUM_FLOORS]={0};
  for(int i=0;i<N;i++){
    long d=getDistanceCm();
    int f=detectFloorFromDistance(d);
    if(f>=0 && f<NUM_FLOORS) counts[f]++;
    delay(20);
    motorPWMTick(); // keep PWM alive while reading
  }
  int best=-1,bestc=0;
  for(int f=0;f<NUM_FLOORS;f++) if(counts[f]>bestc){bestc=counts[f];best=f;}
  if(best==-1) best=detectFloorFromDistance(getDistanceCm());
  return best;
}

// keypad scanning (simple, using mappedKey)
char readKeyMappedOnce(){
  for(int c=0;c<3;c++){ pinMode(colPins[c],OUTPUT); digitalWrite(colPins[c],HIGH); }
  for(int r=0;r<4;r++) pinMode(rowPins[r],INPUT_PULLUP);
  for(int c=0;c<3;c++){
    digitalWrite(colPins[c],LOW); delayMicroseconds(30);
    for(int r=0;r<4;r++){
      if(digitalRead(rowPins[r])==LOW){
        delay(15); while(digitalRead(rowPins[r])==LOW){ motorPWMTick(); delay(6); }
        for(int cc=0;cc<3;cc++) digitalWrite(colPins[cc],HIGH);
        return mappedKey[r][c];
      }
    }
    digitalWrite(colPins[c],HIGH);
  }
  return '\0';
}

// outside buttons -> enqueue
void handleOutsideButtons(){
  const int pins[6]={PIN_GU,PIN_FU,PIN_FD,PIN_SU,PIN_SD,PIN_TD};
  const int floors[6]={0,1,1,2,2,3};
  unsigned long now=millis();
  for(int i=0;i<6;i++){
    if(digitalRead(pins[i])==HIGH && now-lastButtonTime[i]>150){
      enqueueRequest(floors[i]);
      lastButtonTime[i]=now;
    }
  }
}

void operateDoorSequence(int f){
  lcd.clear(); lcd.print("Door Opening");
  doorOpen(); delay(DOOR_RUN_MS); stopDoorMotor();
  lcd.clear(); lcd.print("Door Open"); delay(DOOR_WAIT_MS);
  lcd.clear(); lcd.print("Door Closing");
  doorClose(); delay(DOOR_RUN_MS); stopDoorMotor();
  currentFloor=f;
  lcd.clear(); lcd.print("Arrived: F"); lcd.print(currentFloor);
  delay(400);
}

// ---------------- Movement logic ----------------
void moveToFloor(int f){
  if(f<0||f>=NUM_FLOORS) return;
  busyMoving=true;
  int start=currentFloor; if(start==-1) start=readFloorMajority();
  if(start==f){ stopElevatorMotor(); operateDoorSequence(f); busyMoving=false; return; }

  // Optional acceleration ramp (uncomment to use):
  // for(uint8_t d = 120; d <= 255; d += 25){ motorDuty = d; delay(30); motorPWMTick(); }

  if(start<f){ elevatorUp(); lcd.clear(); lcd.print("Moving Up..."); }
  else{ elevatorDown(); lcd.clear(); lcd.print("Moving Down..."); }

  int stable=0, last=-1;
  unsigned long t0=millis();
  while(millis()-t0 < MOVE_TIMEOUT_MS){
    motorPWMTick(); // keep PWM active
    int rf = readFloorMajority();
    if(rf==f){
      if(rf==last) stable++;
      else { stable=1; last=rf; }
    } else { stable=0; last=rf; }
    if(stable>=STABLE_READS) break;
    delay(MOVE_POLL_DELAY_MS);
  }

  stopElevatorMotor();
  operateDoorSequence(f);
  busyMoving=false;
}

// LCD status
void updateLCD(){
  lcd.setCursor(0,0); lcd.print("Target:");
  if(targetFloor<0) lcd.print(" - "); else{ lcd.print(" "); lcd.print(targetFloor); lcd.print(" "); }
  lcd.setCursor(0,1); lcd.print("Now: ");
  if(busyMoving&&targetFloor>=0) lcd.print(targetFloor);
  else if(currentFloor>=0) lcd.print(currentFloor); else lcd.print("-");
  lcd.print(" ");
  lcd.setCursor(0,2); lcd.print("Queue:"); lcd.print(qCount);
  lcd.print(" ");
  lcd.setCursor(0,3); lcd.print("LM044L 20x4 ");
}

// ---------------- Setup ----------------
void setup(){
  Serial.begin(115200);
  pinMode(MOTOR_UP_PIN,OUTPUT); pinMode(MOTOR_DOWN_PIN,OUTPUT);
  stopElevatorMotor();
  pinMode(DOOR_OPEN_PIN,OUTPUT); pinMode(DOOR_CLOSE_PIN,OUTPUT); stopDoorMotor();
  pinMode(TRIG_PIN,OUTPUT); pinMode(ECHO_PIN,INPUT);
  pinMode(PIN_GU,INPUT); pinMode(PIN_FU,INPUT); pinMode(PIN_FD,INPUT);
  pinMode(PIN_SU,INPUT); pinMode(PIN_SD,INPUT); pinMode(PIN_TD,INPUT);
  for(int c=0;c<3;c++) pinMode(colPins[c], OUTPUT), digitalWrite(colPins[c], HIGH);
  for(int r=0;r<4;r++) pinMode(rowPins[r], INPUT_PULLUP);

  lcd.begin(20,4);
  lcd.clear(); lcd.print("Elevator Ready");
  delay(500);
  currentFloor=readFloorMajority();
  updateLCD();
}

// ---------------- Loop ----------------
void loop(){
  motorPWMTick(); // keep PWM toggling even if idle
  handleOutsideButtons();
  char k=readKeyMappedOnce();
  if(k!='\0'){
    if(k=='0') enqueueRequest(0);
    else if(k>='1'&&k<='3') enqueueRequest(k-'0');
    else if(k=='*'){ if(!busyMoving&&targetFloor<0&&currentFloor>=0) operateDoorSequence(currentFloor); }
    else if(k=='#'){ if(!busyMoving){ doorOpen(); delay(DOOR_RUN_MS); stopDoorMotor(); } }
    updateLCD();
    delay(100);
  }

  if(!busyMoving && !queueEmpty()){
    int nextF=dequeueRequest();
    targetFloor=nextF;
    updateLCD();
    if(currentFloor==nextF && currentFloor!=-1){
      operateDoorSequence(nextF);
      targetFloor=-1;
    } else {
      moveToFloor(nextF);
      targetFloor=-1;
    }
  }
  updateLCD();
  delay(25);
}
