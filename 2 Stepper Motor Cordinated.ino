#define x0 4
#define x1 5
#define x2 6
#define x3 7
#define X_STEP_CNT = 0
bool X_DIR = false;
int _stepX = 0;
int xStepsPerCM = 40;

#define y0 8
#define y1 9
#define y2 10
#define y3 11
#define Y_STEP_CNT = 0
bool Y_DIR = false;
int _stepY = 0;
int yStepsPerCM = 42;

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

struct stepperInfo {
  // externally defined parameters
  float acceleration;
  volatile unsigned long minStepInterval; // ie. max speed, smaller is faster
  void (*dirFunc)(int);
  void (*stepFunc)();

  // derived parameters
  unsigned int c0;                // step interval for first step, determines acceleration
  long stepPosition;              // current position of stepper (total of all movements taken so far)

  // per movement variables (only changed once per movement)
  volatile int dir;                        // current direction of movement, used to keep track of position
  volatile unsigned int totalSteps;        // number of steps requested for current movement
  volatile bool movementDone = false;      // true if the current movement has been completed (used by main program to wait for completion)
  volatile unsigned int rampUpStepCount;   // number of steps taken to reach either max speed, or half-way to the goal (will be zero until this number is known)
  volatile unsigned long estStepsToSpeed;  // estimated steps required to reach max speed
  volatile unsigned long estTimeForMove;   // estimated time (interrupt ticks) required to complete movement
  volatile unsigned long rampUpStepTime;
  volatile float speedScale;               // used to slow down this motor to make coordinated movement with other motors

  // per iteration variables (potentially changed every interrupt)
  volatile unsigned int n;                 // index in acceleration curve, used to calculate next interval
  volatile float d;                        // current interval length
  volatile unsigned long di;               // above variable truncated
  volatile unsigned int stepCount;         // number of steps completed in current movement
};

void xDir(int dir) { if (dir == 0) { X_DIR = true; } else { X_DIR = false; } }
void xStep() {
  if (X_DIR) {
    _stepX++;
  } else {
    _stepX--;
  }
  if (_stepX > 7) {
    _stepX = 0;
  }
  if (_stepX < 0) {
    _stepX = 7;
  }
  switch (_stepX) {
    case 0:      digitalWrite(x0, LOW);      digitalWrite(x1, LOW);      digitalWrite(x2, LOW);      digitalWrite(x3, HIGH);      break;
    case 1:      digitalWrite(x0, LOW);      digitalWrite(x1, LOW);      digitalWrite(x2, HIGH);      digitalWrite(x3, HIGH);      break;
    case 2:      digitalWrite(x0, LOW);      digitalWrite(x1, LOW);      digitalWrite(x2, HIGH);      digitalWrite(x3, LOW);      break;
    case 3:      digitalWrite(x0, LOW);      digitalWrite(x1, HIGH);      digitalWrite(x2, HIGH);      digitalWrite(x3, LOW);      break;
    case 4:      digitalWrite(x0, LOW);      digitalWrite(x1, HIGH);      digitalWrite(x2, LOW);      digitalWrite(x3, LOW);      break;
    case 5:      digitalWrite(x0, HIGH);      digitalWrite(x1, HIGH);      digitalWrite(x2, LOW);      digitalWrite(x3, LOW);      break;
    case 6:      digitalWrite(x0, HIGH);      digitalWrite(x1, LOW);      digitalWrite(x2, LOW);      digitalWrite(x3, LOW);      break;
    case 7:      digitalWrite(x0, HIGH);      digitalWrite(x1, LOW);      digitalWrite(x2, LOW);      digitalWrite(x3, HIGH);      break;
    default:      digitalWrite(x0, LOW);      digitalWrite(x1, LOW);      digitalWrite(x2, LOW);      digitalWrite(x3, LOW);      break;
  }
  delayMicroseconds(500);
}

void yDir(int dir) { if (dir == 0) { Y_DIR = true; } else { Y_DIR = false; } }
void yStep() {
  if (Y_DIR) {
    _stepY--;
  } else {
    _stepY++;
  }
  if (_stepY > 7) {
    _stepY = 0;
  }
  if (_stepY < 0) {
    _stepY = 7;
  }
  switch (_stepY) {
    case 0:      digitalWrite(y0, LOW);      digitalWrite(y1, LOW);      digitalWrite(y2, LOW);      digitalWrite(y3, HIGH);      break;
    case 1:      digitalWrite(y0, LOW);      digitalWrite(y1, LOW);      digitalWrite(y2, HIGH);      digitalWrite(y3, HIGH);      break;
    case 2:      digitalWrite(y0, LOW);      digitalWrite(y1, LOW);      digitalWrite(y2, HIGH);      digitalWrite(y3, LOW);      break;
    case 3:      digitalWrite(y0, LOW);      digitalWrite(y1, HIGH);      digitalWrite(y2, HIGH);      digitalWrite(y3, LOW);      break;
    case 4:      digitalWrite(y0, LOW);      digitalWrite(y1, HIGH);      digitalWrite(y2, LOW);      digitalWrite(y3, LOW);      break;
    case 5:      digitalWrite(y0, HIGH);      digitalWrite(y1, HIGH);      digitalWrite(y2, LOW);      digitalWrite(y3, LOW);      break;
    case 6:      digitalWrite(y0, HIGH);      digitalWrite(y1, LOW);      digitalWrite(y2, LOW);      digitalWrite(y3, LOW);      break;
    case 7:      digitalWrite(y0, HIGH);      digitalWrite(y1, LOW);      digitalWrite(y2, LOW);      digitalWrite(y3, HIGH);      break;
    default:      digitalWrite(y0, LOW);      digitalWrite(y1, LOW);      digitalWrite(y2, LOW);      digitalWrite(y3, LOW);      break;
  }
  delayMicroseconds(500);
}
void resetStepperInfo( stepperInfo& si ) {
  si.n = 0;
  si.d = 0;
  si.di = 0;
  si.stepCount = 0;
  si.rampUpStepCount = 0;
  si.rampUpStepTime = 0;
  si.totalSteps = 0;
  si.stepPosition = 0;
  si.movementDone = false;
}

#define NUM_STEPPERS 2

volatile stepperInfo steppers[NUM_STEPPERS];

void setup() {
  pinMode(x0, OUTPUT);  pinMode(x1, OUTPUT);  pinMode(x2, OUTPUT);  pinMode(x3, OUTPUT);
  pinMode(y0, OUTPUT);  pinMode(y1, OUTPUT);  pinMode(y2, OUTPUT);  pinMode(y3, OUTPUT);

  digitalWrite(x0, LOW);  digitalWrite(x1, LOW);  digitalWrite(x2, LOW);  digitalWrite(x3, LOW);
  digitalWrite(y0, LOW);  digitalWrite(y1, LOW);  digitalWrite(y2, LOW);  digitalWrite(y3, LOW);  

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 1000;                             // compare value
  TCCR1B |= (1 << WGM12);                   // CTC mode
  TCCR1B |= ((1 << CS11) | (1 << CS10));    // 64 prescaler
  interrupts();

  steppers[0].dirFunc = xDir;
  steppers[0].stepFunc = xStep;
  steppers[0].acceleration = 2500;
  steppers[0].minStepInterval = 100;

  steppers[1].dirFunc = yDir;
  steppers[1].stepFunc = yStep;
  steppers[1].acceleration = 2500;
  steppers[1].minStepInterval = 100;
}

void resetStepper(volatile stepperInfo& si) {
  si.c0 = si.acceleration;
  si.d = si.c0;
  si.di = si.d;
  si.stepCount = 0;
  si.n = 0;
  si.rampUpStepCount = 0;
  si.movementDone = false;
  si.speedScale = 1;

  float a = si.minStepInterval / (float)si.c0;
  a *= 0.676;

  float m = ((a*a - 1) / (-2 * a));
  float n = m * m;

  si.estStepsToSpeed = n;
}

volatile byte remainingSteppersFlag = 0;

float getDurationOfAcceleration(volatile stepperInfo& s, unsigned int numSteps) {
  float d = s.c0;
  float totalDuration = 0;
  for (unsigned int n = 1; n < numSteps; n++) {
    d = d - (2 * d) / (4 * n + 1);
    totalDuration += d;
  }
  return totalDuration;
}

void prepareMovement(int whichMotor, long steps) {
  volatile stepperInfo& si = steppers[whichMotor];
  si.dirFunc( steps < 0 ? HIGH : LOW );
  si.dir = steps > 0 ? 1 : -1;
  si.totalSteps = abs(steps);
  resetStepper(si);
  
  remainingSteppersFlag |= (1 << whichMotor);

  unsigned long stepsAbs = abs(steps);

  if ( (2 * si.estStepsToSpeed) < stepsAbs ) {
    // there will be a period of time at full speed
    unsigned long stepsAtFullSpeed = stepsAbs - 2 * si.estStepsToSpeed;
    float accelDecelTime = getDurationOfAcceleration(si, si.estStepsToSpeed);
    si.estTimeForMove = 2 * accelDecelTime + stepsAtFullSpeed * si.minStepInterval;
  }
  else {
    // will not reach full speed before needing to slow down again
    float accelDecelTime = getDurationOfAcceleration( si, stepsAbs / 2 );
    si.estTimeForMove = 2 * accelDecelTime;
  }
}

volatile byte nextStepperFlag = 0;

void setNextInterruptInterval() {

  bool movementComplete = true;

  unsigned long mind = 999999;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di < mind ) {
      mind = steppers[i].di;
    }
  }

  nextStepperFlag = 0;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ! steppers[i].movementDone )
      movementComplete = false;
    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di == mind )
      nextStepperFlag |= (1 << i);
  }

  if ( remainingSteppersFlag == 0 ) {
    TIMER1_INTERRUPTS_OFF
    OCR1A = 65500;
  }

  OCR1A = mind;
}

ISR(TIMER1_COMPA_vect)
{
  unsigned int tmpCtr = OCR1A;

  OCR1A = 65500;

  for (int i = 0; i < NUM_STEPPERS; i++) {

    if ( ! ((1 << i) & remainingSteppersFlag) )
      continue;

    if ( ! (nextStepperFlag & (1 << i)) ) {
      steppers[i].di -= tmpCtr;
      continue;
    }

    volatile stepperInfo& s = steppers[i];

    if ( s.stepCount < s.totalSteps ) {
      s.stepFunc();
      s.stepCount++;
      s.stepPosition += s.dir;
      if ( s.stepCount >= s.totalSteps ) {
        s.movementDone = true;
        remainingSteppersFlag &= ~(1 << i);
      }
    }

    if ( s.rampUpStepCount == 0 ) {
      s.n++;
      s.d = s.d - (2 * s.d) / (4 * s.n + 1);
      if ( s.d <= s.minStepInterval ) {
        s.d = s.minStepInterval;
        s.rampUpStepCount = s.stepCount;
      }
      if ( s.stepCount >= s.totalSteps / 2 ) {
        s.rampUpStepCount = s.stepCount;
      }
      s.rampUpStepTime += s.d;
    }
    else if ( s.stepCount >= s.totalSteps - s.rampUpStepCount ) {
      s.d = (s.d * (4 * s.n + 1)) / (4 * s.n + 1 - 2);
      s.n--;
    }

    s.di = s.d * s.speedScale; // integer
  }

  setNextInterruptInterval();

  TCNT1  = 0;
}


void runAndWait() {
  adjustSpeedScales();
  setNextInterruptInterval();
  TIMER1_INTERRUPTS_ON
  while ( remainingSteppersFlag );
  remainingSteppersFlag = 0;
  nextStepperFlag = 0;
}

void adjustSpeedScales() {
  float maxTime = 0;
  
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ! ((1 << i) & remainingSteppersFlag) )
      continue;
    if ( steppers[i].estTimeForMove > maxTime )
      maxTime = steppers[i].estTimeForMove;
  }

  if ( maxTime != 0 ) {
    for (int i = 0; i < NUM_STEPPERS; i++) {
      if ( ! ( (1 << i) & remainingSteppersFlag) )
        continue;
      steppers[i].speedScale = maxTime / steppers[i].estTimeForMove;
    }
  }
}

void MoveCM(float distanceToMove, char Axis){
  int stepsRequired = 0;
  switch (Axis) {
    case 'x':
      stepsRequired = distanceToMove * xStepsPerCM;
      prepareMovement( 0, stepsRequired );
      break;
    case 'y':
      stepsRequired = distanceToMove * yStepsPerCM;
      prepareMovement( 1, stepsRequired );
      break;
    default:  break;
  }
}

void loop() {

  //for (int i = 0; i < NUM_STEPPERS; i++) {
  //  prepareMovement( i, 2048 );
  //  runAndWait();
  //}
  //for (int i = 0; i < NUM_STEPPERS; i++) {
  //  prepareMovement( i, -2048 );
  //  runAndWait();
  //}

  //prepareMovement( 0, 4096 );
  //prepareMovement( 1, 1024 );
  //runAndWait();

  //delay(1000);

  //prepareMovement( 0, -4096 );
  //prepareMovement( 1, -2048 );
  //runAndWait();

  delay(5000);

  // Start at (0,0)
  // Move to Square
  MoveCM(100, 'x');  MoveCM(25, 'y');  runAndWait();
  delay(5000);

  // Square
  MoveCM(-50, 'x');  runAndWait();
  MoveCM(-50, 'y');  runAndWait();
  MoveCM(50, 'x');  runAndWait();
  MoveCM(50, 'y');  runAndWait();
  delay(5000);
  
  // Move to Diamond
  MoveCM(-150, 'x');  MoveCM(-25, 'y');  runAndWait();
  delay(5000);

  // Dimond
  MoveCM(-25, 'x');  MoveCM(25, 'y');  runAndWait();
  MoveCM(-25, 'x');  MoveCM(-25, 'y');  runAndWait();
  MoveCM(25, 'x');  MoveCM(-25, 'y');  runAndWait();
  MoveCM(25, 'x');  MoveCM(25, 'y');  runAndWait();
  delay(5000);

  // Move to Triangle
  MoveCM(-125, 'y');  runAndWait();
  delay(5000);

  // Triangle
  MoveCM(100, 'x');  runAndWait();
  MoveCM(-50, 'x');  MoveCM(50, 'y');  runAndWait();
  MoveCM(-50, 'x');  MoveCM(-50, 'y');  runAndWait();
  delay(5000);

  // Move to Octagon
  MoveCM(50, 'x');
  MoveCM(150, 'y');
  runAndWait();
  delay(5000);

  // Octagon
  MoveCM(25, 'x');  MoveCM(25, 'y');  runAndWait();
  MoveCM(25, 'y');  runAndWait();
  MoveCM(-25, 'x');  MoveCM(25, 'y');  runAndWait();
  MoveCM(-25, 'x');  runAndWait();
  MoveCM(-25, 'x');  MoveCM(-25, 'y');  runAndWait();
  MoveCM(-25, 'y');  runAndWait();
  MoveCM(25, 'x');  MoveCM(-25, 'y');  runAndWait();
  MoveCM(25, 'x');  runAndWait();
  delay(5000);

  // Move to Polyline
  MoveCM(-25, 'y');  runAndWait();
  delay(5000);

  // Polyline
  MoveCM(75, 'x');  MoveCM(-75, 'y');  runAndWait();
  MoveCM(-25, 'y');  runAndWait();
  MoveCM(-75, 'x');  MoveCM(75, 'y');  runAndWait();
  MoveCM(-75, 'x');  MoveCM(-75, 'y');  runAndWait();
  MoveCM(-25, 'x');  MoveCM(25, 'y');  runAndWait();
  MoveCM(75, 'x');  MoveCM(75, 'y');  runAndWait();
  MoveCM(25, 'x');  runAndWait();
  delay(5000);

  // Move to Unity Logo
  MoveCM(-100, 'x');  MoveCM(150, 'y');  runAndWait();
  delay(5000);

  //Unity Logo
  MoveCM(-24, 'x');  MoveCM(92, 'y');  runAndWait();
  MoveCM(-26, 'x');  MoveCM(-26, 'y');  runAndWait();
  MoveCM(9, 'x');  MoveCM(-39, 'y');  runAndWait();
  MoveCM(-47, 'x');  MoveCM(12, 'y');  runAndWait();
  MoveCM(-12, 'x');  MoveCM(48, 'y');  runAndWait();
  MoveCM(39, 'x');  MoveCM(-10, 'y');  runAndWait();
  MoveCM(25, 'x');  MoveCM(26, 'y');  runAndWait();
  MoveCM(-91, 'x');  MoveCM(24, 'y');  runAndWait();
  MoveCM(26, 'x');  MoveCM(-101, 'y');  runAndWait();
  MoveCM(101, 'x');  MoveCM(-26, 'y');  runAndWait();
  MoveCM(-68, 'x');  MoveCM(-68, 'y');  runAndWait();
  MoveCM(-8, 'x');  MoveCM(35, 'y');  runAndWait();
  MoveCM(27, 'x');  MoveCM(29, 'y');  runAndWait();
  MoveCM(-46, 'x');  MoveCM(13, 'y');  runAndWait();
  MoveCM(-35, 'x');  MoveCM(-35, 'y');  runAndWait();
  MoveCM(38, 'x');  MoveCM(-11, 'y');  runAndWait();
  MoveCM(9, 'x');  MoveCM(-34, 'y');  runAndWait();
  MoveCM(-92, 'x');  MoveCM(23, 'y');  runAndWait();
  MoveCM(74, 'x');  MoveCM(74, 'y');  runAndWait();
  MoveCM(-74, 'x');  MoveCM(-74, 'y');  runAndWait();
  MoveCM(-24, 'x');  MoveCM(92, 'y');  runAndWait();
  MoveCM(35, 'x');  MoveCM(-9, 'y');  runAndWait();
  MoveCM(10, 'x');  MoveCM(-38, 'y');  runAndWait();
  MoveCM(36, 'x');  MoveCM(34, 'y');  runAndWait();
  MoveCM(-13, 'x');  MoveCM(47, 'y');  runAndWait();
  MoveCM(-29, 'x');  MoveCM(-28, 'y');  runAndWait();
  MoveCM(-34, 'x');  MoveCM(10, 'y');  runAndWait();
  MoveCM(67, 'x');  MoveCM(67, 'y');  runAndWait();
  delay(5000);

  // Move Away
  MoveCM(330, 'x');  MoveCM(20, 'y');  runAndWait();
  delay(5000);

  while (true);

}