/*
  Run a syringe in/out using the stepper, recognizes limit switches.
  Reset button: retract and halt
  Start button: start push/pull patterns
    OR stop if running
  Limit switches actively stop it jamming at ends

  Wiring: see associated fritzing
  Switches are momentary, normally-open, close to ground

  On power-up, this should Retract to the limit switch,
  Then push to the limit switch.

  The Reset button should retract.

  What to do if directions are backwards:
    Flip the "coils" of the wiring.

  accelstepper:
    setSpeed() and runSpeed(), no accel, just step @ speed
    runSpeedToPosition() does not work right
    setMaxSpeed(), moveTo(), setAcceleration() and run(), step to target w/accel
    runTo* blocking steps till

    was
    speed max 7000 at 1ms delay = 1000
      actually, anything over about 1000 (delay <= 7) would stutter
      liked the stutter when speed is too high ( > 1000)

    
*/

#include <limits.h>
#include <Bounce2.h>
#include <AccelStepper.h>

// Wiring
const int stepsPerRevolution = 200; // for the stepper
//const int stepsPerRevolution = 20; // ## alan's test

// You can use an Adafruit motorshield...
#define StepAndDir 1 // Kurina's hardware: A step-pin, a dir-pin
#define MotorShieldV2 2 // Testing with MotorShieldV2
// Pick motor-controller: ##
#define AccelType StepAndDir


#if AccelType == StepAndDir
// Kurina's hardware
AccelStepper stepper = AccelStepper(
                         AccelStepper::DRIVER, // 2 pin (set to pinmode OUTPUT automatically) :
                         3, // stepPin
                         4 // dirPin
                       );

#elif AccelType == MotorShieldV2
// for testing w/a motorshield
#include "AccelStepperMotorShield.h"
Adafruit_MotorShield syringeShield = Adafruit_MotorShield(); // shield #1

AccelStepperMotorShield stepper = AccelStepperMotorShield( // is a AccelStepper
                                    syringeShield,
                                    2 // stepper block #2
                                  );
#else
static_assert(false, "Expected the value of AccelType to have on an #elif block");
#endif

const int startPin = 5;
const int resetPin = 2;

const int retractLimitPin = 6; // momentary limit switch, when syringe fully retracted
const int pushLimitPin = 7; // momentary limit switch, when syringe fully depressed
const int debounceInterval = 50;

//////////////// variables to be modified as you wish later on /////////
const int MAXSPEED = stepsPerRevolution * 5; // i.e. 5 rev/sec
const int MINSPEED = stepsPerRevolution * .6;
const int NonAnimatedSpeed = stepsPerRevolution * 2; // reasonable reset/find speed

const int MINSTEP = stepsPerRevolution / 20;         // Smallest amount of motion each time
const int MAXSTEP = stepsPerRevolution * 13;   //200 is a full rotation , 10 to make 10 rotations

const int JerkPercent = 5; // will jerk for some period of time this often
const int JerkSpeed = MAXSPEED / 2;

int maxPosition = 2400;      // when syringe fully depressed. initial value is a guess. automatically updated by limit switch

/////////////////////// end of variables////////////////

Bounce startButton = Bounce();
Bounce pushLimitButton = Bounce();
Bounce retractLimitButton = Bounce();

volatile bool isStarted = false;
volatile bool isReset = false;
int originalPos = 0;
int dir = 1;
long randSteps;
long randSpeed;
long randDir = 0;
volatile int stepsTaken ;

// sgn(number) gives -1,0,1
template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("test, then Simplify, use accelstepper's position, moveto, etc."));

  pinMode(startPin, INPUT_PULLUP);
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(retractLimitPin, INPUT_PULLUP);
  pinMode(pushLimitPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(resetPin), ResetFunc, FALLING );

  startButton.attach(startPin);
  startButton.interval(debounceInterval);

  pushLimitButton.attach(pushLimitPin);
  startButton.interval(debounceInterval);
  retractLimitButton.attach(retractLimitPin);
  retractLimitButton.interval(debounceInterval);

#if AccelType == MotorShieldV2
  stepper.begin(); // because you can't Adafruit_MotorShield.begin() in a constructor
#endif
  stepper.setMaxSpeed(MAXSPEED * 7); // way above useful max, so "jerk" works

  // stepper.currentPosition() starts at 0
  // we'll use that as our position, and update it when we hit a limit
  // and use it to tell how far to go

  randomSeed(analogRead(0));
  stepsTaken = 0;

  find_limits();

  Serial.println(F("Waiting, press start."));
}

void find_limits() {
  // Figure out position limits

  // run to both ends, which also sets the distance
  // in this order, because we set min to 0
  Serial.println(F("Looking for min push limit..."));
  stepper.setCurrentPosition( INT_MAX / 4 ); // so we can retract
  move_to( INT_MIN / 4, -NonAnimatedSpeed); // will figure out 0 for us, way farther negative than possible
  Serial.print(F("min retract limit ")); Serial.println(stepper.currentPosition());

  Serial.println(F("Looking for max push limit..."));
  move_to( INT_MAX / 4, NonAnimatedSpeed); // will figure out maxPosition for us, way farther than possible
  Serial.print(F("max push limit ")); Serial.println(stepper.currentPosition());
}

void ResetFunc() {
  isStarted = false;
  isReset = true;
}

boolean hit_limit() {
  // signal if either limit switch was hit
  // update the start/end position values
  // Needs to be relatively fast because we are in the stepper loop

  pushLimitButton.update();
  retractLimitButton.update();
  startButton.update();

  if ( retractLimitButton.fell() )
  {
    stepper.setCurrentPosition(0);
    Serial.println(F("zeroed"));
    return true;
  }
  else if ( pushLimitButton.fell() )
  {
    maxPosition = stepper.currentPosition();
    Serial.print(F("max pos ")); Serial.println(maxPosition);
    return true;
  }
  else if (startButton.fell()) {
    Serial.println(F("STOP"));
    isStarted = false;
    isReset = false;
    return true;
  }

  return false;
}

void loop() {

  if (!isStarted)
  {
    startButton.update();

    // on push, mark started
    if (startButton.fell())
    {
      Serial.print(F("Start from ")); Serial.println(stepper.currentPosition());
      isStarted = true;
      isReset = false;
    }

    // watch for reset
    else if (isReset)
    {
      isReset = false;
      isStarted = false;

      // retract
      Serial.print(F("Reset/Retracting from ")); Serial.println(stepper.currentPosition());
      Serial.print(F("To ")); Serial.println(INT_MIN / 4);
      move_to(INT_MIN / 4, -NonAnimatedSpeed);
    }
  }

  // running

  else
  {

    // wants some variability


    if ( random(0, 100) <= JerkPercent) {
      // jerk around explicitly
      
      for (int i = 0; i < random(MINSTEP, MAXSTEP ) / 3; i++) {
        move_to( stepper.currentPosition() + random(1, 3), JerkSpeed );
        move_to( stepper.currentPosition() - random(1, 3), -JerkSpeed );
        if (!isStarted) break; // need to quit jerking!
      }
    }
    else {
      // just move

      randSpeed = random(MINSPEED, MAXSPEED);
      /* if ( random(0, 100) <= JerkPercent) {
        // we abuse the motor (try to run too fast) to make jerky motion
        randSpeed = random(MAXSPEED * 1.2, MAXSPEED * 7);
        } */

      // decide which way to go, how far, and how fast
      // We want a big distance (between MINSTEP & MAXSTEP),
      // tending towards push (3/4 of the time)
      int target;

      // guess a distance and direction...
      randSteps = random(MINSTEP, MAXSTEP ); // the goal distance
      randDir = random(-1, 2); // -1=retract, 0,1=push x1, 2=push x2
      if (randDir == 0) randDir++;
      // try again if that would retract too far (<originalPos)
      // or push too far (>maxPosition)
      target = stepper.currentPosition() + randSteps * randDir;
      target = constrain( target, -stepsPerRevolution / 4, maxPosition + (stepsPerRevolution / 4) ); // "+/-" so it reliably hits the end even w/slippage

      // it might be possible to smooth out slow speeds with microsteps
      // you'd have to multiple the distance and speed by number-of-microsteps
      // but only when the randSpeed < 1000/microsteps

      move_to( target, randSpeed * sgn(randDir));
    }
  }
}

void move_to( int target, int speed ) {
  // run the motor to the target, watching for reset, limits, etc.
  // remember, a "reset" could during here too

  stepper.setSpeed( speed );
  int to_go = target - stepper.currentPosition();

  if ( to_go != 0 && sgn(speed) != sgn(to_go) ) {
    // this means a bug: will run off the "end" (one of the ends, oh well!)
    Serial.print(F("   direction ")); Serial.print(speed);
    Serial.print(F(" delta-pos ")); Serial.print( to_go );
    Serial.println();
  }

  // ensure we have distance to go (in the direction of the speed)
  while (!hit_limit() && !isReset && sgn(to_go) == sgn(speed))
  {
    /*
      if (! ( stepper.currentPosition() % 10 ) ) {
      Serial.print(stepper.currentPosition()); Serial.print(F("->")); Serial.print(target);
      Serial.print(F(" +")); Serial.print(speed);
      Serial.println();
      }
    */

    if (stepper.runSpeed()) { // possibly take a step
      to_go = target - stepper.currentPosition();
    }
  }
}
