/*
  Run a syringe in/out using the stepper, recognizes limit switches.
  Reset button: retract and halt
  Start button: start push/pull patterns

  Wiring: see associated fritzing
  Switches are momentary, normally-open, close to ground

  On power-up, this should Retract to the limit switch,
  Then push to the limit switch.
  
  The Reset button should retract.
  
  What to do if directions are backwards:
  * Flip the "coils" of the wiring.

  accelstepper:
    setSpeed() and runSpeed(), no accel, just step @ speed
    runSpeedToPosition() does not work right   
    setMaxSpeed(), moveTo(), setAcceleration() and run(), step to target w/accel
    runTo* blocking steps till

    was
    speed max 7000 at 1ms delay = 1000
      actually, anything over about 1000 (delay <= 7) would stutter
      she liked the stutter
    speed min 5000, at 40 = ~ 125
    switch to microstepping if speed < 125
*/

#include <limits.h>
#include <Bounce2.h>
#include <AccelStepper.h>

// Wiring
//const int stepsPerRevolution = 200; // for the stepper
const int stepsPerRevolution = 20; // ## alan's test

// You can use an Adafruit motorshield...
#define StepAndDir 1 // Kurina's hardware: A step-pin, a dir-pin
#define MotorShieldV2 2 // Testing with MotorShieldV2
// Pick motor-controller: ##
#define AccelType MotorShieldV2 


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
  stepper.setMaxSpeed(MAXSPEED);

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
  stepper.setCurrentPosition( INT_MAX ); // so we can retract
  move_to( INT_MIN, -NonAnimatedSpeed); // will figure out 0 for us, way farther negative than possible
  Serial.print(F("min retract limit ")); Serial.println(stepper.currentPosition());

  Serial.println(F("Looking for max push limit..."));
  move_to( INT_MAX, NonAnimatedSpeed); // will figure out maxPosition for us, way farther than possible
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
  return false;
}

void loop() {
  startButton.update();

  if (!isStarted)
  {

    // on push, mark started
    if (startButton.fell())
    {
      Serial.print(F("Start from ")); Serial.println(stepper.currentPosition());
      isStarted = true;
    }

    // watch for reset
    else if (isReset)
    {
      isReset = false;
      isStarted = false;

      // retract
      Serial.print(F("Reset/Retracting from ")); Serial.println(stepper.currentPosition());
      Serial.print(F("To "));Serial.println(INT_MIN);
      move_to(INT_MIN, -NonAnimatedSpeed);
    }
  }

  // running
  else
  {
    // wants some variability


    // FIXME: "either moveto something, or spasm";
    
    randSpeed = random(MINSPEED, MAXSPEED);

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
    target = constrain( target, -stepsPerRevolution/4, maxPosition + (stepsPerRevolution/4) );// "+/-" so it reliably hits the end even w/slippage

    // it might be possible to smooth out slow speeds with microsteps
    // you'd have to multiple the distance and speed by number-of-microsteps
    // but only when the randSpeed < 1000/microsteps
    
    move_to( target, randSpeed * sgn(randDir));
  }

}

void move_to( int target, int speed ) {
  // run the motor to the target, watching for reset, limits, etc.
  // remember, a "reset" could during here too

  stepper.setSpeed( speed );

  if ( sgn(speed) != sgn(target - stepper.currentPosition()) ) {
    Serial.print(F("   direction ")); Serial.print(speed);
    Serial.print(F(" delta-pos ")); Serial.print( stepper.currentPosition() - target );
    Serial.println();
    // will run off the "end" (one of the ends, oh well!)
  }

  // ( "!= target" ought to be safe, as runSpeed() should only be able to take one step at a time )
  while (!hit_limit() && !isReset && stepper.currentPosition() != target)
  {
    /*
      if (! ( stepper.currentPosition() % 10 ) ) {
      Serial.print(stepper.currentPosition()); Serial.print(F("->")); Serial.print(target);
      Serial.print(F(" +")); Serial.print(speed);
      Serial.println();
      }
    */

    stepper.runSpeed(); // possibly take a step

  }
}
