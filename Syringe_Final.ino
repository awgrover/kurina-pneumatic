/*
  Run a syringe in/out using the stepper.
  Reset button: retract and halt
  Start button: start push/pull patterns


*/

#include <limits.h>
#include <Bounce2.h>
#include <AccelStepper.h>

// Wiring
const int stepPin = 3;
const int dirPin = 4;
const int motorInterfaceType = 1;

const int startPin = 5; // momentary
const int resetPin = 2; // momentary

const int retractLimitPin = 6; // momentary limit switch, when syringe fully retracted
const int pushLimitPin = 7; // momentary limit switch, when syringe fully depressed

//////////////// variables to be modified as you wish later on /////////
const int MAXSPEED = 7000;
const int MINSPEED = 5000;

const int MINSTEP = 10;         // Smallest amount of motion each time
const int MAXSTEP = 50 * 52;   //200 is a full rotation , 10 to make 10 rotations

const int MAXDELAY = 40;    // in ms . 2000 = 2 secs
const int MINDELAY = 1;

int maxPosition = 2400;      // when syringe fully depressed. initial value is a guess. automatically updated by limit switch

/////////////////////// end of variables////////////////

Bounce startButton = Bounce();
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
volatile bool isStarted = false;
volatile bool isReset = false;
int originalPos = 0;
int dir = 1;
long randSteps;
long randDelay;
long randSpeed;
long randDir = 0;
volatile int stepsTaken ;

// sgn(number) gives -1,0,1
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
    }

void setup() {
  pinMode(startPin, INPUT_PULLUP);
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(retractLimitPin, INPUT_PULLUP);
  pinMode(pushLimitPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(resetPin), ResetFunc, FALLING );

  Serial.begin(9600);

  startButton.attach(startPin);
  startButton.interval(50);

  pinMode (stepPin, OUTPUT);
  pinMode (dirPin, OUTPUT);
  stepper.setMaxSpeed(MAXSPEED);

  // stepper.currentPosition() starts at 0
  // we'll use that as our position, and update it when we hit a limit
  // and use it to tell how far to go

  randomSeed(analogRead(0));
  stepsTaken = 0;

  // run to both ends, which also sets the distance
  // in this order, because we set min to 0
  move_to( INT_MIN, MINSPEED, 0 ); // will figure out 0 for us, way farther negative than possible
  move_to( INT_MAX, MINSPEED, 0 ); // will figure out maxPosition for us, way farther than possible
}

void ResetFunc() {
  isStarted = false;
  isReset = true;
}

boolean hit_limit() {
  // signal if either limit switch was hit
  // update the start/end position values
  // Needs to be relatively fast because we are in the stepper loop

  if ( digitalRead(retractLimitPin) ) 
  {
    stepper.setCurrentPosition(0);
    ResetFunc(); // i.e. stop
    Serial.println(F("zeroed"));
    return true;
    }
  else if ( digitalRead(pushLimitPin) ) 
  {
    ResetFunc(); // i.e. stop
    maxPosition = stepper.currentPosition();
    Serial.print(F("max pos"));Serial.println(maxPosition);
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
      isStarted = true;
    }

    // watch for reset
    else if (isReset)
    {
      isReset = false;

      // retract
      move_to(INT_MIN, MINSPEED, 0); // was MAXSPEED
    }
  }

  // running
  else
  {
    // wants some variability
    randDelay = random(MINDELAY, MAXDELAY); // stutter possibly
    randSpeed = random(MINSPEED, MAXSPEED);

    // decide which way to go, how far, and how fast
    // We want a big step (between MINSTEP & MAXSTEP),
    // tending towards push (3/4 of the time)
    int target;
    do {
      // guess a distance and direction...
      randSteps = random(MINSTEP, MAXSTEP ); // the goal distance
      randDir = random(-1, 2); // -1=retract, 0,1=push x1, 2=push x2
      if (randDir == 0) randDir++;
      // try again if that would retract too far (<originalPos)
      // or push too far (>maxPosition)
      target = stepper.currentPosition() + randSteps * randDir;
    } while ( target < 0 || target > (maxPosition + 100) ); // "+100" so it reliably hits the end even w/slippage

    move_to( target, randSpeed * sgn(randDir), randDelay );
  }

}

void move_to( int target, int speed, int stutter_delay ) {
  // run the motor to the target, watching for reset, limits, etc.
  // stutter_delay is meant to introduce stuttering while running
  // remember, a "reset" could during here too

  stepper.setSpeed( speed );

  if ( sgn(speed) != sgn(stepper.currentPosition() - target) ) {
    Serial.print(F("Bad speed direction "));Serial.print(speed);
    Serial.print(F(" delta-pos "));Serial.print( stepper.currentPosition() - target );
    Serial.println();
    // will run off the "end" (one of the ends, oh well!)
    }

  // ( "!= target" ought to be safe, as runSpeed() should only be able to take one step at a time )
  while (!hit_limit() && isStarted && stepper.currentPosition() != target) 
  {
    stepper.runSpeed(); // possibly take a step
    delay (randDelay); // may stutter
  }
}
