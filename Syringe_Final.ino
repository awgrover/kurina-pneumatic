/*
  Run a syringe in/out using the stepper.
  Reset button: retract and halt
  Start button:
*/

#include <Bounce2.h>
#include <AccelStepper.h>

const int stepPin = 3;
const int dirPin = 4;
const int motorInterfaceType = 1;

const int startPin = 5; // momentary
const int resetPin = 2; // momentary

//////////////// variables to be modified as you wish later on /////////
const int MAXSPEED = 7000;
const int MINSPEED = 5000;

const int MINSTEP = 10;         // just a random min step .
const int MAXSTEP = 50 * 52;   //200 is a full rotation , 10 to make 10 rotations

const int MAXDELAY = 40;    // in ms . 2000 = 2 secs
const int MINDELAY = 1;

const int STEPSLIMIT = 2400;      // a limit to no exceed in movment

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

void setup() {
  // put your setup code here, to run once:
  pinMode(startPin, INPUT_PULLUP);
  pinMode(resetPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(resetPin), ResetFunc, FALLING );

  Serial.begin(9600);

  startButton.attach(startPin);
  startButton.interval(50);

  pinMode (stepPin, OUTPUT);
  pinMode (dirPin, OUTPUT);
  stepper.setMaxSpeed(MAXSPEED);
  originalPos = stepper.currentPosition() ; // isn't that just 0?

  randomSeed(analogRead(0));
  stepsTaken = 0;
}

void ResetFunc() {

  isStarted = false;
  isReset = true;

}

void loop() {

  // put your main code here, to run repeatedly:
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
      while (stepsTaken > originalPos)
      {
        stepper.setSpeed(-MAXSPEED);
        stepper.runSpeed();
        delay(10);
        stepsTaken--;
        Serial.println(stepsTaken);

      }
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
    do {
      // guess a distance and direction...
      randSteps = random(MINSTEP, MAXSTEP); // the goal distance
      randDir = random(-1, 2); // -1=retract, 0,1=push x1, 2=push x2
      if (randDir == 0) randDir++;
      // try again if that would retract too far (<originalPos)
      // or push too far (>STEPSLIMIT)
    } while ((stepsTaken + (randSteps * randDir) < originalPos) || stepsTaken + (randSteps * randDir) > STEPSLIMIT);

    // remember, a "reset" could during here
    // step for the distance
    while  (isStarted && stepper.currentPosition() != ( (randSteps * randDir) ))
    {
      stepper.setSpeed(randSpeed * randDir); // that seems unlikely, already near MAXSPEED?
      stepper.runSpeed();
      delay (randDelay);
      stepsTaken += randDir;

      Serial.println(stepsTaken);
    }
    stepper.setCurrentPosition(0);

  }

}
