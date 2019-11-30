#include <Bounce2.h>
#include <AccelStepper.h>


const int stepPin = 3;
const int dirPin = 4;
const int startPin = 5;
const int resetPin = 2;
const int motorInterfaceType = 1;



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
  originalPos = stepper.currentPosition() ;
  randomSeed(analogRead(0));
  stepsTaken = 0;
}

void ResetFunc()
{


  isStarted = false;
  isReset = true;


}
void loop() {

  // put your main code here, to run repeatedly:
  startButton.update();

  if (!isStarted)
  {

    if (startButton.fell())
    {

      isStarted = true;
    }
    else if (isReset)
    {
      isReset = false;

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
  else
  {
    randDelay = random(MINDELAY, MAXDELAY);
    randSpeed = random(MINSPEED, MAXSPEED);

    do {

      randSteps = random(MINSTEP, MAXSTEP);
      randDir = random(-1, 2);
      if (randDir == 0)randDir++;
    }    while ((stepsTaken + (randSteps * randDir) < originalPos) || stepsTaken + (randSteps * randDir) > STEPSLIMIT);


    while  (isStarted && stepper.currentPosition() != ( (randSteps * randDir) ))
    {

      stepper.setSpeed(randSpeed * randDir);
      stepper.runSpeed();
      delay (randDelay);
      stepsTaken += randDir;

      Serial.println(stepsTaken);
    }
    stepper.setCurrentPosition(0);


  }


}
