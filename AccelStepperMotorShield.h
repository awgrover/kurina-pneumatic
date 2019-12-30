#pragma once

/* 
  Subcalss AccelStepper to use MotorShield instead of pins:
  Then this works just like a AccelStepper and can be substituted for it.
  
  All the book-keeping that Adafruit_MotorShield/Adafruit_StepperMotor does
  is wasted for this application:
  A "raw" one-step would be much more efficient.
*/

#include <Adafruit_MotorShield.h>

class AccelStepperMotorShield : public AccelStepper {
  public:
  Adafruit_StepperMotor *stepperBlock;
  Adafruit_MotorShield &shield;
     
  static void dumy() {} // nop
  
  AccelStepperMotorShield(Adafruit_MotorShield &shield, const int stepperBlock)
    : shield(shield),
    stepperBlock(shield.getStepper(200, stepperBlock)), // step/rev is not relevant
    // we can't use .forward and .backward 
    // because we can't construct a function pointer for them
    // because it would have to refer to this->stepperBlock
    // but we want as-if forward/backward behavior (aka "custom"):
    AccelStepper(&dumy,&dumy)
    {} // can't call shield.begin() at constructor time
  
  void begin() {
    shield.begin();
    }
    
  void step(long step) {
    // we know we are only doing forward/backward, so just do it
    if (speed() > 0) stepperBlock->onestep(FORWARD, SINGLE);
    else stepperBlock->onestep(BACKWARD, SINGLE);
  }
  void disableOutputs() { stepperBlock->release(); } // translates to "free spin"

};
