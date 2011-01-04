// MultiStepper.pde
// -*- mode: C++ -*-
//
// Shows how to multiple simultaneous steppers
// Runs one stepper forwards and backwards, accelerating and decelerating
// at the limits. Runs other steppers at the same time
//
// Copyright (C) 2009 Mike McCauley
// $Id: HRFMessage.h,v 1.1 2009/08/15 05:32:58 mikem Exp mikem $

#include <AccelStepper.h>

// Define some steppers and the pins the will use
AccelStepper stepper1; // Defaults to 4 pins on 2, 3, 4, 5
AccelStepper stepper2(4, 6, 7, 8, 9);
AccelStepper stepper3(2, 10, 11);

void setup()
{  
    stepper1.setMaxSpeed(200.0);
    stepper1.setAcceleration(100.0);
    stepper1.moveTo(24);
    
    stepper2.setMaxSpeed(300.0);
    stepper2.setAcceleration(100.0);
    stepper2.moveTo(1000000);
    
    stepper3.setMaxSpeed(300.0);
    stepper3.setAcceleration(100.0);
    stepper3.moveTo(1000000); 
}

void loop()
{
    // Change direction at the limits
    if (stepper1.distanceToGo() == 0)
	stepper1.moveTo(-stepper1.currentPosition());
    stepper1.run();
    stepper2.run();
    stepper3.run();
}
