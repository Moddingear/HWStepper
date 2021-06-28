#include <Arduino.h>

/*
This plugin works based on a circular buffer containing delays between pulses that is executed at precise intervals using an alarm
This means that the tick frequency can be lower that the pulse frequency of the stepper motor, as long as it is ticked often enough.
But this also means that the code anticipates the movement of the stepper motor, so the code is effectively in the future, relative to the motor.
The bigger the buffer size, the more in the future you are, but also the longer you can wait between ticks.
This plugin also utilizes exact acceleration computations.

Positions and targeets are given in steps
Speed is in steps/s
Acceleration and deceleration is in steps/s/s
Duty is a number between 0 and 1, read the datasheet of your stepper driveer to find the optimal duty cycle

Usage order :
    In setup :
        for each motor:
            AddStepper
            SetParameters
            SetTarget
        InitHWStepper
        StartHWStepper
    In loop :
        TickHWStepper
        and other logic such as setting targets
*/

/*
Creates the instruction buffer and fills it initially
*/
void InitHWStepper(uint32_t InNumTimings = 1024, uint16_t InDivider = 2);


/*
Sets the wanted cinematic parameters for a stepper motor
The deceleration should be larger than the acceleration
Everything should be a positive value
*/
void SetParameters(uint8_t StepperIndex, float InAcceleration, float InDeceleration, float InMaxSpeed, float InStartSpeed, float InDuty);

/*
Registers a new stepper motor.
Returns it's index
*/
uint8_t AddStepper(uint8_t Step, uint8_t Dir);

/*
Gets the target set for the stepper motor
*/
int64_t GetTarget(uint8_t StepperIndex);

/*
Sets a target to reach for the stepper motor
*/
void SetTarget(uint8_t StepperIndex, int64_t InNewTarget);

/*
Returns the position of the stepper motor in the buffer. 
This is not the current position of the motor, as the buffer is in the future
*/
int64_t GetPosition(uint8_t StepperIndex);

/*
Set the position of the stepper motor in the buffer. 
*/
void SetPosition(uint8_t StepperIndex, int64_t InNewPosition);

/*
Returns true if the stepper motor has reached it's target and is stopped
*/
bool IsDone(uint8_t StepperIndex);

/*
Refills the buffer, should be called regularly to avoid it drying out
*/
void TickHWStepper();

/*
Starts the hardware timer, so motors start moving
*/
void StartHWStepper(uint8_t timer = 0);

void StopHWStepper();