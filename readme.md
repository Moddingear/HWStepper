# HWStepper
An esp32 library for high-speed control of stepper motors

## How does it work
It uses a circular buffer that holds delays between pulses that need to be sent to the steppers. An hardware timer then calls an interrupt that executes thoses pulses at the interval they were meant to be called at.
This allows the in-code tick rate to be much lower than that of the stepper motor, while having very precise timing.
Though this also means that the microcontroller is in the future relative to the stepper motor.

 - Support acceleration and deceleration. Actual deceleration value will be closer to the acceleration value and will determine if the motor will need to overshoot or not
 - Non blocking
 - Adjustable buffer size
 - Will run continously for 14613 years (or more, this is when using divider = 2) before overflowing
 - Uses exact acceleration and deceleration formulas
 - Uses direct step input
 - Can control any number of motors
 - Will run faster than your motors allow if needed (40kHz+ achieved without any issues)

> Written with [StackEdit](https://stackedit.io/).