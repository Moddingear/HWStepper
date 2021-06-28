#include <Arduino.h>
#include "HWStepper.h"

//Dir on pin 5
//Step on pin 16

const int VDIR = 17, VPUL = 4, DIR = 5, PUL = 16; //I have two pins always high (VDIR and VPUL) to power the optocouplers, these aren't needed with other drivers

const float startspeed = 100; //pulses/s
const float maxspeed = 20e3*2; //pulses/s, this is the max speed allowed by the stepper driver i'm currently using (TB6600)
const float maxacc = 50000; //pulses/s^2
const float duty = 0.3;

void setup() 
{
	Serial.begin(115200);
	pinMode(VDIR, OUTPUT);
	pinMode(VPUL, OUTPUT);
	pinMode(DIR, OUTPUT);
	pinMode(PUL, OUTPUT);
	digitalWrite(VDIR, HIGH);
	digitalWrite(VPUL, HIGH);
	AddStepper(PUL, DIR);
	SetParameters(0, maxacc, maxacc*2, maxspeed, startspeed, duty);
	SetTarget(0, 16*200*10); //Do 10 turns
	InitHWStepper();
	StartHWStepper();
}

bool targetfar = false;

void loop() {
	// put your main code here, to run repeatedly:
	//FillDelay();
	TickHWStepper();
	if (IsDone(0)) //When the stepper is done, go back the other way
	{
		SetTarget(0, targetfar ? 16*200*10 : 0);
		targetfar = !targetfar;
	}
	
}