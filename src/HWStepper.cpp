#include "HWStepper.h"
#include <Arduino.h>
#include <soc/soc.h>
#include <driver/timer.h>
#include <esp32-hal-timer.h>

uint16_t Divider = 2;
uint64_t Scale = TIMER_BASE_CLK / Divider;

struct HWTimerInput
{
	uint32_t PinRegister;
	uint32_t mask;
	uint64_t WaitBefore; //time to wait between the pulse before this one and this one
};

struct HWStepperMotorInternal
{
	float MaxAcceleration;
	float MaxDeceleration;
	float StartSpeed;
	float MaxSpeed;

	float Duty;

	float Speed;
	int64_t position;
	int64_t target;

	uint64_t LastTickLow;
	bool HasNextTick = false;
	uint64_t NextTickLow;
	bool StepHigh = false;
	uint64_t NextTickHigh;

	int DirPinState = 0;
	bool ShouldSetDirection = false;

	uint8_t StepPin;
	uint8_t DirPin;

	uint32_t StepMask;
	uint32_t StepSetRegister;
	uint32_t StepClearRegister;

	uint32_t DirMask;
	uint32_t DirSetRegister;
	uint32_t DirClearRegister;

	int64_t DeltaToTarget() //Positive if position neeeds to increase
	{
		return target-position;
	}

	int64_t GetDistanceToTarget()
	{
		return abs(position - target);
	}

	int64_t GetBrakingDistance()
	{
		if (abs(Speed) <= StartSpeed)
		{
			return 0;
		}
		float deltav = abs(Speed) - StartSpeed;
		return ceil((deltav*deltav / 2 + StartSpeed * deltav)/MaxAcceleration);
	}

	float GetNeededDeceleration()
	{
		float v0 = copysign(StartSpeed, Speed);
		return (v0-Speed)*(v0+Speed)/DeltaToTarget()/2.f;
	}

	int GetDirectionToTarget()
	{
		if (position == target)
		{
			return 0;
		}
		return position < target ? 1 : -1;
	}

	int GetCurrentDirection()
	{
		if (abs(Speed) < StartSpeed)
		{
			return 0;
		}
		return Speed > 0 ? 1 : -1;
	}

	bool ShouldAccelerate(int64_t BrakingDistance)
	{
		if (abs(Speed) >= MaxSpeed) // if it's already at max speed
		{
			return false; //can't go faster
		}
		int currdir = GetCurrentDirection();
		int tdir = GetDirectionToTarget();
		if (currdir == 0 && tdir != 0) // if stopped and needs to go
		{
			return true; //start accelerating
		}
		
		if (currdir != tdir && currdir != 0) //if going the wrong way and not stopped
		{
			return false; //don't accelerate
		}
		return (BrakingDistance < GetDistanceToTarget()); //if it takes less distance to slow down than to go to the target
	}

	bool ShouldDecelerate(int64_t BrakingDistance)
	{
		int currdir = GetCurrentDirection();
		if (currdir == 0) //if stopped
		{
			return false;
		}
		if (currdir != GetDirectionToTarget()) //if going the wrong way
		{
			return true; //slow down to stop and go back
		}
		return (BrakingDistance >= GetDistanceToTarget()); //if it takes longer to slow down than to go to the target
	}

	uint64_t GetNextTick()
	{
		if (HasNextTick)
		{
			if (!StepHigh)
			{
				return NextTickLow;
			}
			else
			{
				return NextTickHigh;
			}
		}
		volatile float t = 0;
		volatile int64_t BrakingDistance = GetBrakingDistance();
		if (ShouldAccelerate(BrakingDistance))
		{
			
			if (GetCurrentDirection() == 0)
			{
				Speed = StartSpeed * GetDirectionToTarget();
				t = 1/Speed;
				if (DirPinState != GetCurrentDirection())
				{
					ShouldSetDirection = true;
				}
			}
			int direction = GetCurrentDirection();
			float AccelerationSign = MaxAcceleration * direction;
			t = (-abs(Speed) + sqrtf(Speed * Speed + 2 * MaxAcceleration))/MaxAcceleration;
			Speed = AccelerationSign*t + Speed;
			HasNextTick = true;
		}
		else if (ShouldDecelerate(BrakingDistance))
		{
			volatile float Need;
			if (GetCurrentDirection() == GetDirectionToTarget())
			{
				Need = GetNeededDeceleration();
				if (abs(Need) > MaxDeceleration )
				{
					Need = GetCurrentDirection() * MaxAcceleration * -1.f;
				}
			}
			else
			{
				Need = GetCurrentDirection() * MaxAcceleration * -1.f;
			}
			
			t = (-abs(Speed) + sqrtf(Speed * Speed + 2 * abs(Need)))/abs(Need);
			Speed = Need*t + Speed;

			
			HasNextTick = true;
		}
		else if(GetCurrentDirection() != 0)
		{
			t = 1/MaxSpeed;
			Speed = copysignf(MaxSpeed, Speed);
			HasNextTick = true;
		}
		if (HasNextTick)
		{
			NextTickLow = (uint64_t)(t*Scale) + LastTickLow;
			NextTickHigh = (uint64_t)(t*Scale*(1+Duty)) + LastTickLow;
			return NextTickLow;
		}
		else
		{
			Speed = 0;
			return UINT64_MAX;
		}
	}

	uint64_t ConsumeTick()
	{
		uint64_t NextTick = GetNextTick();
		if (!StepHigh)
		{
			position += GetCurrentDirection();
		}
		if (StepHigh)
		{
			HasNextTick = false;
			LastTickLow = NextTickLow;
		}
		StepHigh = !StepHigh;
		return NextTick;
	}
};

uint8_t NumSteppers;
HWStepperMotorInternal* Steppers;

uint32_t NumTimings = 0;
volatile uint16_t TimerIndex = 0;
uint16_t FillerIndex = 0;
HWTimerInput* Timings = nullptr;
hw_timer_t * HWStepperTimer = NULL;
uint64_t FillerTime = 0;
bool TimerValid = false;
xSemaphoreHandle TimingsSemaphore;

void PinToRegister(const int InPin, uint32_t& OutMask, uint32_t& OutRegisterSet, uint32_t& OutRegisterClear);

void HWStepperTimerISR();
void RestartTimings();

void PinToRegister(const int InPin, uint32_t& OutMask, uint32_t& OutRegisterSet, uint32_t& OutRegisterClear)
{
	if (InPin < 32)
	{
		OutRegisterSet = GPIO_OUT_W1TS_REG;
		OutRegisterClear = GPIO_OUT_W1TC_REG;
		OutMask = 1<<InPin;
	}
	else
	{
		OutRegisterSet = GPIO_OUT1_W1TS_REG;
		OutRegisterClear = GPIO_OUT1_W1TC_REG;
		OutMask = 1<<(InPin-32);
	}
}

void IRAM_ATTR HWStepperTimerISR()
{
	
	REG_WRITE(Timings[TimerIndex].PinRegister, Timings[TimerIndex].mask);
	TimerIndex = (TimerIndex >= NumTimings-1) ? 0 : TimerIndex+1;
	if (Timings[TimerIndex].WaitBefore == UINT64_MAX)
	{
		timerStop(HWStepperTimer);
	}
	else
	{
		uint64_t NextWait = Timings[TimerIndex].WaitBefore;
		timerAlarmWrite(HWStepperTimer, NextWait, true);
		BaseType_t Awoken = pdFALSE;
		xSemaphoreGiveFromISR(TimingsSemaphore, &Awoken);
	}
}

void InitHWStepper(uint32_t InNumTimings, uint16_t InDivider)
{
	NumTimings = InNumTimings;
	TimingsSemaphore = xSemaphoreCreateCounting(NumTimings,NumTimings);
	if (Timings != nullptr)
	{
		free(Timings);
	}
	Divider = InDivider < 2 ? 2 : InDivider;
	Scale = TIMER_BASE_CLK / Divider;
	Timings = (HWTimerInput*)calloc(NumTimings, sizeof(HWTimerInput));
	TickHWStepper();
}

uint8_t AddStepper(uint8_t Step, uint8_t Dir)
{
	if (NumSteppers == 0)
	{
		NumSteppers++;
		Steppers = (HWStepperMotorInternal*)calloc(NumSteppers, sizeof(HWStepperMotorInternal));
	}
	else
	{
		NumSteppers++;
		Steppers = (HWStepperMotorInternal*)realloc(Steppers, NumSteppers * sizeof(HWStepperMotorInternal));
	}
	uint8_t StepperIndex = NumSteppers-1;
	Steppers[StepperIndex].MaxAcceleration = 0;
	Steppers[StepperIndex].MaxDeceleration = 0;
	Steppers[StepperIndex].StartSpeed = 0;
	Steppers[StepperIndex].MaxSpeed = 0;
	Steppers[StepperIndex].Duty = 0.5;
	Steppers[StepperIndex].Speed = 0;
	Steppers[StepperIndex].position = 0;
	Steppers[StepperIndex].target = 0;
	Steppers[StepperIndex].StepPin = Step;
	Steppers[StepperIndex].DirPin = Dir;
	PinToRegister(Step, Steppers[StepperIndex].StepMask, Steppers[StepperIndex].StepSetRegister, Steppers[StepperIndex].StepClearRegister);
	PinToRegister(Dir, Steppers[StepperIndex].DirMask, Steppers[StepperIndex].DirSetRegister, Steppers[StepperIndex].DirClearRegister);
	return StepperIndex;
}

void SetParameters(uint8_t StepperIndex, float InAcceleration, float InDeceleration, float InMaxSpeed, float InStartSpeed, float InDuty)
{
	Steppers[StepperIndex].MaxAcceleration = InAcceleration;
	Steppers[StepperIndex].MaxDeceleration = InDeceleration;
	Steppers[StepperIndex].MaxSpeed = InMaxSpeed;
	Steppers[StepperIndex].StartSpeed = InStartSpeed;
	Steppers[StepperIndex].Duty = InDuty;
}

int64_t GetTarget(uint8_t StepperIndex)
{
	return Steppers[StepperIndex].target;
}

void SetTarget(uint8_t StepperIndex, int64_t InNewTarget)
{
	Steppers[StepperIndex].target = InNewTarget;
	RestartTimings();
}

int64_t GetPosition(uint8_t StepperIndex)
{
	return Steppers[StepperIndex].position;
}

void SetPosition(uint8_t StepperIndex, int64_t InNewPosition)
{
	Steppers[StepperIndex].position = InNewPosition;
	RestartTimings();
}

bool IsDone(uint8_t StepperIndex)
{
	if (Steppers[StepperIndex].GetDirectionToTarget() == 0)
	{
		if (Steppers[StepperIndex].GetCurrentDirection() == 0)
		{
			return true;
		}
	}
	return false;
}

void TickHWStepper()
{
	while (xSemaphoreTake(TimingsSemaphore, 0) == pdTRUE)
	{
		int indexofmin = 0;
		uint64_t nextofmin = UINT64_MAX;
		for (int i = 0; i < NumSteppers; i++)
		{
			uint64_t nextofthis = Steppers[i].GetNextTick();
			if(nextofthis < nextofmin)
			{
				indexofmin = i;
				nextofmin = nextofthis;
			}
		}
		if (nextofmin != UINT64_MAX)
		{
			
			HWTimerInput nextinput;
			if (Steppers[indexofmin].ShouldSetDirection) //Needs a direction pin change
			{
				nextinput.mask = Steppers[indexofmin].DirMask;
				if (Steppers[indexofmin].GetCurrentDirection() == 1)
				{
					nextinput.PinRegister = Steppers[indexofmin].DirSetRegister;
				}
				else
				{
					nextinput.PinRegister = Steppers[indexofmin].DirClearRegister;
				}
				Steppers[indexofmin].DirPinState = Steppers[indexofmin].GetCurrentDirection();
				Steppers[indexofmin].ShouldSetDirection = false;
				nextinput.WaitBefore = (FillerTime + nextofmin)/2 - FillerTime; //Insert the direction change between the next pulse and previous pulse
				FillerTime = (FillerTime + nextofmin)/2;
			}
			else
			{
				nextinput.mask = Steppers[indexofmin].StepMask;
				if (!Steppers[indexofmin].StepHigh)
				{
					nextinput.PinRegister = Steppers[indexofmin].StepSetRegister;
				}
				else
				{
					nextinput.PinRegister = Steppers[indexofmin].StepClearRegister;
				}
				uint64_t NextTime = Steppers[indexofmin].ConsumeTick();
				nextinput.WaitBefore = NextTime - FillerTime;
				FillerTime = NextTime;
			}
			Timings[FillerIndex] = nextinput;
			FillerIndex = (FillerIndex >= NumTimings-1) ? 0 : FillerIndex+1;
		}
		else //If all motors are done
		{
			HWTimerInput nextinput;
			nextinput.mask = 0;
			nextinput.PinRegister = GPIO_OUT_W1TC_REG;
			nextinput.WaitBefore = UINT64_MAX;
			Timings[FillerIndex] = nextinput;
			xSemaphoreGive(TimingsSemaphore);
			break;
		}
	}
}

void StartHWStepper(uint8_t timer)
{
	if (HWStepperTimer == nullptr)
	{
		HWStepperTimer = timerBegin(0, Divider, true);
		timerAttachInterrupt(HWStepperTimer, HWStepperTimerISR, true);
		timerAlarmWrite(HWStepperTimer, Timings[0].WaitBefore, true);
		timerAlarmEnable(HWStepperTimer);
	}
	else
	{
		timerStart(HWStepperTimer);
	}
}

void RestartTimings()
{
	if (HWStepperTimer != nullptr)
	{
		if (timerStarted(HWStepperTimer))
		{
			return;
		}
		for (int i = 0; i < NumSteppers; i++)
		{
			Steppers[i].LastTickLow=0;
		}
		TimerIndex = 0;
		FillerIndex = 0;
		FillerTime = 0;
		vSemaphoreDelete(TimingsSemaphore);
		TimingsSemaphore = xSemaphoreCreateCounting(NumTimings,NumTimings);
		TickHWStepper();
		timerAlarmWrite(HWStepperTimer, Timings[0].WaitBefore, false);
		timerWrite(HWStepperTimer, 0);
		timerStart(HWStepperTimer);
	}
}

void StopHWStepper()
{
	if (HWStepperTimer == nullptr)
	{
		timerStop(HWStepperTimer);
	}
}