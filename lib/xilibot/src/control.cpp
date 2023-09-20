// BROBOT EVO 2 by JJROBOTS
// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS
// License: GPL v2
// Control functions (PID controls, Steppers control...)

#include "defines.hpp"

// Default control values from constant definitions
float Kp = KP;
float Kd = KD;
float Kp_thr = KP_THROTTLE;
float Ki_thr = KI_THROTTLE;
float Kp_user = KP;
float Kd_user = KD;
float Kp_thr_user = KP_THROTTLE;
float Ki_thr_user = KI_THROTTLE;
float Kp_position = KP_POSITION;
float Kd_position = KD_POSITION;

// position control
volatile int32_t steps1;
volatile int32_t steps2;
int32_t target_steps1;
int32_t target_steps2;

volatile uint16_t step_count_M1 = 0;
volatile uint16_t step_count_M2 = 0;
int16_t step_max_M1 = 0;
int16_t step_max_M2 = 0;

// Actual speed of motors
int16_t speed_M1, speed_M2;

// Actual direction of steppers motors
int8_t dir_M1 = 0;
int8_t dir_M2 = 0;

float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;

// Timer1 ccount reached
void IRAM_ATTR timer1_isr()
{
    // Always set to low both pulses
    GPOC = (1 << STEP_M1_PIN) | (1 << STEP_M2_PIN);

    if (step_count_M1 > 0) {
        step_count_M1--;
    } else {
        if (dir_M1 != 0) {
          GPOS = (1 << STEP_M1_PIN);
          steps1 -= dir_M1;
        }
        step_count_M1 += step_max_M1;
    }
    if (step_count_M2 > 0) {
        step_count_M2--;
    } else {
        if (dir_M2 != 0) {
          GPOS = (1 << STEP_M2_PIN);
          steps2 -= dir_M2;
        }
        step_count_M2 += step_max_M2;
    }
}

void initStepperMotors()
{
    // Initialize timer1
    timer1_disable();
    timer1_attachInterrupt(timer1_isr);
    timer1_write(55);   //55 = 10KHz,  111 = 5KHz (with MSx=101)
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
}


// Set speed of Stepper Motor1 (tspeed could be positive or negative for reverse)
void setMotorSpeedM1(int16_t tspeed)
{
    long timer_period;
    int16_t speed;

    // WE LIMIT MAX ACCELERATION of the motors
    if ((speed_M1 - tspeed) > MAX_ACCEL)
        speed_M1 -= MAX_ACCEL;
    else if ((speed_M1 - tspeed) < -MAX_ACCEL)
        speed_M1 += MAX_ACCEL;
    else
        speed_M1 = tspeed;

    speed = speed_M1 * 26; // Adjust factor from control output speed to real motor speed in steps/second

    if (speed == 0)
    {
        timer_period = ZERO_SPEED;
        dir_M1 = 0;
    }
    else if (speed > 0)
    {
        timer_period = 100000 / speed;
        dir_M1 = 1;
        GPOS = (1 << DIR_M1_PIN);
    }
    else
    {
        timer_period = 100000 / -speed;
        dir_M1 = -1;
        GPOC = (1 << DIR_M1_PIN);
    }
    if (timer_period > 65535)   // Check for minimum speed (maximum period without overflow)
        timer_period = ZERO_SPEED;

    step_max_M1 = timer_period;
    step_count_M1 = timer_period;
/*
#ifdef DEBUG
    Serial.printf("step_max_M1 & step_count_M1 = %d\n", timer_period);
#endif
*/
}

// Set speed of Stepper Motor2 (tspeed could be positive or negative for reverse)
void setMotorSpeedM2(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M2 - tspeed) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = tspeed;

  speed = speed_M2 * 26; // Adjust factor from control output speed to real motor speed in steps/second

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M2 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 100000 / speed;
    dir_M2 = 1;
    GPOS = (1 << DIR_M2_PIN);
  }
  else
  {
    timer_period = 100000 / -speed;
    dir_M2 = -1;
    GPOC = (1 << DIR_M2_PIN);
  }
  if (timer_period > 65535)   // Check for minimum speed (maximum period without overflow)
    timer_period = ZERO_SPEED;

  step_max_M2 = timer_period;
  step_count_M2 = timer_period;
/*
#ifdef DEBUG
    Serial.printf("step_max_M2 & step_count_M2 = %d\n", timer_period);
#endif
*/
}

float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM)
{
  float output;
  float P;

  P = constrain(Kpp * float(setPointPos - actualPos), -115, 115); // Limit command
  output = P + Kdp * float(speedM);
  return (output);
}

// PI controller implementation (Proportional, integral). DT in seconds
float speedPIControl(float DT, int16_t input, int16_t setPoint,  float Kp, float Ki)
{
  int16_t error;
  float output;

  error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

  //Serial.println(PID_errorSum);

  output = Kp * error + Ki * PID_errorSum * DT; // DT is in miliseconds...
  return (output);
}

// PD controller implementation(Proportional, derivative). DT in seconds
float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint - input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-1).
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  float Kd_setPoint = constrain((setPoint - setPointOld), -8, 8); // We limit the input part...
  output = Kp * error + (Kd * Kd_setPoint - Kd * (input - PID_errorOld)) / DT;
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  //PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;  // error for Kd is only the input component
  setPointOld = setPoint;
  return (output);
}

void setDefaultControlGains()
{
    Kp = Kp_user;            // Default user control gains
		Kd = Kd_user;
		Kp_thr = Kp_thr_user;
		Ki_thr = Ki_thr_user;
}

void setRaiseUpControlGains()
{
    Kp = KP_RAISEUP;         // Control gains for raising up
		Kd = KD_RAISEUP;
		Kp_thr = KP_THROTTLE_RAISEUP;
		Ki_thr = KI_THROTTLE_RAISEUP;
}