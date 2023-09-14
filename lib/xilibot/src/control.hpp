#ifndef control_h
#define control_h

void initStepperMotors();

// Set speed of Stepper Motor1 (tspeed could be positive or negative for reverse)
void setMotorSpeedM1(int16_t tspeed);

// Set speed of Stepper Motor2 (tspeed could be positive or negative for reverse)
void setMotorSpeedM2(int16_t tspeed);

float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM);

// PI controller implementation (Proportional, integral). DT in seconds
float speedPIControl(float DT, int16_t input, int16_t setPoint,  float Kp, float Ki);

// PD controller implementation(Proportional, derivative). DT in seconds
float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd);

void setDefaultControlGains();

void setRaiseUpControlGains();

#endif