// This one takes lots of code from:
// BROBOT EVO 2 by JJROBOTS
// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS
// License: GPL v2

#include <Arduino.h>
#include <Wire.h>
//#include <SPI.h>
#include <defines.hpp>
// OLED display
#ifdef ENABLE_DISPLAY
#include <U8g2lib.h>
#endif
// Color sensor
#ifdef ENABLE_COLOR_SENSOR
#include <Adafruit_TCS34725.h>
#include <color_detection.hpp>
#endif
// Wifi connection
#include <ESP8266WiFi.h>
#include <credentials.hpp>
#include "../gfx/face1.xbm"

extern volatile int32_t steps1;
extern volatile int32_t steps2;
extern int32_t target_steps1;
extern int32_t target_steps2;
extern float PID_errorSum;

extern float Kp_thr;
extern float Ki_thr;
extern float Kp_position;
extern float Kd_position;
extern float Kp;
extern float Kd;

extern int16_t OSCmove_speed;
extern int16_t OSCmove_steps1;
extern int16_t OSCmove_steps2;

extern uint8_t OSCnewMessage;
extern uint8_t OSCpage;
extern uint8_t OSCmove_mode;

extern bool modifing_control_parameters;

// Actual speed of motors
extern int16_t speed_M1, speed_M2;

extern int8_t dir_M1, dir_M2;

extern float angle_offset;
extern float angle_adjusted_filtered;

extern long timer_old;
extern long timer_prev;
extern long timer_value;

// Angle of the robot (used for stability control)
float angle_adjusted;
float angle_adjusted_Old;

float dt;

float estimated_speed_filtered; // Estimated robot speed
int16_t actual_robot_speed; // overall robot speed (measured from steppers speed)

int16_t motor1;
int16_t motor2;

int16_t motor1_control;
int16_t motor2_control;

uint8_t mode = 0; // mode = 0 Normal mode, mode = 1 Pro mode (More aggressive)

boolean positionControlMode = false;

float control_output;
float max_target_angle = MAX_TARGET_ANGLE;

int16_t throttle;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;

float target_angle;

uint8_t cascade_control_loop_counter = 0;
uint8_t loop_counter;		 // To generate a medium loop 40Hz
uint8_t slow_loop_counter;	 // slow loop 2Hz
uint8_t sendBattery_counter; // To send battery status
int16_t BatteryValue;

#ifdef ENABLE_DISPLAY
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#endif

#ifdef ENABLE_COLOR_SENSOR

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

uint8_t     sensorColor;
uint16_t    red, green, blue, clear;
volatile bool sensorReady = false;

// Reads
void getRGB_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
	uint16_t tmp_r, tmp_g, tmp_b, tmp_c;

	GPOS = (1 << TCS_LED_PIN);
	tmp_c = tcs.read16(TCS34725_CDATAL);
	tmp_r = tcs.read16(TCS34725_RDATAL);
	tmp_g = tcs.read16(TCS34725_GDATAL);
	tmp_b = tcs.read16(TCS34725_BDATAL);

	// Avoid divide by zero errors ... if clear = 0 return black
	if (tmp_c == 0) {
		*r = *g = *b = *c = 0;
		return;
	}
	uint16_t sum = tmp_c;

	*r = (float)tmp_r / sum * 255.0;
	*g = (float)tmp_g / sum * 255.0;
	*b = (float)tmp_b / sum * 255.0;
	GPOC = (1 << TCS_LED_PIN);
}

#endif

void setup()
{
	// Turn off the very bright blue led
	GPOS = (1 << LED_BUILTIN);

	Serial.begin(115200);
	Serial.print("Initiating...");

	// Configure pins
	pinMode(DIR_M1_PIN, OUTPUT);			// DIR MOTOR 1
	pinMode(STEP_M1_PIN, OUTPUT);			// STEP MOTOR 1
	pinMode(DIR_M2_PIN, OUTPUT);			// DIR MOTOR 2
	pinMode(STEP_M2_PIN, OUTPUT);			// STEP MOTOR 2
	pinMode(MOTOR_ENABLE_PIN, OUTPUT);		// ENABLE MOTORS PIN AS OUTPUT
	pinMode(TCS_LED_PIN, OUTPUT);			// TCS LED
	pinMode(BUZZER_PIN, OUTPUT);			// BUZZER
	digitalWrite(MOTOR_ENABLE_PIN, HIGH);   // Motors disabled
	digitalWrite(TCS_LED_PIN, LOW);         // TCS LED off

	// Make the buzzer sound
	// do not use tone() because it causes conflicts with timer1 used for the steppers!
	digitalWrite(BUZZER_PIN, HIGH); delay(10); digitalWrite(BUZZER_PIN, LOW);
	delay(10); 
	digitalWrite(BUZZER_PIN, HIGH); delay(10); digitalWrite(BUZZER_PIN, LOW);

	/*if (WiFi.softAP(WIFI_SSID, WIFI_PASS, 1, false, 1)) {
		Serial.print("Access Point is created with SSID: ");
		Serial.println(WIFI_SSID);
		Serial.print("Access Point IP: ");
		Serial.println(WiFi.softAPIP());
	} else {
		Serial.println("Unable to Create Access Point");
	}*/

	WiFi.begin(WIFI_SSID, WIFI_PASS);
	while (WiFi.status() != WL_CONNECTED)
	{
		delay(500);
		Serial.print(".");
	}
	Serial.println(" connected");

	UDP.begin(UDP_PORT);
	Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), UDP_PORT);

	OSC_init();

	// Initialize I2C bus (MPU6050 is connected via I2C)
	// join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.begin();
	Wire.setClock(500000); // 500kHz I2C clock. Comment this line if having compilation difficulties

	Serial.println("XILIBOT");
	delay(200);
	
	Serial.println(F("Initializing I2C devices..."));
#ifdef ENABLE_DISPLAY
	if (!display.begin()) {
		Serial.println(F("SH1106 allocation failed"));
	}
	display.setFont(u8g2_font_6x12_mr);
	// Export as XBM and use this header: const unsigned char face1_bits[] U8X8_PROGMEM = {
	display.drawXBM(0, 0, 128, 64, face1_bits);
	delay(500); // Pause for 1/2 seconds
	display.updateDisplay();
	display.sendBuffer();
#endif
	Serial.println("Don't move!");
	MPU6050_setup();  // setup MPU6050 IMU
	delay(500);

  	// Calibrate MPU gyros
  	MPU6050_calibrate();

	digitalWrite(BUZZER_PIN, HIGH); delay(300); digitalWrite(BUZZER_PIN, LOW); 

#ifdef ENABLE_COLOR_SENSOR
	sensorColor = COLOR_NONE;
	while (!tcs.begin()) {
		Serial.println(F("TCS34725 NOT found"));
    	delay(1000);
	}
	Serial.println(F("Found color sensor"));
#endif

	// STEPPER MOTORS INITIALIZATION
	Serial.println("Steppers init");
	// Enable stepper drivers and TIMER interrupts

	initStepperMotors();

	setMotorSpeedM1(0);               // Motor1 stopped
	dir_M1 = 0;

	setMotorSpeedM2(0);               // Motor2 stopped
	dir_M2 = 0;

	digitalWrite(MOTOR_ENABLE_PIN, LOW);   // Enable motors

  	Serial.println("Start...");
  	timer_old = micros();
	timer_prev = timer_old;
}

void loop()
{
	OSC_MsgRead();  // Read UDP OSC messages
	delay(0);
	if (OSCnewMessage)
	{
		OSCnewMessage = 0;
		if (OSCpage == 1)   // Get commands from user (PAGE1 are user commands: throttle, steering...)
		{
			if (modifing_control_parameters)  // We came from the settings screen
			{
				OSCfader[0] = 0.5; // default neutral values
				OSCfader[1] = 0.5;
				OSCtoggle[0] = 0;  // Normal mode
				mode = 0;
				modifing_control_parameters = false;
			}

			if (OSCmove_mode)
			{
#ifdef DEBUG
				Serial.print("MOVE ");
				Serial.print(OSCmove_speed);
				Serial.print(" ");
				Serial.print(OSCmove_steps1);
				Serial.print(",");
				Serial.println(OSCmove_steps2);
#endif				
				positionControlMode = true;
				OSCmove_mode = false;
#ifdef DEBUG
				Serial.printf("Target steps? %d, %d\n", target_steps1, target_steps2);
#endif
				target_steps1 = steps1 + OSCmove_steps1;
				target_steps2 = steps2 + OSCmove_steps2;
			}
			else
			{
#ifdef DEBUG
				Serial.println("THROTTLE");
#endif
				positionControlMode = false;
				throttle = (OSCfader[0] - 0.5) * max_throttle;
				// We add some exponential on steering to smooth the center band
				steering = OSCfader[1] - 0.5;
				if (steering > 0)
				steering = (steering * steering + 0.5 * steering) * max_steering;
				else
				steering = (-steering * steering + 0.5 * steering) * max_steering;
#ifdef DEBUG
				Serial.print(throttle);
				Serial.print(" ");
				Serial.println(steering);
#endif
			}

			if ((mode == 0) && (OSCtoggle[0]))
			{
#ifdef DEBUG
				Serial.println("MODE 0");
#endif
				// Change to PRO mode
				max_throttle = MAX_THROTTLE_PRO;
				max_steering = MAX_STEERING_PRO;
				max_target_angle = MAX_TARGET_ANGLE_PRO;
				mode = 1;
			}
			if ((mode == 1) && (OSCtoggle[0] == 0))
			{
#ifdef DEBUG
				Serial.println("MODE 1");
#endif
				// Change to NORMAL mode
				max_throttle = MAX_THROTTLE;
				max_steering = MAX_STEERING;
				max_target_angle = MAX_TARGET_ANGLE;
				mode = 0;
			}
		}
		else if (OSCpage == 2) { // OSC page 2
#ifdef DEBUG
				Serial.println("READ PARAMS");
#endif
			// Check for new user control parameters
			readControlParameters();
		}

	} // End new OSC message

	timer_value = micros();

	// New IMU data?
	if (MPU6050_newData())
	{
		MPU6050_read_3axis();
		loop_counter++;
		slow_loop_counter++;
		dt = (timer_value - timer_old) * 0.000001; // dt in seconds
		timer_old = timer_value;

		angle_adjusted_Old = angle_adjusted;
		// Get new orientation angle from IMU (MPU6050)
		float MPU_sensor_angle = MPU6050_getAngle(dt);
		angle_adjusted = MPU_sensor_angle + angle_offset;
		if ((MPU_sensor_angle>-15) && (MPU_sensor_angle<15)) {
			angle_adjusted_filtered = angle_adjusted_filtered*0.99 + MPU_sensor_angle*0.01;
		}
/*	
	#ifdef DEBUG
		Serial.print(dt);
		Serial.print(" ");
		Serial.print(angle_offset);
		Serial.print(" ");
		Serial.print(angle_adjusted);
		Serial.print(",");
		Serial.println(angle_adjusted_filtered);
	#endif

	#ifdef DEBUG
		Serial.printf("Steps1: %d, steps2: %d\n", steps1, steps2);
		//Serial.printf("OSC move_steps1: %d, OSC move_steps2: %d\n", OSCmove_steps1, OSCmove_steps2);
		//Serial.printf("OSC move_steps1: %d, OSC move_steps2: %d\n", OSCmove_steps1, OSCmove_steps2);
	#endif
*/
		// We calculate the estimated robot speed:
		// Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
		actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward  

		int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 25.0; // 25 is an empirical extracted factor to adjust for real units
		int16_t estimated_speed = -actual_robot_speed + angular_velocity;
		estimated_speed_filtered = estimated_speed_filtered * 0.9 + (float)estimated_speed * 0.1; // low pass filter on estimated speed
/*
	#ifdef DEBUG
		Serial.print(angle_adjusted);
		Serial.print(" ");
		Serial.println(estimated_speed_filtered);
	#endif
*/
		if (positionControlMode)
		{
			// POSITION CONTROL. INPUT: Target steps for each motor. Output: motors speed
			motor1_control = positionPDControl(steps1, target_steps1, Kp_position, Kd_position, speed_M1);
			motor2_control = positionPDControl(steps2, target_steps2, Kp_position, Kd_position, speed_M2);

			// Convert from motor position control to throttle / steering commands
			throttle = (motor1_control + motor2_control) / 2;
			throttle = constrain(throttle, -190, 190);
			steering = motor2_control - motor1_control;
			steering = constrain(steering, -50, 50);
		}

		// ROBOT SPEED CONTROL: This is a PI controller.
		//    input:user throttle(robot speed), variable: estimated robot speed, output: target robot angle to get the desired speed
		target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
		target_angle = constrain(target_angle, -max_target_angle, max_target_angle); // limited output

	#if DEBUG==3
		Serial.print(angle_adjusted);
		Serial.print(" ");
		Serial.print(estimated_speed_filtered);
		Serial.print(" ");
		Serial.println(target_angle);
	#endif

		// Stability control (100Hz loop): This is a PD controller.
		//    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
		//    We integrate the output (sum), so the output is really the motor acceleration, not motor speed.
		control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
		control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT); // Limit max output from control

		// The steering part from the user is injected directly to the output
		motor1 = control_output + steering;
		motor2 = control_output - steering;

		// Limit max speed (control output)
		motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
		motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

		int angle_ready;
		if (OSCpush[0])     // If we press the SERVO button we start to move
			angle_ready = 82;
		else
			angle_ready = 74;  // Default angle
		if ((angle_adjusted < angle_ready) && (angle_adjusted > -angle_ready)) // Is robot ready (upright?)
		{
			// NORMAL MODE
			GPOC = (1 << MOTOR_ENABLE_PIN);  // Motors enable
			// NOW we send the commands to the motors
			// NOTE: Change here forward-to-backward and left-to-right movement
			setMotorSpeedM1(motor1);
			setMotorSpeedM2(motor2);
		} else  {  // Robot not ready (flat), angle > angle_ready => ROBOT OFF		
			GPOS = (1 << MOTOR_ENABLE_PIN);  // Disable motors
			setMotorSpeedM1(0);
			setMotorSpeedM2(0);
			PID_errorSum = 0;  // Reset PID I term
			Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
			Kd = KD_RAISEUP;
			Kp_thr = KP_THROTTLE_RAISEUP;
			Ki_thr = KI_THROTTLE_RAISEUP;
			// RESET steps
			steps1 = 0;
			steps2 = 0;
			positionControlMode = false;
			OSCmove_mode = false;
			throttle = 0;
			steering = 0;
		}
		
		// Normal condition?
		if ((angle_adjusted < 56) && (angle_adjusted > -56))
		{
			setDefaultControlGains();
		}
		else    // We are in the raise up procedure => we use special control parameters
		{
			setRaiseUpControlGains();
		}

	}    // End of new IMU 

	// Medium loop 7.5Hz
	if (loop_counter >= 15)
	{
		loop_counter = 0;

	} // End of medium loop
	else if (slow_loop_counter >= 100) // 1Hz
	{
		slow_loop_counter = 0;

#ifdef ENABLE_COLOR_SENSOR
		// Read only when speed < 1, and turn the led on before reading
		if (estimated_speed_filtered < 0.5 && fabs(angle_adjusted) < 20) {
			if (sensorReady) {
				sensorColor = detectColor(red, green, blue);
			} else {
				getRGB_noDelay(&red, &green, &blue, &clear);
			}
			sensorReady = !sensorReady;
		}
#endif
		//Serial.printf("\nTime:\n%d\n", timer_value - timer_prev);
		timer_prev = timer_value;
		//Serial.printf("\nSpeed: %f, Angle: %f\n", estimated_speed_filtered, angle_adjusted);
#ifdef ENABLE_DISPLAY
		//uint16_t max_color = max(max(r, g), b);
		//r = map(r, 0, max_color, 0, 255);
		//g = map(g, 0, max_color, 0, 255);
		//b = map(b, 0, max_color, 0, 255);
		/*display.print("R:");
		display.println(r, DEC);
		display.print("G:");
		display.println(g, DEC);
		display.print("B:");
		display.println(b, DEC);
		//display.println(ColorNameString(r, g, b));
		//display.printf("\nTime:\n%d\n", timer_value - timer_prev);
		display.printf("Temp: %d\n", colorTemp);
		*/
#ifdef ENABLE_COLOR_SENSOR
		const char *colorNames[] = {
			"BLACK",
			"PINK",
			"PURPLE",
			"BLUE",
			"LIGHTBLUE",
			"CYAN",
			"GREEN",
			"YELLOW",
			"ORANGE",
			"RED",
			"WHITE",
		};
		/*
		if (sensorColor < 0 || sensorColor > 10) {
			//Serial.println("NO COLOR");
			//display.drawStr(0, 10, "NO COLOR");
			//GPOC = (1 << BUZZER_PIN);
		} else {
			if (sensorColor == COLOR_BLUE || sensorColor == COLOR_LIGHTBLUE) {
				GPOS = (1 << BUZZER_PIN);
				//Serial.println(colorNames[sensorColor]);
			}
			sensorColor = COLOR_NONE;
			//display.drawStr(0, 20, colorNames[sensorColor]);
		}
		*/
		//display.updateDisplay();
		
		//display.printf("Lux: %d\n", lux);
		//Serial.printf("R: %d, G: %d, B: %d\n", red, green, blue);
#endif
		//display.printf("Angle: %f\n", angle_adjusted);
		//display.printf("M1: %d\n", motor1);
		//display.printf("M2: %d\n", motor2);
		//display.printf("Speed: %d\n", actual_robot_speed);
		//display.display();
		//Serial.printf("M1: %d\n", motor1);
		//Serial.printf("M2: %d\n", motor2);
		//Serial.println(ColorNameString(r, g, b));
		//display.updateDisplay();
#endif

	}  // End of slow loop
}
