// This one takes lots of code from:
// BROBOT EVO 2 by JJROBOTS
// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS
// License: GPL v2

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <defines.hpp>
// OLED display
#ifdef ENABLE_DISPLAY
#include <U8g2lib.h>
#endif
// Color sensor
#ifdef ENABLE_COLOR_READER
#include <Adafruit_TCS34725.h>
#include <tcs34725.h>
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

uint8_t mode; // mode = 0 Normal mode, mode = 1 Pro mode (More aggressive)

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

#ifdef ENABLE_COLOR_READER

extern uint16_t red, green, blue;

#define LUX_TO_PERCENTAGE(val)                (getPercentage(val, 0.0105, -0.0843))
#define REFLECTED_LIGHT_TO_PERCENTAGE(val)    (getPercentage(val, 0.0017, -8))

#define MANHATTAN
#include <color_detection.hpp>

uint8_t       sensorColor;
uint8_t       reflectedLight;
uint8_t       ambientLight;
uint16_t      red, green, blue, clear, lux;
volatile bool sensorReady = true;

// Default settings: TCS34725_GAIN_4X,  TCS34725_INTEGRATIONTIME_154MS
TCS34725      rgb_sensor;

// TCS Interrupt Service Routine (color was retrieved)
void IRAM_ATTR tcs_isr()
{
    // If RGB_SENSOR_INTERRUPT_PORT is LOW, sensor is ready
    sensorReady = !digitalRead(TCS_INT_PIN);
}

/**
 * @brief Map lux/reflected light values to percentages
 *    Weights of the equation must be calculated empirically
 *    Map equation: y = ax + b
 *    System to solve:
 *      100% = MaxRawValue * a + b
 *      0% = MinRawValue * a + b
 *
 *    See macros LUX_TO_PERCENTAGE and REFLECTED_LIGHT_TO_PERCENTAGE.
 */
uint8_t getPercentage(const uint16_t rawValue, const float& a_coef, const float& b_coef) {
    int8_t percent = static_cast<int8_t>(rawValue * a_coef + b_coef);
    if (percent > 100)
        return 100;
    if (percent < 0)
        return 0;
    return static_cast<uint8_t>(percent);
}

void processColor()
{
	//Serial.printf("Ready? %d\n", sensorReady);
    if (sensorReady) {
        // Data measurement
        // noDelay param set to true: Asynchronous mode, must be used with interrupt configured.
        bool status = rgb_sensor.updateData(true);
		Serial.printf("Status? %d\n", status);
        if (status) {
            // Ambient light (lux) computation
            rgb_sensor.updateLux();

            int16_t lux = lround(rgb_sensor.lux);
			Serial.printf("Lux? %d\n", lux);

            if (lux >= 30) {
                // Set ambient light (lux) - map 0-100
                //ambientLight = map(rgb_sensor.lux, 0, rgb_sensor.maxlux, 0, 100);
                ambientLight = LUX_TO_PERCENTAGE(lux); // cast ?

                // RGBC Channels are usable
                // Map values to max ~440;
                // Continuous values from 0-65535 (16bits) to 0-1023 (10bits)
                red   = rgb_sensor.r_comp >> 6,
                green = rgb_sensor.g_comp >> 6,
                blue  = rgb_sensor.b_comp >> 6,

                // Set clear channel as reflected light - map 0-100
                reflectedLight = REFLECTED_LIGHT_TO_PERCENTAGE(rgb_sensor.c_comp);

                // Set detected color
                sensorColor = detectColor(red, green, blue);
				Serial.println(sensorColor, DEC);
            } else {
                sensorColor = COLOR_NONE;
				Serial.println(F("not enough light to guess colors"));
            }
            clear = rgb_sensor.c_comp >> 6;
            
            // Human readable debugging
        	Serial.print("Lux: "); Serial.print(rgb_sensor.lux, DEC);
            Serial.print("; max: "); Serial.print(rgb_sensor.maxlux);
            Serial.print("; R: "); Serial.print(red, DEC);
            Serial.print("; G: "); Serial.print(green, DEC);
            Serial.print("; B: "); Serial.print(blue, DEC);
            Serial.print("; C: "); Serial.println(clear, DEC);

            // Spreadsheet debugging
            /*Serial.print(rgb_sensor.lux, DEC); Serial.print(";");
            Serial.print(rgb_sensor.maxlux); Serial.print(";");
            Serial.print(red, DEC); Serial.print(";");
            Serial.print(green, DEC); Serial.print(";");
            Serial.print(blue, DEC); Serial.print(";");
            Serial.println(clear, DEC);*/
        } else {
            sensorColor = COLOR_NONE;
            Serial.println(F("not valid data! wait next measure"));
        }
        // Interrupt tear down
        rgb_sensor.tcs.clearInterrupt();
        sensorReady = false;
    }
}

// It was: TCS34725_INTEGRATIONTIME_614MS
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

#endif

void setup()
{
	Serial.begin(115200);
	Serial.print("Initiating...");

	// Configure pins
	pinMode(DIR_M1_PIN, OUTPUT);		// DIR MOTOR 1
	pinMode(STEP_M1_PIN, OUTPUT);		// STEP MOTOR 1
	pinMode(DIR_M2_PIN, OUTPUT);		// DIR MOTOR 2
	pinMode(STEP_M2_PIN, OUTPUT);		// STEP MOTOR 2
	pinMode(MOTOR_ENABLE, OUTPUT);		// ENABLE MOTORS PIN AS OUTPUT
	pinMode(TCS_INT_PIN, OUTPUT);		// TCS INTERRUPT
	pinMode(D8, OUTPUT);				// BUZZER
	digitalWrite(MOTOR_ENABLE, HIGH);   // Motors ENABLE
	digitalWrite(MOTOR_ENABLE, HIGH);   // Motors ENABLE

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
	
	// Make the buzzer sound
	// do not use tone() because it causes conflicts with timer1 used for the steppers!
	digitalWrite(D8, HIGH); delay(10); digitalWrite(D8, LOW);
	delay(10); 
	digitalWrite(D8, HIGH); delay(10); digitalWrite(D8, LOW);

	Serial.println(F("Initializing I2C devices..."));
#ifdef ENABLE_DISPLAY
	if (!display.begin()) {
		Serial.println(F("SH1106 allocation failed"));
		//for (;;) delay(1000);	 // Don't proceed, loop forever
	}
	//display.clearBuffer();
 	//display.setFont(u8g2_font_ncenB08_tr);	// choose a suitable font
	// Export as XBM and use this header: const unsigned char face1_bits[] U8X8_PROGMEM = {
	//display.setFont(u8g2_font_6x12_mr);
	//display.setFlipMode(0);
	display.drawXBM(0, 0, 128, 64, face1_bits);
	delay(500); // PaUse for 1/2 seconds
	//display.updateDisplay();
	display.sendBuffer();
#endif
	Serial.println("Don't move!");
	MPU6050_setup();  // setup MPU6050 IMU
	delay(500);

  	// Calibrate MPU gyros
  	MPU6050_calibrate();

	digitalWrite(D8, HIGH); delay(300); digitalWrite(D8, LOW); 
#ifdef ENABLE_COLOR_READER

	// Device config
	sensorColor = COLOR_NONE;
	
	while (!rgb_sensor.begin()) {
    	Serial.println(F("TCS34725 NOT found"));
    	delay(1000);
	}
	Serial.println(F("Found sensor"));

	// Setup for color sensor   
	attachInterrupt(TCS_INT_PIN, tcs_isr, FALLING);

	rgb_sensor.tcs.setGain(TCS34725_GAIN_1X);
	rgb_sensor.tcs.setIntegrationTime(TCS34725_INTEGRATIONTIME_614MS);
    // Set persistence filter to generate an interrupt for every RGB Cycle,
    // regardless of the integration limits
    rgb_sensor.tcs.write8(TCS34725_PERS, TCS34725_PERS_NONE);
    // RGBC interrupt enable. When asserted, permits RGBC interrupts to be generated.
    rgb_sensor.tcs.setInterrupt(true);

#endif

	// STEPPER MOTORS INITIALIZATION
	Serial.println("Steppers init");
	// Enable stepper drivers and TIMER interrupts

	initStepperMotors();

	setMotorSpeedM1(0);               // Motor1 stopped
	dir_M1 = 0;

	setMotorSpeedM2(0);               // Motor2 stopped
	dir_M2 = 0;

	digitalWrite(MOTOR_ENABLE, LOW);   // Enable motors
	delay(200);

	// Little motor vibration and servo move to indicate that robot is ready
	for (uint8_t k = 0; k < 3; k++) {
		setMotorSpeedM1(5);
		setMotorSpeedM2(5);
		delay(200);
		setMotorSpeedM1(-5);
		setMotorSpeedM2(-5);
		delay(200);
	}

  	Serial.println("Start...");
  	timer_old = micros();
	timer_prev = timer_old;
}

uint16_t r, g, b, c, colorTemp, lux;

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
			//Serial.print("M ");
			//Serial.print(OSCmove_speed);
			//Serial.print(" ");
			//Serial.print(OSCmove_steps1);
			//Serial.print(",");
			//Serial.println(OSCmove_steps2);
			positionControlMode = true;
			OSCmove_mode = false;
			target_steps1 = steps1 + OSCmove_steps1;
			target_steps2 = steps2 + OSCmove_steps2;
		}
		else
		{
			positionControlMode = false;
			throttle = (OSCfader[0] - 0.5) * max_throttle;
			// We add some exponential on steering to smooth the center band
			steering = OSCfader[1] - 0.5;
			if (steering > 0)
			steering = (steering * steering + 0.5 * steering) * max_steering;
			else
			steering = (-steering * steering + 0.5 * steering) * max_steering;
		}

		if ((mode == 0) && (OSCtoggle[0]))
		{
			// Change to PRO mode
			max_throttle = MAX_THROTTLE_PRO;
			max_steering = MAX_STEERING_PRO;
			max_target_angle = MAX_TARGET_ANGLE_PRO;
			mode = 1;
		}
		if ((mode == 1) && (OSCtoggle[0] == 0))
		{
			// Change to NORMAL mode
			max_throttle = MAX_THROTTLE;
			max_steering = MAX_STEERING;
			max_target_angle = MAX_TARGET_ANGLE;
			mode = 0;
		}
		}
		else if (OSCpage == 2) { // OSC page 2
		// Check for new user control parameters
		readControlParameters();
		}
	#if DEBUG==1
		Serial.print(throttle);
		Serial.print(" ");
		Serial.println(steering);
	#endif
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
		if ((MPU_sensor_angle>-15)&&(MPU_sensor_angle<15))
		angle_adjusted_filtered = angle_adjusted_filtered*0.99 + MPU_sensor_angle*0.01;
		
	#if DEBUG==1
		Serial.print(dt);
		Serial.print(" ");
		Serial.print(angle_offset);
		Serial.print(" ");
		Serial.print(angle_adjusted);
		Serial.print(",");
		Serial.println(angle_adjusted_filtered);
	#endif

		// We calculate the estimated robot speed:
		// Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
		actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward  

		int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 25.0; // 25 is an empirical extracted factor to adjust for real units
		int16_t estimated_speed = -actual_robot_speed + angular_velocity;
		estimated_speed_filtered = estimated_speed_filtered * 0.9 + (float)estimated_speed * 0.1; // low pass filter on estimated speed

	#if DEBUG==2
		Serial.print(angle_adjusted);
		Serial.print(" ");
		Serial.println(estimated_speed_filtered);
	#endif

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
		//    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
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
		digitalWrite(MOTOR_ENABLE, LOW);  // Motors enable
		// NOW we send the commands to the motors
		setMotorSpeedM1(motor1);
		setMotorSpeedM2(motor2);
		}
		else   // Robot not ready (flat), angle > angle_ready => ROBOT OFF
		{
		digitalWrite(MOTOR_ENABLE, HIGH);  // Disable motors
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

	} // End of new IMU data

	// Medium loop 7.5Hz
	if (loop_counter >= 15)
	{
		loop_counter = 0;

	} // End of medium loop
	else if (slow_loop_counter >= 100) // 1Hz
	{
		slow_loop_counter = 0;

		// Read only when speed < 1, and turn the led on before reading
		if (true) { // estimated_speed_filtered < 1 && fabs(angle_adjusted) < 75) {
#ifdef ENABLE_COLOR_READER
			processColor();
#endif
		}
		timer_prev = timer_value;
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
#if ENABLE_COLOR_READER
		//Serial.printf("\nTime:\n%d\n", timer_value - timer_prev);
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
		display.clearDisplay();
		//display.setCursor(0, 0);
		if (sensorColor < 0 || sensorColor > 10) {
			display.drawStr(0, 10, "NO COLOR");
		} else {
			Serial.println(colorNames[sensorColor]);
			display.drawStr(0, 20, colorNames[sensorColor]);
		}
		//display.printf("Lux: %d\n", lux);
#endif
		//display.printf("Angle: %f\n", angle_adjusted);
		//display.printf("M1: %d\n", motor1);
		//display.printf("M2: %d\n", motor2);
		//display.printf("Speed: %d\n", actual_robot_speed);
		//display.display();
		//Serial.printf("M1: %d\n", motor1);
		//Serial.printf("M2: %d\n", motor2);
		//Serial.printf("R: %d, G: %d, B: %d\n", r, g, b);
		//Serial.println(ColorNameString(r, g, b));
		//display.updateDisplay();
#endif

	}  // End of slow loop
}
