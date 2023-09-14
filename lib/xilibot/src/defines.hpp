#ifndef defines_h
#define defines_h

#include <Arduino.h>

// Enable I2C devices?
#define ENABLE_DISPLAY
#define ENABLE_COLOR_READER

//#define DEBUG 1 // 0 = No debug info (default) DEBUG 1 for console output

// NORMAL MODE PARAMETERS (MAXIMUM SETTINGS)
#define MAX_THROTTLE 550
#define MAX_STEERING 140
#define MAX_TARGET_ANGLE 14

// PRO MODE = MORE AGGRESSIVE (MAXIMUM SETTINGS)
#define MAX_THROTTLE_PRO 780	// Max recommended value: 860
#define MAX_STEERING_PRO 260	// Max recommended value: 280
#define MAX_TARGET_ANGLE_PRO 26 // Max recommended value: 32

// Default control terms for EVO 2
#define KP 0.32
#define KD 0.050
#define KP_THROTTLE 0.080
#define KI_THROTTLE 0.1
#define KP_POSITION 0.06
#define KD_POSITION 0.45
// #define KI_POSITION 0.02

// Control gains for raise-up (the raise-up movement require special control parameters)
#define KP_RAISEUP 0.1
#define KD_RAISEUP 0.16
#define KP_THROTTLE_RAISEUP 0 // No speed control on raise-up
#define KI_THROTTLE_RAISEUP 0.0

#define MAX_CONTROL_OUTPUT 500
#define ITERM_MAX_ERROR 30 // Iterm windup constants for PI control
#define ITERM_MAX 10000

#define ANGLE_OFFSET 1.0 // Offset angle for balance (to compensate robot own weight distribution)

// Servo definitions
#define SERVO_AUX_NEUTRO 1500 // Servo neutral position
#define SERVO_MIN_PULSEWIDTH 700
#define SERVO_MAX_PULSEWIDTH 2500

#define SERVO2_NEUTRO 1500
#define SERVO2_RANGE 1400

#define ZERO_SPEED 65535
#define MAX_ACCEL 14 // Maximum motor acceleration (MAX RECOMMENDED VALUE: 20) (default:14)

// AUX definitions
#define CLR(x, y) (x &= (~(1 << y)))
#define SET(x, y) (x |= (1 << y))
#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

// Motor enable pin
#define MOTOR_ENABLE D0

// X motor
#define DIR_M1_PIN D3
#define STEP_M1_PIN D4

// Y motor
#define DIR_M2_PIN D5
#define STEP_M2_PIN D6

#define TCS_INTERRUPT D7

#ifdef ENABLE_COLOR_READER
 // Colors (detected & LED (except NONE for this last one)) expected values
 #define COLOR_NONE      0xFF
 #define COLOR_BLACK     0
 #define COLOR_PINK      1
 #define COLOR_PURPLE    2
 #define COLOR_BLUE      3
 #define COLOR_LIGHTBLUE 4
 #define COLOR_CYAN      5
 #define COLOR_GREEN     6
 #define COLOR_YELLOW    7
 #define COLOR_ORANGE    8
 #define COLOR_RED       9
 #define COLOR_WHITE     10
#endif

// Include the rest of modules
#include "control.hpp"
#include "mpu6050.hpp"
#include "network.hpp"
#include "osc.hpp"

#endif