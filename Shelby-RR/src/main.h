#include <avr/interrupt.h>
#include <stdio.h>
#include <Arduino.h>
#include <LiquidCrystal.h>
#include <LinearRegression.h>

#define COLUMS 16
#define ROWS 2
#define RS 12
#define EN 14
#define D4 6
#define D5 10
#define D6 8
#define D7 4
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
    
#define NUM_SENSORS 8    
#define TIMEOUT 100      
#define EMITTER_PIN 2
#define DEBUG 0

#define IR_OFFSET 550

#define BOT_GREEN 13
#define BOT_RED 15
#define LED_GREEN 11
#define LED_RED 9
#define FLOOR_LIGHTS 33

#define CALIBRATE_MOTORS 0
#define MOTORS_CALIBRATION_SET 50

#define MOTORS_STOP_DUTYCYCLE 0

#define SOFT_CHANGE 3
#define MEDIUM_CHANGE 5
#define HARD_CHANGE 12
#define GAIN_OL 2

#define RED_VALUE 7
#define GREEN_VALUE 7
#define BLUE_VALUE 7
#define S0 28
#define S1 18
#define S2 20
#define S3 26
#define OUT_COLOR_SENSOR 30 

#define OUT_LDR 30 

