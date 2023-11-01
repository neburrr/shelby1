#include "main.h"

int8_t alignedReady = 1;
int8_t endRace = 1;

volatile int8_t val = 1;
volatile int16_t linessr = 0;
volatile int32_t counter = 0;
volatile int16_t led = 0;
volatile int8_t rpm_counter = 0;

float raceTime = 0;
uint16_t raceCounter = 0;

int controller = 1;

int colorData[] = {0, 0, 0};

byte analogInputs[] = {A11, A7, A5, A1, A2, A3, A0, A9};
int16_t sensorsOffset[] = {100, 80, 100, 80, 120, 120, 100, 300};
//byte analogInputs[] = {A11, A5, A2, A0};
//int16_t sensorsOffset[] = {100, 100, 120, 100};
uint16_t analogSensorValues[NUM_SENSORS];
uint8_t digitalSensorValues[NUM_SENSORS];


volatile typedef struct {
  uint8_t state, new_state;
  float rotations = 0, rpm, new_rpm;
  float DC;
  unsigned long tes, tis;
} fsm_motors;

volatile typedef struct {
  uint8_t state, new_state;
  unsigned long tes, tis;
} fsm_linesensors;

fsm_motors m1, m2;
fsm_linesensors lssr;

void set_state_motors(fsm_motors & fsm, uint8_t new_state, float new_rpm);
void set_state_sensors(fsm_linesensors & fsm, uint8_t new_state);

void lineSensorM1();
void lineSensorM2();
void calibrate();
float getMotorsVoltage();
void controlEqualVelocity();
void clearLCD();

LinearRegression lrm1 = LinearRegression();
double valuesM1[3];
LinearRegression lrm2 = LinearRegression();
double valuesM2[3];

void setup(){
  //TCS3200 PIN OUTPUT

  pinMode(BOT_GREEN, INPUT);
  pinMode(BOT_RED, INPUT);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(39, INPUT);

  /*
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT_COLOR_SENSOR, INPUT);*/

  pinMode(OUT_LDR, INPUT);

  pinMode(FLOOR_LIGHTS, OUTPUT);

	DDRC |= (1<<DDC2); //LED VERMELHO COMO OUTPUT
	DDRC |= (1<<DDC0); //LED VERDE COMO OUTPUT

	DDRE |= (1<<DDE3); //MOTOR1 COMO OUTPUT
	DDRE |= (1<<DDE5); //MOTOR2 COMO OUTPUT

	OCR3A = 0; //PIN5 NO MOTOR1 (MOTOR DA DIREITA)
	OCR3C = 0; //PIN3 NO MOTOR2 (MOTOR DA ESQUERDA)
	
	TCCR3A = 0;
	TCCR3B = 0;
	TCCR3A = (1 << COM3A1) | (1 << COM3C1) | (1 << WGM31); //MODO NAO INVERTIDO | FAST PWM
  TCCR3B = (1 << WGM32) | (1 << WGM33) | (1 << CS31); //FAST PWM | SEM PRESCALER
	ICR3 = 0x03ff; //10 BIT DE RESOLUÇAO (DUTY CYCLE= OCR3X/ICR3) 

	TCCR2A |= (1<<WGM21); //MODO CTC
	TIMSK2 |= (1<<OCIE2A); //ISR COMP VECT
	TCCR2B |= (1<<CS22); //256 PRESCALER
  OCR2A = 10; //f=1008hz (F=16M/2*256*(1+30))

  //OCR2A = 10; //f=2840Hz

  attachInterrupt(digitalPinToInterrupt(19), lineSensorM1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), lineSensorM2, CHANGE);

  Serial.begin(9600);

  lcd.begin(COLUMS, ROWS);      
 	//lcd.print("WWWW");

	sei(); //INICIA A INTERRUPÇAO

  if(CALIBRATE_MOTORS){
    calibrate();
  }
}

ISR (TIMER2_COMPA_vect){
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {    
    //Serial.print(" - ");
    //Serial.print(analogRead(analogInputs[i]) - sensorsOffset[i]);

    if(analogRead(analogInputs[i]) - sensorsOffset[i] > IR_OFFSET)
      digitalSensorValues[i] = true;
    else  
      digitalSensorValues[i] = false;

    //Serial.print(" - ");
    //Serial.print(digitalSensorValues[i]);
  }

  //Serial.println();

  if(digitalSensorValues[0])
    lssr.new_state = 1;
  else if(digitalSensorValues[7])
    lssr.new_state = 7;
  else if(digitalSensorValues[2] && digitalSensorValues[3])
    lssr.new_state = 3;
  else if(digitalSensorValues[4] && digitalSensorValues[5])
    lssr.new_state = 5;
  else if(digitalSensorValues[1] || digitalSensorValues[2])
    lssr.new_state = 2;
  else if(digitalSensorValues[5] || digitalSensorValues[6])
    lssr.new_state = 6;
  else if(digitalSensorValues[3] || digitalSensorValues[4])
    lssr.new_state = 4;

  if(digitalSensorValues[0] && digitalSensorValues[1] &&digitalSensorValues[2] && digitalSensorValues[4] && digitalSensorValues[5] && digitalSensorValues[6] && digitalSensorValues[7])
    lssr.new_state = 8; //end

  //Serial.println(lssr.state);
  //exception states

  /*
  for(uint8_t i = 0; i < NUM_SENSORS - 1; i++){
    if(digitalSensorValues[i]){
      if(!digitalSensorValues[i + 1]){
        for(uint8_t j = i; NUM_SENSORS; j++){
          if(digitalSensorValues[j]){
            lssr.new_state = lssr.state;
            break;
          }
        }
      }
    }
  }*/

  set_state_sensors(lssr, lssr.new_state);

  if(m1.rotations > MOTORS_CALIBRATION_SET || m2.rotations > MOTORS_CALIBRATION_SET){
    unsigned long cur_time = millis();
    m1.tis = cur_time - m1.tes;
    m2.tis = cur_time - m2.tes; 
    
    m1.new_rpm = ((m1.rotations/5) / m1.tis) * 1000;
    m2.new_rpm = ((m2.rotations/5) / m1.tis) * 1000;

    m1.rotations = 0;
    m2.rotations = 0;

    cur_time = millis();
    m1.tes = cur_time;
    m2.tes = cur_time;

    set_state_motors(m1, 1, m1.new_rpm);
    set_state_motors(m2, 1, m2.new_rpm);
  }

  if(digitalRead(BOT_RED)){
    set_state_motors(m1, false, m1.rotations);
    set_state_motors(m2, false, m2.rotations);
  }
  raceCounter++;
}

void lineSensorM1(){
  m1.rotations++;
}

void lineSensorM2(){
  m2.rotations++;
}

void calibrate(){
  double ocr3a[100];
  double ocr3c[100];
  double rpmM1[100];
  double rpmM2[100];

  OCR3A = 200;
  OCR3C = 200;

  float oldRPM = 0;
  float voltageDCDC = getMotorsVoltage();
  Serial.print("Voltage Out DCDC: ");
  Serial.println(voltageDCDC);
  Serial.print("MOTORS_CALIBRATION_SET: ");
  Serial.println(MOTORS_CALIBRATION_SET);
  Serial.println("OCR3A OCR3C OCR3A*VOLTAGEDCDC OCR3C*VOLTAGEDCDC RPM1 RPM2 ");
  
  for (uint8_t i = 0; i < 84; i++)
  {

    while(m1.rpm == oldRPM);
    oldRPM = m1.rpm;

    OCR3A += 10;
    OCR3C += 10;

    ocr3a[i] = OCR3A;
    ocr3c[i] = OCR3C;
    rpmM1[i] = m1.rpm;
    rpmM2[i] = m2.rpm;

    Serial.print(OCR3A);
    Serial.print(", ");
    Serial.print(OCR3C);
    Serial.print(", ");
    Serial.print(OCR3A*voltageDCDC);
    Serial.print(", ");
    Serial.print(OCR3C*voltageDCDC);
    Serial.print(", ");
    Serial.print(m1.rpm);
    Serial.print(", ");
    Serial.println(m2.rpm);
  }

  for (uint8_t i = 0; i < 84; i++){
    lrm1.learn(ocr3a[i],rpmM1[i]);
    lrm2.learn(ocr3c[i],rpmM2[i]);
  }

    Serial.print("Values: ");  
    lrm1.getValues(valuesM1);
    Serial.print("X = ");
    Serial.print(valuesM1[0]);
    Serial.print(" b = ");
    Serial.print(valuesM1[1]); 
    Serial.print(" n = ");
    Serial.println(valuesM1[2]);
    Serial.print("Values: ");  
    lrm2.getValues(valuesM2);
    Serial.print("X = ");
    Serial.print(valuesM2[0]);
    Serial.print(" b = ");
    Serial.print(valuesM2[1]); 
    Serial.print(" n = ");
    Serial.println(valuesM2[2]);
}

void race(){
  float RPMs1 = 800;
  float RPMs2 = 1200;
  float angleOffsetm1= 0;
  float angleOffsetm2= 0;
  float gainOL = 1.7;

  raceCounter = 0;
  
  while(lssr.state != 8){ // fim da pista
    if(lssr.state == 4 || lssr.state == 3 || lssr.state == 5){ //ROBOT FOLLOWING LINE. THE CARACTERISTIC EXPRESSIONS RULE
      //m1.DC = 5.722 * m_inc - 768.59; //ocr equation
      //m2.DC = 5.563 * m_inc - 833.55;
      RPMs1 += 300;
      RPMs2 += 300;

      angleOffsetm1 = 1;
      angleOffsetm2 = 1;
    }
    else if(lssr.state == 3){
      angleOffsetm1 = 1;
      angleOffsetm2 = 1 - 0.0461;
    }
    else if(lssr.state == 5) {
      angleOffsetm1 = 1 - 0.0461;
      angleOffsetm2 = 1;
    }
    else if(lssr.state == 2){
      angleOffsetm1 = 1;
      angleOffsetm2 = 1 - 0.0671;;
    }
    else if(lssr.state == 6){
      angleOffsetm1 = 1 - 0.0778;
      angleOffsetm2 = 1;
    }
    else if(lssr.state == 1) {
      angleOffsetm1 = 1;
      angleOffsetm2 = 1 - 0.1370;
    }
    else if(lssr.state == 7) {
      angleOffsetm1 = 1 - 0.1300;
      angleOffsetm2 = 1;
    }
    if(angleOffsetm1 != 1)
      angleOffsetm1 = angleOffsetm1/gainOL;
    if(angleOffsetm2 != 1)  
      angleOffsetm2 = angleOffsetm2/gainOL;

    m1.DC = ((RPMs1 + 768.99)/5.722) * angleOffsetm1;
    m2.DC = ((RPMs2 + 833.55)/5.563) * angleOffsetm2;

    if(OCR3A > 960)
      OCR3A = 960;
    if(OCR3C > 1024)
      OCR3C = 1024;

    OCR3A = m1.DC;
    OCR3C = m2.DC;

    if(RPMs1 > 4750 || RPMs2 > 4750){
      if(RPMs1 > 4850)
        RPMs1 = 4850;
      if(RPMs2 > 4850)
        RPMs2 = 4850;
      gainOL = 1.4;  
      /*  
      if(OCR3A < 965)
        OCR3A = 965;
      if(OCR3C < 1004)
        OCR3C = 1004; */
    }

    /*
    Serial.print(angleOffsetm1);
    Serial.print(" - ");
    Serial.print(m1.DC);
    Serial.print(" - ");
    Serial.print(OCR3A);
    Serial.print(" - ");
    Serial.print(OCR3C);
    Serial.print(" - ");
    Serial.println(lssr.state);*/
    }
    raceTime = raceCounter;
}

void brake(){
	//PORTC = (1<<4); //LIGA AS LUZES DE TRAVAGEM

  while((OCR3A > 10) && (OCR3C > 10)){ //SAI DO LOOP QUANDO QUALQUER UM DOS REGISTOS FOR MENOR QUE 10
		OCR3A = MOTORS_STOP_DUTYCYCLE;
		OCR3C = MOTORS_STOP_DUTYCYCLE;
	}

	OCR3A = 0; //DESLIGA O MOTOR1
	OCR3C = 0; //DESLIGA O MOTOR2

  set_state_motors(m1, 0, 0);
  set_state_motors(m2, 0, 0);
}

void controlEqualVelocity(){
  while(1){
      Serial.print(m1.rpm);
      Serial.print("  --  ");
      Serial.print(m2.rpm);
      Serial.print("  --  ");
      Serial.print(OCR3A);
      Serial.print("  --  ");
      Serial.println(OCR3C);

      if(OCR3A < 1000 && m1.rpm < m2.rpm)
        OCR3A = OCR3A + 5; 
      else if(OCR3C < 1000 && m2.rpm < m1.rpm)
        OCR3C = OCR3C + 5;
      else if (m1.rpm > m2.rpm)
        OCR3A = OCR3A - 5;
      else if (m2.rpm > m1.rpm)
        OCR3C = OCR3C - 5;
      
      if(OCR3A > 800 || OCR3C > 800){
        OCR3A = 800;
        OCR3C = 800;
        }
      }
}

bool startShot(){
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  colorData[0] = pulseIn(OUT_COLOR_SENSOR, LOW); //RED

  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  colorData[1] = pulseIn(OUT_COLOR_SENSOR, LOW); //BLUE
  
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  colorData[2] = pulseIn(OUT_COLOR_SENSOR, LOW); //GREEN
  
  /*
  Serial.print("red: ");
  Serial.print(colorData[0]);
  Serial.print(" blue: ");
  Serial.print(colorData[1]);
  Serial.print(" green: ");
  Serial.println(colorData[2]);*/

  if(colorData[2] <= GREEN_VALUE && colorData[1] <= BLUE_VALUE && colorData[0] <= RED_VALUE)
    return false;

  return true;
}

float getMotorsVoltage(){
  float val = analogRead(A13);
  //Serial.println(val);
  return val*5/1023*9688/1188;
}

void displayData(){
	raceTime = raceCounter * 0.992 / 1000;
  Serial.println(raceTime);
  Serial.println(raceCounter);
  clearLCD();
	while(endRace){
    //Serial.println(raceTime);
    //Serial.println(raceCounter);
    lcd.setCursor(0,0);
    lcd.print(" --Race  Done-- ");
    lcd.setCursor(0,1);
    lcd.print("Racetime:");
    lcd.print(raceTime);
    
		if(digitalRead(BOT_RED)){
      endRace = 0;
    }
	}
}

void clearLCD(){
	lcd.setCursor(0, 0);       
 	lcd.print(F("                "));
	lcd.setCursor(0, 1);
	lcd.print(F("                "));
}

void set_state_motors(fsm_motors & fsm, uint8_t new_state, float new_rpm)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
  fsm.rpm = fsm.new_rpm;
}

void set_state_sensors(fsm_linesensors & fsm, uint8_t new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

void loop(){
		alignedReady = 1;
    endRace = 1;
    
    //wait for both buttons to be pressed
    clearLCD();
    while(!(digitalRead(BOT_GREEN) && digitalRead(BOT_RED))){
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_RED, HIGH);
      lcd.setCursor(0,0);
      lcd.print("    ShelbyRR   ");
      lcd.setCursor(0,1);
      lcd.print("DC Voltage:");
      lcd.setCursor(11,1);
      lcd.print(getMotorsVoltage());
    }

    //wait for alignement
    clearLCD();
		while(alignedReady){
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, LOW);
      
      if(lssr.state == 4){
        digitalWrite(LED_GREEN, HIGH);
        lcd.setCursor(0,0);
        lcd.print("  -Aligned-  ");
        lcd.setCursor(0,1);
        lcd.print(" lssr.state:");
        lcd.setCursor(13,1);
        lcd.print(lssr.state);
      }
      else{
        lcd.setCursor(0,0);
        lcd.print("    ShelbyRR   ");
        lcd.setCursor(0,1);
        lcd.print(" lssr.state:");
        lcd.setCursor(13,1);
        lcd.print(lssr.state);
      }

      if(digitalRead(BOT_GREEN))
        alignedReady = 0; 
    }

    clearLCD();
    while (!digitalRead(39)){
      
      lcd.setCursor(0,0);
      lcd.print("Ready To Race");

      /*
      lcd.setCursor(0,1);
      lcd.print("R");
      lcd.print(colorData[0]);
      lcd.setCursor(5,1);
      lcd.print("G");
      lcd.print(colorData[2]);
      lcd.setCursor(11,1);
      lcd.print("B");
      lcd.print(colorData[1]);*/

      //digitalWrite(BOT_GREEN, HIGH);
      //digitalWrite(BOT_RED, HIGH);
      set_state_motors(m1, true, m1.rotations);
      set_state_motors(m2, true, m2.rotations);
    }

    //clearLCD();
    //lcd.setCursor(0,0);
    //lcd.print("   SEE YAAAAA ");
		race();
		brake();
		displayData();
}

