#include <Wire.h>
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>

#define BLYNK_PRINT Serial

#define BLYNK_USE_DIRECT_CONNECT

#define COUNT_LOW 1638
#define COUNT_HIGH 7864
#define TIMER_WIDTH 16

#define LED2    2
#define A1      25
#define B1      26
#define PWM1    27
#define A2      18
#define B2      19
#define PWM2    23
#define SW1     4
#define SW2     5
#define Pin_BATT 33
#define Servo1 16
#define Servo2 17

static const int servosPins[2] = {16, 17};

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "vr1IHeY87lRaX4Ni02XW33Uq8txX-Kad";
int PWM_blynk = 0;
float batt;
float filterConstant = 0.975; // filter constant
float newVal = 0.0 ;
float sensorVal = 0.0;

float R1 = 22000.00;
float R2 = 22000.00;

unsigned long currentMillisLED2;
unsigned long LED2Millis; // when button was released
unsigned long LED2OnDelay; // wait to turn on LED
bool ledState2 = false;
bool onLED2 = false;

unsigned long currentMillisLoop;
unsigned long LoopMillis; // when button was released
unsigned long LoopOnDelay; // wait to turn on LED
unsigned long LoopOffDelay; // wait to turn on LED
unsigned long ledTurnedOnAt; // when led was turned on
bool ledReady = false;
bool onLoop = false;
bool statL = false;
bool ledstate = false;

float smooth(float data, float filterVal, float smoothedVal) {
  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);
  return smoothedVal;
}

float Batt_meter() {
  float VoutBat = 0.00;
  float VinBat = 0.00;
  int V_Batt_Value = 0;
  //V_Batt_Value = analogRead(Pin_BATT);
  sensorVal = analogRead(Pin_BATT);
  newVal = smooth(sensorVal, filterConstant, newVal);
  V_Batt_Value = newVal;
  VoutBat = (V_Batt_Value * 3.535) / 4095.00; // formula for calculating voltage out i.e. V+, here 5.00
  VinBat = VoutBat / (R2 / (R1 + R2)); // formula for calculating voltage in i.e. GND
  return (VinBat);
}

//********************** LED Blink delay millis function ***********************

void LED2_blink() {
  currentMillisLED2 = millis();
  if (onLED2 == true) {
    LED2Millis = currentMillisLED2;
    ledState2 = true;
    onLED2 = false;
  }
  if (ledState2 == true) {
    digitalWrite(LED2, HIGH);
    //Control_Output(DCOUT1, ON);
    if ((unsigned long)(currentMillisLED2 - LED2Millis) >= LED2OnDelay) {
      digitalWrite(LED2, LOW);
      //Control_Output(DCOUT1, OFF);
      ledState2 = false;
    }
  }
}

void LED2_Blinlkloop() {
  currentMillisLoop = millis();
  if (onLoop == true) {
    if (statL == false) {
      LoopMillis = currentMillisLoop;
      statL = true;
      ledReady = true;
    }
  }
  else {
    ledReady = false;
    statL = false;
  }

  if (ledReady == true) {
    if ((unsigned long)(currentMillisLoop - LoopMillis) >= LoopOnDelay) {
      digitalWrite(LED2, HIGH);
      //Control_Output(DCOUT1, ON);
      //Serial.println("*** LED2 ON *****");
      ledstate = true;
      ledTurnedOnAt = currentMillisLoop;
      ledReady = false;
    }
  }
  if (ledstate == true) {
    if ((unsigned long)(currentMillisLoop - ledTurnedOnAt) >= LoopOffDelay) {
      digitalWrite(LED2, LOW);
      //Control_Output(DCOUT1, OFF);
      //Serial.println("*** LED2 oFF *****");
      ledstate = false;
      statL = false;
    }
  }
}

void LED2_onBlinlk(unsigned long timeBlink) {
  onLED2 = true;
  LED2OnDelay = timeBlink;
}

void LED2_on_Blinlkloop(unsigned long timeOn , unsigned long timeOff) {
  onLoop = true;
  LoopOnDelay = timeOn;
  LoopOffDelay = timeOff;
}

void LED2_off_Blinlkloop() {
  onLoop = false;
  digitalWrite(LED2, LOW);
}

//***********************************************************************************


//-----------Motor_1-----------
void motor1_forword(byte pwm) {
  digitalWrite(A1, HIGH);
  digitalWrite(B1, LOW);
  ledcWrite(0, pwm);
}

void motor1_backword(byte pwm) {
  digitalWrite(A1, LOW);
  digitalWrite(B1, HIGH);
  ledcWrite(0, pwm);
}

void motor1_stop() {
  digitalWrite(A1, HIGH);
  digitalWrite(B1, HIGH);
  ledcWrite(0, 0);
}
//-----------Motor_2-----------
void motor2_forword(byte pwm) {
  digitalWrite(A2, HIGH);
  digitalWrite(B2, LOW);
  ledcWrite(1, pwm);
}

void motor2_backword(byte pwm) {
  digitalWrite(A2, LOW);
  digitalWrite(B2, HIGH);
  ledcWrite(1, pwm);
}
void motor2_stop() {
  digitalWrite(A2, HIGH);
  digitalWrite(B2, HIGH);
  ledcWrite(1, 0);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(LED2, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(B2, OUTPUT);
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);

  ledcSetup(0, 1000, 8);
  ledcAttachPin(PWM1, 0);

  ledcSetup(1, 1000, 8);
  ledcAttachPin(PWM2, 1);

  ledcSetup(3, 50, TIMER_WIDTH); // channel 3, 50 Hz, 16-bit width
  ledcAttachPin(Servo1, 3);   // GPIO 16 assigned to channel 3

  ledcSetup(4, 50, TIMER_WIDTH); // channel 4, 50 Hz, 16-bit width
  ledcAttachPin(Servo2, 4);   // GPIO 17 assigned to channel 4


  Serial.println("Waiting for connections...");
  Blynk.setDeviceName("MR32-BOT BLE");
  Blynk.begin(auth);
  Serial.println("MTB MR32-BOT Start");
}

void loop() {
  LED2_blink();
  LED2_Blinlkloop();
  Blynk.run();
  batt = Batt_meter();
}

BLYNK_WRITE(V0)
{
  if (param[0] == 1) {
    motor1_forword(PWM_blynk);
    motor2_forword(PWM_blynk);
  }
  else {
    motor1_stop();
    motor2_stop();
  }
}

BLYNK_WRITE(V1)
{
  if (param[0] == 1) {
    motor1_backword(PWM_blynk);
    motor2_backword(PWM_blynk);
  }
  else {
    motor1_stop();
    motor2_stop();
  }
}

BLYNK_WRITE(V2)
{
  if (param[0] == 1) {
    motor1_forword(PWM_blynk);
    motor2_backword(PWM_blynk);
  }
  else {
    motor1_stop();
    motor2_stop();
  }
}

BLYNK_WRITE(V3)
{
  if (param[0] == 1) {
    motor2_forword(PWM_blynk);
    motor1_backword(PWM_blynk);
  }
  else {
    motor1_stop();
    motor2_stop();
  }
}

BLYNK_WRITE(V4)
{
  PWM_blynk = param[0].asInt();
}

BLYNK_READ(V5)
{
  Blynk.virtualWrite(V5, batt);
}

BLYNK_WRITE(V6)
{
  ledcWrite(3, map(param[0].asInt(), 0, 180, COUNT_LOW, COUNT_HIGH));
}

BLYNK_WRITE(V7)
{
  ledcWrite(4, map(param[0].asInt(), 0, 180, COUNT_LOW, COUNT_HIGH));
}

BLYNK_WRITE(V8)
{
  if (param[0].asInt() == 1) {
    LED2_on_Blinlkloop(200, 50);
  }
  else {
    LED2_off_Blinlkloop();
  }
}
