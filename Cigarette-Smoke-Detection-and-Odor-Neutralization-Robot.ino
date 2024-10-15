/*
    Motor1= Top Left 
    Motor2= Bottom Left
    Motor3= Bottom Right
    Motor4= Top Right
 
    Motor1, Motor4 are forward 2 Motors
    Motor2, Motor3 are backward 2 Motors
 
  
 
  */
#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <MQ135.h>
#include <MQ2.h>
#include <MQ7.h>
#include "PMS.h"

#define relay1 4  // phun suong
#define relay2 4  // quat cho phun suong
#define relay3 4  // quat hut loc khi bui
#define relay4 4  //bat ion am

#define btn1 3 // mode do khi / mode che do di chuyen
#define btn2 3  
#define btn3 3 // phun sương
#define btn4 3//quat + ion am

#define ledRed 2
#define ledGreen 3
#define ledBlue 4

#define PIN_MQ7 A3
#define PIN_MQ2 A0
#define PIN_MQ135 A2  // MQ135 Analog Input Pin

#define Speed 180

#define RIGHT_IR A2  // Right IR sensor connected to analog pin A2 of Arduino Uno:
#define LEFT_IR A3
#define Trig A0
#define Echo A1
#define spoint 90

#define buzzer 5
SoftwareSerial BLESerial(20, 21);
SoftwareSerial ESPSerial(22, 23);
SoftwareSerial DUSTSerial(2, 3);  // RX, TX
//initial motors pin
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

Servo servo;
int pos = 0;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
PMS pms(DUSTSerial);
PMS::DATA data;


MQ135 mq135(PIN_MQ135);
MQ2 mq2(PIN_MQ2);
MQ7 mq7(PIN_MQ7, 5.0);


int distance;
int Left;
int Right;
int L = 0;
int R = 0;

int Right_Value = 0;  //Variable to store Right IR sensor value:
int Left_Value = 0;   //Variable to store Left IR sensor value:

int val;
int Speeed = 255;  // Change this value between 0 to 255 for speed

float h, t, correctedPPM, lpg, smoke, co;
int PM2_5, PM10_0;

bool phunsuongcheck = false;
void setup() {

  BLESerial.begin(9600);  //Set the baud rate to your Bluetooth module.
  Serial.begin(9600);
  DUSTSerial.begin(9600);
  if (!sht31.begin(0x44)) {  // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
  pinMode(buzzer, OUTPUT);
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledBlue, OUTPUT);
  pinMode(RIGHT_IR, INPUT);  //set analog pin RIGHT as an input:
  pinMode(LEFT_IR, INPUT);   //set analog pin RIGHT as an input:
  servo.attach(10);
  start();

  mq2.begin();

  for (pos = 90; pos <= 180; pos += 1) {  // goes from 90 degrees to 180 degrees:
    servo.write(pos);                     //tell servo to move according to the value of 'pos' variable:
    delay(15);                            //wait 15ms for the servo to reach the position:
  }
  for (pos = 180; pos >= 0; pos -= 1) {  // goes from 180 degrees to 0 degrees:
    servo.write(pos);                    //tell servo to move according to the value of 'pos' variable:
    delay(15);                           //wait 15ms for the servo to reach the position:
  }
  for (pos = 0; pos <= 90; pos += 1) {  //goes from 180 degrees to 0 degrees:
    servo.write(pos);                   //tell servo to move according to the value of 'pos' variable:
    delay(15);                          //wait 15ms for the servo to reach the position:
  }
}
void loop() {
}

//=======================================PHẦN HIỂN THỊ======================================================================================================================================
// lên màn hình và Blynk và đen báo tình trạng không khí

//=======================================PHẦN XỬ LÝ==========================================================================================================================================


void AUTOmode() {

  if (h < 30) {
    digitalWrite(relay1, HIGH);  // phun suong
    delay(100);
    digitalWrite(relay2, HIGH);  // quat phun sương
    phunsuongcheck = false;
  } else if (phunsuongcheck == false && h >= 30) {
    digitalWrite(relay1, LOW);  // phun suong
    delay(100);
    digitalWrite(relay2, LOW);  // quat phun sương
  }

  if (smoke > 5000 || co > 5000 || correctedPPM > 5000 || lpg > 5000 || PM2_5 > 30 || PM10_0 > 30) {
    digitalWrite(relay3, HIGH);  // quat loc khi bui

    if (co > 5000 || lpg > 5000) {
      digitalWrite(buzzer, HIGH);  // canh bao bang coi
      digitalWrite(relay1, HIGH);  // phun suong
      delay(100);
      digitalWrite(relay2, HIGH);  // quat phun sương
      phunsuongcheck = true;
    } else {
      digitalWrite(buzzer, LOW);  // tat canh bao bang coi
      digitalWrite(relay1, LOW);  // phun suong
      delay(100);
      digitalWrite(relay2, LOW);  // quat phun sương
      phunsuongcheck = false;
    }

    if (smoke > 5000 || correctedPPM > 5000 || PM2_5 > 30 || PM10_0 > 30) {
      digitalWrite(relay4, HIGH);  // bat ion am
    } else {
      digitalWrite(relay4, LOW);  // bat ion am
    }

  } else {
    digitalWrite(relay3, LOW);  // quat loc khi bui
    // chat luong khog khi tot
  }
}
void MANUALmode() {}



void ledRGB(int r, int g, int b) {
  analogWrite(ledRed, r);
  analogWrite(ledGreen, g);
  analogWrite(ledBlue, b);
}



//=======================================PHẦN ĐO KHÍ=========================================================================================================================================

void readSensor() {



  t = sht31.readTemperature();
  h = sht31.readHumidity();
  if (pms.read(data)) {
    PM2_5 = data.PM_AE_UG_2_5;   //(ug/m3)
     PM10_0 =data.PM_AE_UG_10_0; //(ug/m3)
  }

  correctedPPM = mq135.getCorrectedPPM(t, h);
  lpg = mq2.readLPG();
  smoke = mq2.readSmoke();
  co = mq7.getPPM();
}



// ======================================PHẦN DI CHUYỂN======================================================================================================================================


void FOLLOWmode() {
  delay(50);                //wait 50ms between pings:
  distance = ultrasonic();  //send ping, get distance in cm and store it in 'distance' variable:
  //Serial.print("distance");
  // Serial.println(distance);  // print the distance in serial monitor:
  Right_Value = digitalRead(RIGHT_IR);  // read the value from Right IR sensor:
  Left_Value = digitalRead(LEFT_IR);    // read the value from Left IR sensor:
  //Serial.print("RIGHT");
  //Serial.println(Right_Value);  // print the right IR sensor value in serial monitor:
  // Serial.print("LEFT");
  // Serial.println(Left_Value);  //print the left IR sensor value in serial monitor:
  if ((distance > 1) && (distance < 15)) {  //check wheather the ultrasonic sensor's value stays between 1 to 15.
                                            //If the condition is 'true' then the statement below will execute:
    //Move Forward:
    forward();
  } else if ((Right_Value == 0) && (Left_Value == 1)) {  //If the condition is 'true' then the statement below will execute:
    //Turn Left
    turnleft();
    delay(150);
  } else if ((Right_Value == 1) && (Left_Value == 0)) {  //If the condition is 'true' then the statement below will execute:
    //Turn Right
    turnright();
    delay(150);
  } else if (distance > 15) {  //If the condition is 'true' then the statement below will execute:
    //Stop
    Stop();
  }
}
void AVOIDmode() {
  distance = ultrasonic();
  if (distance <= 20) {
    Stop();
    back();
    delay(100);
    Stop();
    L = leftsee();
    servo.write(spoint);
    delay(800);
    R = rightsee();
    servo.write(spoint);
    if (L < R) {
      turnleft();
      delay(500);
      Stop();
      delay(200);
    } else if (L > R) {
      turnright();
      delay(500);
      Stop();
      delay(200);
    }
  } else {
    forward();
  }
}
void WIFImode() {
  if (ESPSerial.available() > 0) {
    val = ESPSerial.read();
    Stop();  //initialize with motors stoped
    if (val == 'F') {
      forward();
    }
    if (val == 'B') {
      back();
    }
    if (val == 'L') {
      left();
    }
    if (val == 'R') {
      right();
    }
    if (val == 'I') {
      topright();
    }
    if (val == 'J') {
      topleft();
    }
    if (val == 'K') {
      bottomright();
    }
    if (val == 'M') {
      bottomleft();
    }
    if (val == 'T') {
      Stop();
    }
  }
}
void BLEmode() {
  if (BLESerial.available() > 0) {
    val = BLESerial.read();
    Stop();  //initialize with motors stoped
    if (val == 'F') {
      forward();
    }
    if (val == 'B') {
      back();
    }
    if (val == 'L') {
      left();
    }
    if (val == 'R') {
      right();
    }
    if (val == 'I') {
      topright();
    }
    if (val == 'J') {
      topleft();
    }
    if (val == 'K') {
      bottomright();
    }
    if (val == 'M') {
      bottomleft();
    }
    if (val == 'T') {
      Stop();
    }
  }
}
void RFmode() {
}
void forward() {
  motor1.setSpeed(Speeed);
  motor1.run(FORWARD);
  motor2.setSpeed(Speeed);
  motor2.run(BACKWARD);
  motor3.setSpeed(Speeed);
  motor3.run(FORWARD);
  motor4.setSpeed(Speeed);
  motor4.run(BACKWARD);
}
void back() {
  motor1.setSpeed(Speeed);
  motor1.run(BACKWARD);
  motor2.setSpeed(Speeed);
  motor2.run(FORWARD);
  motor3.setSpeed(Speeed);
  motor3.run(BACKWARD);
  motor4.setSpeed(Speeed);
  motor4.run(FORWARD);
}
void left() {

  motor1.setSpeed(Speeed);
  motor1.run(BACKWARD);
  motor2.setSpeed(Speeed);
  motor2.run(BACKWARD);
  motor3.setSpeed(Speeed);
  motor3.run(BACKWARD);
  motor4.setSpeed(Speeed);
  motor4.run(BACKWARD);
}
void turnleft() {
  motor1.setSpeed(Speeed);
  motor1.run(BACKWARD);
  motor2.setSpeed(Speeed);
  motor2.run(BACKWARD);
  motor3.setSpeed(Speeed);
  motor3.run(FORWARD);
  motor4.setSpeed(Speeed);
  motor4.run(FORWARD);
}
void right() {

  motor1.setSpeed(Speeed);
  motor1.run(FORWARD);
  motor2.setSpeed(Speeed);
  motor2.run(FORWARD);
  motor3.setSpeed(Speeed);
  motor3.run(FORWARD);
  motor4.setSpeed(Speeed);
  motor4.run(FORWARD);
}
void turnright() {
  motor1.setSpeed(Speeed);
  motor1.run(FORWARD);
  motor2.setSpeed(Speeed);
  motor2.run(FORWARD);
  motor3.setSpeed(Speeed);
  motor3.run(BACKWARD);
  motor4.setSpeed(Speeed);
  motor4.run(BACKWARD);
}
void topleft() {

  motor1.setSpeed(Speeed);
  motor1.run(FORWARD);
  //  motor2.setSpeed(Speeed/3.1);
  //  motor2.run(FORWARD);
  motor3.setSpeed(Speeed);
  motor3.run(FORWARD);
  //  motor4.setSpeed(Speeed);
  //  motor4.run(FORWARD);
}
void topright() {

  //  motor1.setSpeed(Speeed);
  //  motor1.run(FORWARD);
  motor2.setSpeed(Speeed);
  motor2.run(BACKWARD);
  //  motor3.setSpeed(Speeed/3.1);
  //  motor3.run(FORWARD);
  motor4.setSpeed(Speeed);
  motor4.run(BACKWARD);
}
void bottomleft() {

  //  motor1.setSpeed(Speeed);
  //  motor1.run(FORWARD);
  motor2.setSpeed(Speeed);
  motor2.run(FORWARD);
  //  motor3.setSpeed(Speeed/3.1);
  //  motor3.run(FORWARD);
  motor4.setSpeed(Speeed);
  motor4.run(FORWARD);
}
void bottomright() {




  motor1.setSpeed(Speeed);
  motor1.run(BACKWARD);
  //  motor2.setSpeed(Speeed/3.1);
  //  motor2.run(FORWARD);
  motor3.setSpeed(Speeed);
  motor3.run(BACKWARD);
  //  motor4.setSpeed(Speeed);
  //  motor4.run(FORWARD);
}
void Stop() {
  motor1.setSpeed(0);
  motor1.run(RELEASE);  //stop the motor when release the button
  motor2.setSpeed(0);
  motor2.run(RELEASE);
  motor3.setSpeed(0);
  motor3.run(RELEASE);
  motor4.setSpeed(0);
  motor4.run(RELEASE);
}
void start() {
  delay(3000);
  for (int a = 0; a < 4; a++) {
    servo.write(spoint);
    delay(50);
    servo.write(40);
    delay(50);
    servo.write(90);
    delay(50);
    servo.write(spoint);
  }
}
int ultrasonic() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  long t = pulseIn(Echo, HIGH);
  long cm = t / 29 / 2;  //time convert distance
  return cm;
}
int leftsee() {
  servo.write(20);
  delay(800);
  Left = ultrasonic();
  return Left;
}
int rightsee() {
  servo.write(150);
  delay(800);
  Right = ultrasonic();
  return Right;
}
//================================================================================================================