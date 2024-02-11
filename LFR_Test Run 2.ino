#include <Arduino.h>

const int motorENAPin = 11;//Right 
const int motorIN1Pin = 10;
const int motorIN2Pin = 9;

const int motorIN3Pin = 6;//Left
const int motorIN4Pin = 5;
const int motorENBPin = 3;

const int IRS1Pin = A0;
const int IRS2Pin = A1;
const int IRS3Pin = A2;
const int IRS4Pin = A3;
const int IRS5Pin = A4;
const int NEARPin = A5;
const int CLPPin = 9;
const int BUTTON_1 = 13;
const int BUTTON_2 = 12;

const float proportionalConstant = 50.0;
const float integralConstant = 0.01;
const float derivativeConstant = 0.1;

int minValues[6];  
int maxValues[6];  
int threshold[6];  

float integral = 0.0;
float previousError = 0.0;
int speed=0;

void moveForward(int speed) {
  analogWrite(motorENAPin, speed);
  analogWrite(motorENBPin, speed);
  digitalWrite(motorIN1Pin, HIGH);
  digitalWrite(motorIN2Pin, LOW);
  digitalWrite(motorIN3Pin, HIGH);
  digitalWrite(motorIN4Pin, LOW);
}

void moveLeft(int speed) {
  analogWrite(motorENAPin, speed);
  analogWrite(motorENBPin, speed);
  digitalWrite(motorIN1Pin, HIGH);
  digitalWrite(motorIN2Pin, LOW);
  digitalWrite(motorIN3Pin, LOW);
  digitalWrite(motorIN4Pin, HIGH);
}

void hardTurnLeft() {
  analogWrite(motorENAPin, 255);  
  analogWrite(motorENBPin, 0);  
  digitalWrite(motorIN1Pin, HIGH);
  digitalWrite(motorIN2Pin, LOW);
  digitalWrite(motorIN3Pin, LOW);
  digitalWrite(motorIN4Pin, LOW);
}

void moveRight(int speed) {
  analogWrite(motorENAPin, speed);
  analogWrite(motorENBPin, speed);
  digitalWrite(motorIN1Pin, LOW);
  digitalWrite(motorIN2Pin, HIGH);
  digitalWrite(motorIN3Pin, HIGH);
  digitalWrite(motorIN4Pin, LOW);
}

void hardTurnRight() {
  analogWrite(motorENAPin, 0);  
  analogWrite(motorENBPin, 255);  
  digitalWrite(motorIN1Pin, LOW);
  digitalWrite(motorIN2Pin, LOW);
  digitalWrite(motorIN3Pin, HIGH);
  digitalWrite(motorIN4Pin, LOW);
}

void moveBackward(int speed) {
  analogWrite(motorENAPin, speed);
  analogWrite(motorENBPin, speed);
  digitalWrite(motorIN1Pin, LOW);
  digitalWrite(motorIN2Pin, HIGH);
  digitalWrite(motorIN3Pin, LOW);
  digitalWrite(motorIN4Pin, HIGH);
}

void stopMotors() {
  analogWrite(motorENAPin, LOW);
  analogWrite(motorENBPin, LOW);
  digitalWrite(motorIN1Pin, LOW);
  digitalWrite(motorIN2Pin, LOW);
  digitalWrite(motorIN3Pin, LOW);
  digitalWrite(motorIN4Pin, LOW);
}

void makeUTurn() {
  analogWrite(motorENAPin, 225);
  analogWrite(motorENBPin, 225);
  digitalWrite(motorIN1Pin, HIGH);
  digitalWrite(motorIN2Pin, LOW);
  digitalWrite(motorIN3Pin, LOW);
  digitalWrite(motorIN4Pin, HIGH);
  delay(2000);
  stopMotors();
}

void calibrate() {
  Serial.println("Calibrating...");

  for (int sensor = 0; sensor < 5; sensor++) {
    minValues[sensor] = analogRead(sensor);
    maxValues[sensor] = analogRead(sensor);
  }

  for (int iteration = 0; iteration < 3000; iteration++) {
    moveRight(255);
    Serial.println("Moving Right");
  
      for (int sensor = 0; sensor < 5; sensor++) {
      int sensorValue = analogRead(sensor);
      if (sensorValue < minValues[sensor]) {
        minValues[sensor] = sensorValue;
      }
      if (sensorValue > maxValues[sensor]) {
        maxValues[sensor] = sensorValue;
      }
    }
  }

  for (int sensor = 0; sensor < 5; sensor++) {
    threshold[sensor] = (minValues[sensor] + maxValues[sensor]) / 2;
    Serial.print("Sensor ");
    Serial.print(sensor);
    Serial.print(" - Threshold: ");
    Serial.println(threshold[sensor]);
  }

  Serial.println("Calibration completed.");
  stopMotors();
}

void pidControl(int s1, int s2, int s3, int s4, int s5) {
  int error = (s1 * 1) + (s2 * 2) + (s3 * 0) + (s4 * (-2)) + (s5 * (-1));

  integral = integral + error;
  float derivative = error - previousError;

  float correction = proportionalConstant * error + integralConstant * integral + derivativeConstant * derivative;

  previousError = error;

  int leftSpeed = 255;
  int rightSpeed = 255;

  leftSpeed -= correction;
  rightSpeed += correction;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  if (correction > 0) {
    moveRight(rightSpeed);
    Serial.println("Moving Right");
  } else if (correction < 0) {
    moveLeft(leftSpeed);
    Serial.println("Moving Left");
  } else {
    moveForward(speed);
    Serial.println("Moving Forward");
  }
}

void setup() {
  pinMode(motorENAPin, OUTPUT);
  pinMode(motorIN1Pin, OUTPUT);
  pinMode(motorIN2Pin, OUTPUT);
  pinMode(motorIN3Pin, OUTPUT);
  pinMode(motorIN4Pin, OUTPUT);
  pinMode(motorENBPin, OUTPUT);

  pinMode(IRS1Pin, INPUT);
  pinMode(IRS2Pin, INPUT);
  pinMode(IRS3Pin, INPUT);
  pinMode(IRS4Pin, INPUT);
  pinMode(IRS5Pin, INPUT);
  pinMode(NEARPin, INPUT);
  pinMode(CLPPin, INPUT);

  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);

  Serial.begin(115200);
}

void loop() {
  while (digitalRead(BUTTON_1)) {}
  delay(1000);
  calibrate();
  while (digitalRead(BUTTON_2)) {}
  delay(1000);

while (1){
    int s1 = analogRead(IRS1Pin) > threshold[0];
    int s2 = analogRead(IRS2Pin) > threshold[1];
    int s3 = analogRead(IRS3Pin) > threshold[2];
    int s4 = analogRead(IRS4Pin) > threshold[3];
    int s5 = analogRead(IRS5Pin) > threshold[4];
    int N = digitalRead(NEARPin);
    int C = digitalRead(CLPPin);

   if (analogRead(IRS1Pin) > threshold[0] && analogRead(IRS2Pin) > threshold[1] && analogRead(IRS3Pin) < threshold[2] && analogRead(IRS4Pin) > threshold[3] && analogRead(IRS5Pin) > threshold[4]) 
   {pidControl(s1,s2,s3,s4,s5);
      Serial.println("Moving Forward");
    } else if (analogRead(IRS1Pin) < threshold[0] && analogRead(IRS2Pin) > threshold[1] && analogRead(IRS3Pin) > threshold[2] && analogRead(IRS4Pin) > threshold[3] && analogRead(IRS5Pin) > threshold[4])
    {moveLeft(255);
      Serial.println("Turn Left");
    } else if (analogRead(IRS1Pin) > threshold[0] && analogRead(IRS2Pin) > threshold[1] && analogRead(IRS3Pin) > threshold[2] && analogRead(IRS4Pin) > threshold[3] && analogRead(IRS5Pin) < threshold[4])  {
      moveRight(255);
      Serial.println("Turn Right");
    } else if (analogRead(IRS1Pin) < threshold[0] && analogRead(IRS2Pin) < threshold[1] && analogRead(IRS3Pin) > threshold[2] && analogRead(IRS4Pin) > threshold[3] && analogRead(IRS5Pin) > threshold[4])  {
      hardTurnLeft();
      Serial.println("Turn Sharp Left");
    } else if (analogRead(IRS1Pin) > threshold[0] && analogRead(IRS2Pin) > threshold[1] && analogRead(IRS3Pin) > threshold[2] && analogRead(IRS4Pin) < threshold[3] && analogRead(IRS5Pin) < threshold[4])  {
      hardTurnRight();
      Serial.println("Turn Sharp Right");
    } else if (analogRead(IRS1Pin) < threshold[0] && analogRead(IRS2Pin) > threshold[1] && analogRead(IRS3Pin) < threshold[2] && analogRead(IRS4Pin) > threshold[3] && analogRead(IRS5Pin) > threshold[4])  {
      while (analogRead(IRS3Pin) > threshold[2]) {
        moveLeft(255);
      }
      Serial.println("Acute Left");
    } else if (analogRead(IRS1Pin) > threshold[0] && analogRead(IRS2Pin) > threshold[1] && analogRead(IRS3Pin) < threshold[2] && analogRead(IRS4Pin) > threshold[3] && analogRead(IRS5Pin) < threshold[4])  {
      while (analogRead(IRS3Pin) > threshold[2]) {
        moveRight(255);
      }
      Serial.println("Acute Right");
    } else if (analogRead(IRS1Pin) < threshold[0] && analogRead(IRS2Pin) < threshold[1] && analogRead(IRS3Pin) < threshold[2] && analogRead(IRS4Pin) > threshold[3] && analogRead(IRS5Pin) > threshold[4])  {
      hardTurnLeft();
      Serial.println("Hard Left");
    } else if (analogRead(IRS1Pin) > threshold[0] && analogRead(IRS2Pin) > threshold[1] && analogRead(IRS3Pin) < threshold[2] && analogRead(IRS4Pin) < threshold[3] && analogRead(IRS5Pin) < threshold[4])  {
      hardTurnRight();
      Serial.println("Hard Right");
    } else if (analogRead(IRS1Pin) > threshold[0] && analogRead(IRS2Pin) > threshold[1] && analogRead(IRS3Pin) > threshold[2] && analogRead(IRS4Pin) > threshold[3] && analogRead(IRS5Pin) > threshold[4])  {
      moveForward(speed);
      while (analogRead(IRS3Pin) > threshold[2]) {
      pidControl(s1,s2,s3,s4,s5);
      }
    } else if (N == 1) {
  moveRight(255);
  delay(1000);
  moveForward(255);
  delay(1000);
  moveLeft(255);
  delay(1000);
  while (analogRead(IRS3Pin) > threshold[2]) {
    pidControl(s1, s2, s3, s4, s5);
    delay(1000);
  }
}
 else if (C == 1) {
      stopMotors();
      Serial.println("Stopping due to CLP");
    }

    delay(100);
  }
}
