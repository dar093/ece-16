/*
  HC-SR04 NewPing Iteration Demonstration
  HC-SR04-NewPing-Iteration.ino
  Demonstrates using Iteration function of NewPing Library for HC-SR04 Ultrasonic Range Finder
  Displays results on Serial Monitor

  DroneBot Workshop 2017
  http://dronebotworkshop.com
*/

// This uses Serial Monitor to display Range Finder distance readings

// Include NewPing Library
#include "NewPing.h"

// Hook up HC-SR04 with Trig to Arduino Pin 10, Echo to Arduino pin 13
// Maximum Distance is 2000cm

#define TRIGGER_PIN1  10
#define ECHO_PIN1     10
#define TRIGGER_PIN2 11
#define ECHO_PIN2    11
#define TRIGGER_PIN3 12
#define ECHO_PIN3    12
#define MAX_DISTANCE 2000
 
NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);
NewPing sonar3(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE);

float duration1, distance1, duration2, distance2, duration3, distance3;

int iterations = 5;

const int vib_mot1 = 7;
const int vib_mot2 = 6;
const int vib_mot3 = 5;


void vibrate(float dist, int motor){
   if (dist <= 15 && dist >= 2) // Checking the distance, you can change the value
  {
    digitalWrite(motor, LOW);
    //digitalWrite(buzzer, HIGH);
  }
  else {
    digitalWrite(motor, HIGH);
    //digitalWrite(buzzer, LOW);
  }

  return;
}//end vibrate

void setup() {
  Serial.begin (9600);

  pinMode(vib_mot1, OUTPUT);
  pinMode(vib_mot2, OUTPUT);
  pinMode(vib_mot2, OUTPUT);
}//end setup

void loop() {
   
  duration1 = sonar1.ping_median(iterations);// Determine distance from duration
  //delayMicroseconds(200);
  duration2 = sonar2.ping_median(iterations);
  //delayMicroseconds(200); 
  duration3 = sonar3.ping_median(iterations);
  
  distance1 = (duration1 / 2) * 0.0343;
  distance2 = (duration2 / 2) * 0.0343;
  distance2 = (duration2 / 2) * 0.0343;
  
  // Send results to Serial Monitor
  Serial.print("Distance1: ");
  if (distance1 >= 2000 || distance1 <= 2) {
    Serial.print("Out of range");
  }//end if
  else {
    Serial.print(distance1);
    Serial.print(" cm");
    vibrate(distance1, vib_mot1);
  }//end else
  
  Serial.print(", Distance2: ");
  if (distance2 >= 2000 || distance2 <= 2) {
    Serial.print("Out of range");
  }//end if
  else {
    Serial.print(distance2);
    Serial.print(" cm");
    vibrate(distance2, vib_mot2);
  }//end else  

    Serial.print(", Distance3: ");
  if (distance3 >= 2000 || distance3 <= 2) {
    Serial.println("Out of range");
  }//end if
  else {
    Serial.print(distance3);
    Serial.println(" cm");
    vibrate(distance3, vib_mot3);
  }//end else

  
  
  
  

  
}//end loop
