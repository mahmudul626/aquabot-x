#include <LiquidCrystal_I2C.h>
#include <Wire.h>

LiquidCrystal_I2C lcd (0x27, 16, 2);
int irleft = 13;
int irright = 12;
int echo = 11;
int trig = 10;
int lmotorF = 9;
int lmotorB = 6;
int rmotorF = 5;
int rmotorB = 3;
long duration;
float distance;


void setup() {
  pinMode(lmotorF, OUTPUT);
  pinMode(lmotorB, OUTPUT);
  pinMode(rmotorF, OUTPUT);
  pinMode(rmotorB, OUTPUT);
  pinMode(irleft, INPUT);
  pinMode(irright, INPUT);
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);
  lcd.init();
  lcd.backlight();
}

void loop() {
  digitalWrite(trig, LOW);
  delay(2);
  digitalWrite(trig, HIGH);
  delay(10);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH);
  distance = duration * 0.034 / 2;
  
  int lirValue = digitalRead(irleft);
  int rirValue = digitalRead(irright);
  if(lirValue == HIGH && rirValue == HIGH) {
    if(distance >= 10.00) {
      int speed = map(distance, 10, 30, 100, 255);
      speed = constrain(speed, 0, 255);
      analogWrite(lmotorF, speed);
      analogWrite(rmotorF, speed);
    } else if (distance <= 5.00) {
      digitalWrite(lmotorF, LOW);
      digitalWrite(rmotorF, LOW);
    }
  }
}