#include "Arduino.h"
#include "Drivetrain.h"

const int LEFT_ENA = 5;
const int LEFT_ENB = 4;
// left front
const int LEFT_IN1 = 27;
const int LEFT_IN2 = 29;
// left back
const int LEFT_IN3 = 31;
const int LEFT_IN4 = 33;
const int RIGHT_ENA = 3;
const int RIGHT_ENB = 2;
// right front
const int RIGHT_IN1 = 41;
const int RIGHT_IN2 = 37;
// right back
const int RIGHT_IN3 = 39;
const int RIGHT_IN4 = 35;

void Drivetrain::setup() {
  pinMode(LEFT_ENA, OUTPUT);
  pinMode(LEFT_ENB, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_IN3, OUTPUT);
  pinMode(LEFT_IN4, OUTPUT);
  pinMode(RIGHT_ENA, OUTPUT);
  pinMode(RIGHT_ENB, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);
}

void Drivetrain::forwards(float pow) {
  if (pow < 0) {
    backwards(-pow);
    return;
  }
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_IN3, HIGH);
  digitalWrite(LEFT_IN4, LOW);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(RIGHT_IN3, HIGH);
  digitalWrite(RIGHT_IN4, LOW);
  int actual_pow = (int) (pow * 255);
  analogWrite(RIGHT_ENA, actual_pow);
  analogWrite(RIGHT_ENB, actual_pow);
  analogWrite(LEFT_ENA, actual_pow);
  analogWrite(LEFT_ENB, actual_pow);
}

void Drivetrain::right(float pow) {
  if (pow < 0) {
    left(-pow);
    return;
  }
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_IN3, LOW);
  digitalWrite(LEFT_IN4, HIGH);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);
  digitalWrite(RIGHT_IN3, HIGH);
  digitalWrite(RIGHT_IN4, LOW);
  int actual_pow = (int) (pow * 255);
  analogWrite(RIGHT_ENA, actual_pow);
  analogWrite(RIGHT_ENB, actual_pow);
  analogWrite(LEFT_ENA, actual_pow);
  analogWrite(LEFT_ENB, actual_pow);
}

void Drivetrain::left(float pow) {
  if (pow < 0) {
    right(-pow);
    return;
  }
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(LEFT_IN3, HIGH);
  digitalWrite(LEFT_IN4, LOW);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(RIGHT_IN3, LOW);
  digitalWrite(RIGHT_IN4, HIGH);
  int actual_pow = (int) (pow * 255);
  analogWrite(RIGHT_ENA, actual_pow);
  analogWrite(RIGHT_ENB, actual_pow);
  analogWrite(LEFT_ENA, actual_pow);
  analogWrite(LEFT_ENB, actual_pow);
}

void Drivetrain::backwards(float pow) {
  if (pow < 0) {
    forwards(-pow);
    return;
  }
  int actual_pow = (int) (pow * 255);
  analogWrite(RIGHT_ENA, actual_pow);
  analogWrite(RIGHT_ENB, actual_pow);
  analogWrite(LEFT_ENA, actual_pow);
  analogWrite(LEFT_ENB, actual_pow);
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(LEFT_IN3, LOW);
  digitalWrite(LEFT_IN4, HIGH);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);
  digitalWrite(RIGHT_IN3, LOW);
  digitalWrite(RIGHT_IN4, HIGH);
}

void Drivetrain::rotate_right(float pow) {
  if (pow < 0) {
    rotate_left(-pow);
    return;
  }
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_IN3, HIGH);
  digitalWrite(LEFT_IN4, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);
  digitalWrite(RIGHT_IN3, LOW);
  digitalWrite(RIGHT_IN4, HIGH);
  int actual_pow = (int) (pow * 255);
  analogWrite(RIGHT_ENA, actual_pow);
  analogWrite(RIGHT_ENB, actual_pow);
  analogWrite(LEFT_ENA, actual_pow);
  analogWrite(LEFT_ENB, actual_pow);
}

void Drivetrain::rotate_left(float pow) {
  if (pow < 0) {
    rotate_right(-pow);
    return;
  }
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(LEFT_IN3, LOW);
  digitalWrite(LEFT_IN4, HIGH);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(RIGHT_IN3, HIGH);
  digitalWrite(RIGHT_IN4, LOW);
  int actual_pow = (int) (pow * 255);
  analogWrite(RIGHT_ENA, actual_pow);
  analogWrite(RIGHT_ENB, actual_pow);
  analogWrite(LEFT_ENA, actual_pow);
  analogWrite(LEFT_ENB, actual_pow);
}

void Drivetrain::stop() {
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(LEFT_IN3, HIGH);
  digitalWrite(LEFT_IN4, HIGH);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, HIGH);
  digitalWrite(RIGHT_IN3, HIGH);
  digitalWrite(RIGHT_IN4, HIGH);
  analogWrite(RIGHT_ENA, 0);
  analogWrite(RIGHT_ENB, 0);
  analogWrite(LEFT_ENA, 0);
  analogWrite(LEFT_ENB, 0);
}
