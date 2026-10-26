#include "Enes100.h"
#include "Drivetrain.h"
#include <Servo.h>

// pins
const int WIFI_TX = 14; 
const int WIFI_RX = 15;
const int SIDE_ULTRASONIC_TRIG = 22;
const int SIDE_ULTRASONIC_ECHO = 24;
const int FRONT_ULTRASONIC_TRIG = 23;
const int FRONT_ULTRASONIC_ECHO = 25;
const int DEPTH_PIN = A0;
const int ARM_SERVO_PIN = 7;
const int PULLEY_SERVO_PIN = 6;
const int PUMP_ENA = 45;
const int PUMP_IN1 = 43;

// servo stuff
Servo arm_servo;
const float PULLEY_CONTRACTED = .9;
const float PULLEY_EXTENDED = .1;
const float ARM_DOWN = 30;
const float ARM_UP = 60;

// other various constants
const float ROTATE_POW = .4 * 255;
const float FORWARD_POW = .4 * 255;
const float SIDEWAYS_POW = .4 * 255;
const float MAX_THETA_ERROR = .02; // around 1.14 degrees of error
const float MAX_X_ERROR = .03;
const float MAX_Y_ERROR = .03;
const float ULTRASONIC_CLOSE = 30;
const float ULTRASONIC_FAR = 110;
const float ULTRASONIC_ERROR = 10;

float x_pos;
float y_pos;
float theta;

Drivetrain drivetrain = Drivetrain();

// try to use as little as possible, quite inefficient
void update_pos() {
  x_pos = Enes100.getX();
  y_pos = Enes100.getY();
  theta = Enes100.getTheta();
  if (theta < 0) {
    theta = PI - theta;
  }
}

void drive_to_x(float target_x) { // should account for drift at some point
  update_pos();
  int positive = target_x > x_pos ? 1 : -1;
  if (abs(theta) < MAX_THETA_ERROR) {
    drivetrain.forwards(FORWARD_POW * positive);
  }
  else if (abs(theta - PI / 2) < MAX_THETA_ERROR) {
    drivetrain.right(SIDEWAYS_POW * positive);
  }
  else if (abs(theta - PI) < MAX_THETA_ERROR) {
    drivetrain.left(SIDEWAYS_POW * positive);
  }
  else {
    drivetrain.backwards(FORWARD_POW * positive);
  }

  while (abs(target_x - x_pos) > MAX_X_ERROR * 5) {
    x_pos = Enes100.getX();
  }

  while (abs(target_x - x_pos) > MAX_X_ERROR * 5) {
    x_pos = Enes100.getX();
    if (abs(theta) < MAX_THETA_ERROR) {
    drivetrain.forwards(FORWARD_POW * positive);
    }
    else if (abs(theta - PI / 2) < MAX_THETA_ERROR) {
      drivetrain.right(SIDEWAYS_POW * positive);
    }
    else if (abs(theta - PI) < MAX_THETA_ERROR) {
      drivetrain.left(SIDEWAYS_POW * positive);
    }
    else {
      drivetrain.backwards(FORWARD_POW * positive);
    }
    delay(100);
    drivetrain.stop();
  }

  drivetrain.stop();
}

void drive_to_y(float target_y) { // should account for drift at some point
  update_pos();
  int positive = target_y > y_pos ? 1 : -1;
  if (abs(theta) < MAX_THETA_ERROR) {
    drivetrain.left(SIDEWAYS_POW * positive);
  }
  else if (abs(theta - PI / 2) < MAX_THETA_ERROR) {
    drivetrain.forwards(FORWARD_POW * positive);
  }
  else if (abs(theta - PI) < MAX_THETA_ERROR) {
    drivetrain.right(SIDEWAYS_POW * positive);
  }
  else {
    drivetrain.backwards(FORWARD_POW * positive);
  }

  while (abs(target_y - y_pos) > MAX_Y_ERROR * 5) {
    y_pos = Enes100.getY();
  }

  while (abs(target_y - y_pos) > MAX_Y_ERROR) {
    y_pos = Enes100.getY();
    if (abs(theta) < MAX_THETA_ERROR) {
      drivetrain.left(SIDEWAYS_POW * positive);
    }
    else if (abs(theta - PI / 2) < MAX_THETA_ERROR) {
      drivetrain.forwards(FORWARD_POW * positive);
    }
    else if (abs(theta - PI) < MAX_THETA_ERROR) {
      drivetrain.right(SIDEWAYS_POW * positive);
    }
    else {
      drivetrain.backwards(FORWARD_POW * positive);
    }
    delay(100);
    drivetrain.stop();
  }

  drivetrain.stop();
}

void rotate_to_angle(float angle) {
  bool going_right = true;
  if (abs(angle - theta) < 2 * PI - abs(angle - theta)) {
    if (angle > theta) { 
      drivetrain.rotate_left(ROTATE_POW);
      going_right = false;
    }
    else drivetrain.rotate_right(ROTATE_POW);
  }
  else {
    if (angle > theta) drivetrain.rotate_right(ROTATE_POW);
    else {
      drivetrain.rotate_left(ROTATE_POW);
      going_right = false;
    }
  }

  while (abs(angle - theta) > 5 * MAX_THETA_ERROR) {
    // avoids call to update_pos for latency purposes
    theta = Enes100.getTheta();
    if (theta < 0) {
      theta = PI - theta;
    } 
  }

  while (abs(angle - theta) > MAX_THETA_ERROR) {
    if (going_right) drivetrain.right(ROTATE_POW); // maybe multiply these by some constant?
    else drivetrain.left(ROTATE_POW);
    delay(100);
    drivetrain.stop();
    theta = Enes100.getTheta();
    if (theta < 0) {
      theta = PI - theta;
    }
  }
  drivetrain.stop();
}

int get_side_distance() {
  digitalWrite(SIDE_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SIDE_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SIDE_ULTRASONIC_TRIG, LOW);

  return (int) (pulseIn(SIDE_ULTRASONIC_ECHO, HIGH) * .0343 / 2);
}

int get_front_distance() {
  digitalWrite(FRONT_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(FRONT_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(FRONT_ULTRASONIC_TRIG, LOW);

  return (int) (pulseIn(FRONT_ULTRASONIC_ECHO, HIGH) * .0343 / 2);
}

void lower_arm() {
  arm_servo.write(ARM_DOWN);
  delay(1000);
}

void raise_arm() {
  arm_servo.write(ARM_UP);
  delay(1000);
}

// returns whether successful
bool get_and_send_water_depth() {
  int depth_value = analogRead(DEPTH_PIN);

  int water_height;

  if (depth_value == 0) return false; // no water detected

  if (depth_value < 490) water_height = 20;
  else if (depth_value < 565) water_height = 30;
  else water_height = 40;

  Enes100.mission(DEPTH, water_height);

  return true;
}

void get_and_send_pollutant_status() {
  if (Enes100.MLGetPrediction(1) == 1) Enes.mission(WATER_TYPE, FRESH_POLLUTED);
  else Enes.mission(WATER_TYPE, FRESH_UNPOLLUTED);
}

float get_navigation_y1(float measure_1, float measure_2, float measure_3) {
  int path1 = 0;
  int path2 = 0; 
  int path3 = 0;
  if (abs(measure_1 - ULTRASONIC_CLOSE) < ULTRASONIC_ERROR) path1 = 1;
  else if (abs(measure_1 - ULTRASONIC_FAR) < ULTRASONIC_ERROR) return .5;
  if (abs(measure_2 - ULTRASONIC_CLOSE) < ULTRASONIC_ERROR) path2 = 1;
  else if (abs(measure_2 - ULTRASONIC_FAR) < ULTRASONIC_ERROR) return 1.0;
  if (abs(measure_3 - ULTRASONIC_CLOSE) < ULTRASONIC_ERROR) path3 = 1;
  else if (abs(measure_3 - ULTRASONIC_FAR) < ULTRASONIC_ERROR) return 1.5;

  // only case remaining is that tracks in second column
  if (path1 == 0) return .5;
  if (path2 == 0) return 1.0;
  else return 1.5;
}

float get_navigation_y2(float measure_1, float measure_2, float measure_3) {
  int path1 = 0;
  int path2 = 0; 
  int path3 = 0;
  if (abs(measure_1 - ULTRASONIC_FAR) < ULTRASONIC_ERROR) path1 = 2;
  else if (abs(measure_1 - ULTRASONIC_CLOSE) < ULTRASONIC_ERROR) return .5;
  if (abs(measure_2 - ULTRASONIC_FAR) < ULTRASONIC_ERROR) path2 = 2;
  else if (abs(measure_2 - ULTRASONIC_CLOSE) < ULTRASONIC_ERROR) return 1.0;
  if (abs(measure_3 - ULTRASONIC_FAR) < ULTRASONIC_ERROR) path3 = 2;
  else if (abs(measure_3 - ULTRASONIC_CLOSE) < ULTRASONIC_ERROR) return 1.5;

  // only case remaining is that tracks in first column
  if (path1 == 0) return .5;
  if (path2 == 0) return 1.0;
  else return 1.5;
}

void setup() {
  // Update with proper AruCo cdoe
  Enes100.begin("TTT", WATER, 174, 1120, WIFI_TX, WIFI_RX);

  drivetrain.setup();

  arm_servo.attach(ARM_SERVO_PIN);

  pinMode(SIDE_ULTRASONIC_ECHO, INPUT);
  pinMode(FRONT_ULTRASONIC_ECHO, INPUT);
  pinMode(SIDE_ULTRASONIC_TRIG, OUTPUT);
  pinMode(FRONT_ULTRASONIC_TRIG, OUTPUT);
  pinMode(DEPTH_PIN, INPUT);
  Serial.begin(9600);
}

void loop() {
  delay(10000); // 10 second time between turn on and activation
  update_pos();
  if (y_pos > 1) {
    rotate_to_angle(3 * PI / 2);
    // TODO: find values and finish
    drive_to_y(.55); 
    // should probably do stuff with ultrasonic sensor here 
    // DO MISSION STUFF
  }
  else {
    rotate_to_angle(PI / 2);
    drive_to_y(1.45);
  }
  drive_to_y(1);
  rotate_to_angle(0);
  drive_to_x(1.1);
  drive_to_y(.5);
  rotate_to_angle(PI / 2);
  float side_measurement_1 = get_side_distance();
  drive_to_y(1.0);
  float side_measurement_2 = get_side_distance();
  drive_to_y(1.5);
  float side_measurement_3 = get_side_distance();
  drive_to_y(get_navigation_y1(side_measurement_1, side_measurement_2, side_measurement_3));
  drive_to_x(1.9);
  drive_to_y(get_navigation_y2(side_measurement_1, side_measurement_2, side_measurement_3));
  drive_to_x(3.0);
  drive_to_y(1.5);
  drive_to_x(3.7);

  while (true) {
    delay(100);
  }
}
