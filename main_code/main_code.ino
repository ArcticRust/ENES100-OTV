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
const int PULLEY_SERVO_PIN = 11;
const int PUMP_ENA = 45;
const int PUMP_IN1 = 43;

// servo stuff
Servo arm_servo;
Servo pulley_servo;
const float PULLEY_CONTRACTED = 170;
const float PULLEY_EXTENDED = 10;
const float ARM_DOWN = 16;
const float ARM_UP = 80;

// other various constants
const float ROTATE_POW = .3;
const float FORWARD_POW = .25;
const float SIDEWAYS_POW = .4;
const float MAX_THETA_ERROR = PI / 60; // 3 degrees
const float MAX_X_ERROR = .03;
const float MAX_Y_ERROR = .03;
const float ULTRASONIC_CLOSE = 30;
const float ULTRASONIC_FAR = 110;
const float ULTRASONIC_ERROR = 10;

float x_pos;
float y_pos;
float theta;

Drivetrain drivetrain = Drivetrain();

float angle_difference(float target, float current) {
  float diff = target - current;

  while (diff > PI) diff -= 2 * PI;
  while (diff < -PI) diff += 2 * PI;

  return diff;
}

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
  if (abs(angle_difference(0, theta)) < MAX_THETA_ERROR) {
    drivetrain.forwards(FORWARD_POW * positive);
  }
  else if (abs(angle_difference(PI / 2, theta)) < MAX_THETA_ERROR) {
    drivetrain.right(SIDEWAYS_POW * positive);
  }
  else if (abs(angle_difference(PI, theta)) < MAX_THETA_ERROR) {
    drivetrain.left(SIDEWAYS_POW * positive);
  }
  else {
    drivetrain.backwards(FORWARD_POW * positive);
  }

  while (abs(target_x - x_pos) > MAX_X_ERROR * 5) {
    x_pos = Enes100.getX();
  }

  while (abs(target_x - x_pos) > MAX_X_ERROR) {
    x_pos = Enes100.getX();
    if (abs(theta) < MAX_THETA_ERROR) {
    drivetrain.forwards(FORWARD_POW * positive);
    }
    else if (abs(angle_difference(PI / 2, theta)) < MAX_THETA_ERROR) {
      drivetrain.right(SIDEWAYS_POW * positive);
    }
    else if (abs(angle_difference(PI, theta)) < MAX_THETA_ERROR) {
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
  else if (abs(angle_difference(PI / 2, theta)) < MAX_THETA_ERROR) {
    drivetrain.forwards(FORWARD_POW * positive);
  }
  else if (abs(angle_difference(PI, theta)) < MAX_THETA_ERROR) {
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
    else if (abs(angle_difference(PI / 2, theta)) < MAX_THETA_ERROR) {
      drivetrain.forwards(FORWARD_POW * positive);
    }
    else if (abs(angle_difference(PI, theta)) < MAX_THETA_ERROR) {
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
  update_pos();
  float diff = angle_difference(angle, theta);

  bool going_left = diff > 0;

  if (going_left) drivetrain.rotate_left(ROTATE_POW);
  else drivetrain.rotate_right(ROTATE_POW);

  while (abs(angle_difference(angle, theta)) > 5 * MAX_THETA_ERROR) {
    theta = Enes100.getTheta();
    if (theta < 0) {
      theta = PI - theta;
    }
  }

  while (abs(angle_difference(angle, theta)) > MAX_THETA_ERROR) {
    if (going_left) drivetrain.left(ROTATE_POW);
    else drivetrain.right(ROTATE_POW);

    delay(50);
    drivetrain.stop();

    theta = Enes100.getTheta();
    if (theta < 0) {
      theta = PI - theta;
    }
  }

  drivetrain.stop();
}

void localize_to_bucket() {
  float init_theta = Enes100.getTheta();
  float angle_changes = PI / 45;
  float min_angle = init_theta;
  float min_dist = get_front_distance();

  rotate_to_angle(init_theta);
  for (int i = 1; i <= 10; i++) {
    rotate_to_angle(init_theta + i * angle_changes);
    float dist = get_front_distance();
    if (dist < min_dist) {
      min_angle = Enes100.getTheta();
      min_dist = dist;
    }
  }

  for (int i = 1; i <= 10; i++) {
    rotate_to_angle(init_theta - i * angle_changes);
    float dist = get_front_distance();
    if (dist < min_dist) {
      min_angle = Enes100.getTheta();
      min_dist = dist;
    }
  }

  rotate_to_angle(min_angle);

  while (get_front_distance() > 3) {
    drivetrain.forwards(FORWARD_POW);
    delay(50);
    drivetrain.stop();
  }
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

void contract_pulley() {
  pulley_servo.write(PULLEY_CONTRACTED); 
  delay(1000);
}

void extend_pulley() {
  pulley_servo.write(PULLEY_EXTENDED);
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
  int result = Enes100.MLGetPrediction(1);
  Enes100.println("Model predicts: " + String(result));
  
  if (result == 1) Enes100.mission(WATER_TYPE, FRESH_POLLUTED);
  else Enes100.mission(WATER_TYPE, FRESH_UNPOLLUTED);
  
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
   // Team Name, Mission Type, Marker ID, Room Number, Wifi Module TX Pin, Wifi Module RX Pin
   // Connects vision system to the OTV
  Enes100.begin("TrashTrappingTurtle", WATER, 88, 1120, WIFI_TX, WIFI_RX);
  Enes100.println("Connected...");

  drivetrain.setup();

  
  arm_servo.attach(ARM_SERVO_PIN);
  pulley_servo.attach(PULLEY_SERVO_PIN);

  
  pinMode(SIDE_ULTRASONIC_ECHO, INPUT);
  pinMode(FRONT_ULTRASONIC_ECHO, INPUT);
  pinMode(SIDE_ULTRASONIC_TRIG, OUTPUT);
  pinMode(FRONT_ULTRASONIC_TRIG, OUTPUT);
  pinMode(DEPTH_PIN, INPUT);  

}

void loop() {
  drivetrain.forwards(FORWARD_POW);
  delay(5000);
  drivetrain.rotate_left(ROTATE_POW);
  delay(5000);

}
