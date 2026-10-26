const int SIDE_ULTRASONIC_TRIG = 22;
const int SIDE_ULTRASONIC_ECHO = 24;

int get_side_distance() {
  digitalWrite(SIDE_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SIDE_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SIDE_ULTRASONIC_TRIG, LOW);

  return (int) (pulseIn(SIDE_ULTRASONIC_ECHO, HIGH) * .0343 / 2);
}

void setup() {
  Serial.begin(9600);
  pinMode(SIDE_ULTRASONIC_TRIG, OUTPUT);
  pinMode(SIDE_ULTRASONIC_ECHO, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(get_side_distance());
  delay(400);
}
