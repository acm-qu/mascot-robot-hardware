//Left Wheel BTS7960 motor driver sketch 
int LEFT_R_IS = 6;
int LEFT_R_EN = 2;
int LEFT_R_PWM = A4;
int LEFT_L_IS = 7;
int LEFT_L_EN = 4;
int LEFT_L_PWM = A3;

//Left Wheel BTS7960 motor driver sketch
int RIGHT_R_IS = 9;
int RIGHT_R_EN = 3;
int RIGHT_R_PWM = A1;
int RIGHT_L_IS = 8;
int RIGHT_L_EN = 5;
int RIGHT_L_PWM = A0;

void setup() {
  // put your setup code here, to run once:
  setupRightWheel();
  setupLeftWheel();
}

void loop() {
  // put your main code here, to run repeatedly:
  int i;
  for(i = 0; i <= 255; i= i+10){ //clockwise rotation
   rotateWheel('R', 1, i);
   rotateWheel('L', 1, i);
   delay(500);
  }
  delay(500);
  for(i = 0; i <= 255; i= i+10){ //counter clockwise rotation
   rotateWheel('R', -1, i);
   rotateWheel('L', -1, i);
   delay(500);
  }
  delay(500);
}

void setupLeftWheel() {
  pinMode(LEFT_R_IS, OUTPUT);
  pinMode(LEFT_R_EN, OUTPUT);
  pinMode(LEFT_R_PWM, OUTPUT);
  pinMode(LEFT_L_IS, OUTPUT);
  pinMode(LEFT_L_EN, OUTPUT);
  pinMode(LEFT_L_PWM, OUTPUT);
  digitalWrite(LEFT_R_IS, LOW);
  digitalWrite(LEFT_L_IS, LOW);
  digitalWrite(LEFT_R_EN, HIGH);
  digitalWrite(LEFT_L_EN, HIGH);
}

void setupRightWheel() {
  pinMode(RIGHT_R_IS, OUTPUT);
  pinMode(RIGHT_R_EN, OUTPUT);
  pinMode(RIGHT_R_PWM, OUTPUT);
  pinMode(RIGHT_L_IS, OUTPUT);
  pinMode(RIGHT_L_EN, OUTPUT);
  pinMode(RIGHT_L_PWM, OUTPUT);
  digitalWrite(RIGHT_R_IS, LOW);
  digitalWrite(RIGHT_L_IS, LOW);
  digitalWrite(RIGHT_R_EN, HIGH);
  digitalWrite(RIGHT_L_EN, HIGH);
}
// wheel='R' / 'L'
// dir = 1 clockwise / -1 counter-clockwise
// speed = 0 -> 255
void rotateWheel(char wheel, int dir, int speed) {
  if (wheel == 'R') {
    analogWrite(RIGHT_R_PWM, ((dir == 1) ? speed : 0));
    analogWrite(RIGHT_L_PWM, ((dir == -1) ? speed : 0));
  } else if (wheel == 'L') {
    analogWrite(LEFT_R_PWM, ((dir == 1) ? speed : 0));
    analogWrite(LEFT_L_PWM, ((dir == -1) ? speed : 0));
  }
  delay(500);
}