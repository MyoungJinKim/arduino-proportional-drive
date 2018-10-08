#include <SoftwareSerial.h>

//
// L298N motor driver
//
#define ENA 5 
#define ENB 6
#define IN1 9
#define IN2 4
#define IN3 8
#define IN4 7

// drive the car with the given pwm values.
void drive(int pwmA, int pwmB);

//
// encoders
//
#define ENCODER_A 3
#define ENCODER_B 2
#define RESOLUTION 20.0

volatile int encoderA = 0;
volatile int encoderB = 0;

//
// wheel specification
//
#define DIAMETER      65.0
#define CIRCUMFERENCE (DIAMETER * PI)

//
// control routine
//

// move the car for the given distance with the specified pwm.
void move(int distance, int pwm = 150);

//
// bluetooth
//
#define TX 10
#define RX 11

SoftwareSerial mySerial(RX, TX);

void setup() {
  // setup serial ports
  Serial.begin(115200);
  mySerial.begin(57600);
  
  // L298N motor driver
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // encoders
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), ISR_encoderA, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), ISR_encoderB, FALLING);
}

float Kp = 0.0;

void loop() {
  if (mySerial.available()) {
    char command = mySerial.read();
    if (command == 'G' || command == 'g') {
      int distance = mySerial.parseInt();
      int pwm = mySerial.parseInt();
      move(distance, pwm);
    } else if (command = 'S' || command == 's') {
      Kp = mySerial.parseFloat();
      mySerial.print("set Kp = ");
      mySerial.println(Kp);
    }
  }
}

//
// ISR for encoders
//

void ISR_encoderA() {
  encoderA++;
}

void ISR_encoderB() {
  encoderB++;
}

//
// motor drive routine
//

void drive(int pwmA, int pwmB) {
  // MOTOR A direction
  if (pwmA > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if (pwmA < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // MOTOR B direction
  if (pwmB > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (pwmB < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  // speed of motors
  analogWrite(ENA, abs(pwmA));
  analogWrite(ENB, abs(pwmB));
}

//
// distance control routines
//

// returns number of ticks to go the specified distance.
int ticksForDistance(int distance) {
  return (int)(distance * RESOLUTION / CIRCUMFERENCE);
}

// move the car for the given distance with the specified pwm.
void move(int distance, int pwm) {
  int ticksToMove = ticksForDistance(distance);

  noInterrupts();
  encoderA = 0;
  encoderB = 0;
  interrupts();

  mySerial.print("Running ");
  mySerial.print(distance);
  mySerial.print(" with PWM = ");
  mySerial.println(pwm);
  mySerial.print(" and Kp = ");
  mySerial.print(Kp);
  
  while (encoderB <= ticksToMove) {
    int error = encoderB - encoderA;
    int pwmB = pwm;
    int pwmA = (int)(pwmB + Kp * error);

    mySerial.print(error);
    mySerial.print(", ");
    mySerial.print(pwmA);
    mySerial.print(", ");
    mySerial.println(pwmB);
    
    drive(pwmA, pwmB);
    delay(200);
  }
  drive(0, 0);
}

