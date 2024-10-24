#include <Servo.h>
#include <math.h>

Servo myServo;
int servoPin {9};

void setup() {
  // open the serial port:
  Serial.begin(9600);   // baud rate
  myServo.attach(servoPin);
  myServo.write(0);
  Serial.println("Gripper Ready");
}

/* Conversion Variables */
float a_sq {625}; // a = 25 mm
float b_sq {2025}; // b = 45 mm
float a2 {50};
float offset{45};

int width2deg(float width) {
  // theta = acosd( (a^2 + b^2 - (width+offset)^2) / (2*a*b) )
  float x = width/2 + offset;
  int theta = (int) (acos( (a_sq + x*x - b_sq )/(a2*x)) / M_PI * 180);
  // pos_deg = pos_deg > 180 ? pos_deg - 90 : pos_deg;
  constrain(theta, 0 , 180);
  return theta;
}

int pos_deg{0};
int width{0};
void loop() {
    // pos_deg = width;
    // check for incoming serial data:
    if (Serial.available() > 0) {
        // read incoming serial data:
        width = Serial.parseFloat();
        pos_deg = width2deg(width);
        Serial.print("I saw ");
        Serial.print(width);
        Serial.print(", turning ");
        Serial.println(pos_deg);
        while (myServo.read() != pos_deg)
          myServo.write(pos_deg);
        // delay(1000);
    }
}
