#include <servo.h>

Servo myServo;
int servoPin {9};

void setup() {
  // open the serial port:
  Serial.begin(9600);   // baud rate
  myServo.atatch(servoPin);
}

void loop() {
    int pos_deg{0};
    // check for incoming serial data:
    if (Serial.available() > 0) {
        // read incoming serial data:
        float width = Serial.read();
        pos_deg = width; // fix this
        

    }
    myServo.write(pos_deg);
}
