#include <SoftwareSerial.h>
// #include <Stepper.h>

// Digging setups
const int digPin = 9;
bool isDigging = false; // check if the digging motor is already on
// int stepsPerRevolution = 2048;
// Stepper digStepper (stepsPerRevolution, 4, 6, 5, 7);
// int rpm = 15;
const int relayPin = 7;

const int ledPin = LED_BUILTIN;  // or use a custom pin like 7

SoftwareSerial motorSerial(10, 11); // RX, TX — only TX is used


// heartbeat setup
unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 500;  // ms

unsigned long lastCommandDigTime = 0;
const unsigned long commandDigTimeout = 30000;  // ms


void setup() {
  Serial.begin(9600);  // Start USB Serial for Serial Monitor
  motorSerial.begin(9600);
  stopMotors(); // ensure motor is not moving at start
  
  // initialize pins to be output type
  // pinMode(12, OUTPUT); // digging motor

  pinMode(ledPin, OUTPUT); // LED
  digitalWrite(ledPin, LOW); // start off
  digitalWrite(12, HIGH); // dig motor low

  // digStepper.setSpeed(rpm);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // high is OFF, the way I wired it..?

}

void stopMotors() {
  motorSerial.write(64);  // M1 stop
  motorSerial.write(192);  // M2 stop
}

void loop() {
  // digStepper.step(stepsPerRevolution);
  // delay(500);

  if (Serial.available()) {
    char cmd = Serial.read();
    lastCommandTime = millis(); //reset heartbeat timer

    digitalWrite(ledPin, HIGH);
    delay(20);                  // short blink, non-blocking if you want to risk it
    digitalWrite(ledPin, LOW);

    switch (cmd) {
      case 'f':
        // motor forward logic
        motorSerial.write(127); // M1, Forward
        motorSerial.write(255); // M2, Forward
        break;
      case 'b':
        // motor backward logic
        motorSerial.write(1); // M1, Reverse
        motorSerial.write(128); // M2, Reverse
        break;
      case 'l':
        // left turn logic
        motorSerial.write(1); // M1, Reverse
        motorSerial.write(255); // M2, Forward
        break;
      case 'r':
        // right turn logic
        motorSerial.write(127); // M1, Forward
        motorSerial.write(128); // M2, Reverse
        break;
      case 's':
        stopMotors();
        break;
      case 'd':
        Serial.println("should dig triggered");
        shouldDig();
        break;
      default:
        break;
    }
  }

  //heartbeat not received, kill the motors
  if (millis() - lastCommandTime > commandTimeout) {
    stopMotors();
    // diggingOff();
  }
  if (millis() - lastCommandDigTime > commandDigTimeout) {
    diggingOff();
  }
}

void shouldDig(){
  if(isDigging == false){
    lastCommandDigTime = millis();
    diggingOn();
    // isDigging = true;
    Serial.println("on");
  }
  else{
    diggingOff();
    // isDigging = false;
    Serial.println("off");
  }
}

void diggingOn(){
  // digitalWrite(digPin, HIGH);
  digitalWrite(relayPin, LOW);  // Turn on relay → motor spins
  isDigging = true;
  // delay(2000);
  // Serial.println("confirm dig on");
}

void diggingOff(){
  // digitalWrite(digPin, LOW);
  digitalWrite(relayPin, HIGH);   // Turn off relay → motor stops
  isDigging = false;
   // reset timer
  // Serial.println("confirm dig off");
  // delay(2000);
}

// void loop() {
//   delay(1000);
//   Serial.println("Stage 1");
//   motorSerial.write(127); // M1, Forward
//   motorSerial.write(255); // M2, Forward
//   delay(2000);
//   Serial.println("Stage 2");
//   motorSerial.write(1); // M1, Reverse
//   motorSerial.write(128); // M2, Reverse
//   delay(2000);
//   // Serial.println("Stage 3");
//   // motorSerial.write(127); // M1, Forward
//   // motorSerial.write(128); // M2, Reverse
//   // delay(2000);
//   // Serial.println("Stage 4");
//   // motorSerial.write(1); // M1, Reverse
//   // motorSerial.write(255); // M2, Forward
//   // delay(2000);
//   motorSerial.write('0'); // shutdown both motors
// }

