#include <SoftwareSerial.h>

SoftwareSerial HC05(10, 11);  // RX, TX

char command;
char lastCommand;

void setup() {
  HC05.begin(9600);  //Set the baud rate to your Bluetooth module.
  Serial.begin(9600);
}

void loop() {
  if (HC05.available() > 0) {
    command = HC05.read();

    // Stop(); //initialize with motors stoped
    if (command != lastCommand) {

    Serial.print("Received: ");
    Serial.println(command);
    }
    lastCommand = command;

    //   switch (command) {
    //     case 'F':
    //       forward();
    //       break;
    //     case 'B':
    //       back();
    //       break;
    //     case 'L':
    //       left();
    //       break;
    //     case 'R':
    //       right();
    //       break;
    //   }
  }
}