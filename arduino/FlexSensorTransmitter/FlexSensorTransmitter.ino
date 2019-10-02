//Transmitter Code (Glove) - Mert Arduino and Tech
#include <SPI.h>                      //the communication interface with the modem
#include "RF24.h"                     //the library which helps us to control the radio modem

#define CE_PIN 9
#define CSN_PIN 10
#define SERIAL_PORT 9600
#define FINGER_COUNT 5

RF24 radio(CE_PIN, CSN_PIN);                     //9 and 10 are a digital pin numbers to which signals CE and CSN are connected.
const uint64_t pipe = 0xE0E0F0F1E1LL; //the address of the modem, that will receive data from Arduino.

//define the flex sensor input pins
typedef enum {
    THUMB = 0,
    INDEX = 1,
    MIDDLE = 2,
    RING = 3,
    PINKY = 4
} Finger;

static int flex_sensor_analog_pins = {A1, A2, A3, A4, A5};
static int flex_upper_bounds = {575, 590, 600, 600, 580};
static int flex_lower_bounds = {530, 520, 520, 520, 510};
static int servo_min_degrees = {5, 180, 5, 5, 180};
static int servo_max_degrees = {150, 35, 150, 150, 35};

static int radio_msg[FINGER_COUNT]; //Total number of data to be sent (data package)                         

void mappedFlexValue(int finger) {
  int finger_flex_value = analogRead(flex_sensor_analog_pins[finger]);
  radio_msg[finger] = map(finger_flex_value, flex_lower_bounds[finger], flex_upper_bounds[finger], servo_min_degrees[finger], servo_max_degrees[finger]);
}

void setup(void){
  Serial.begin(SERIAL_PORT);
  radio.begin();                      //it activates the modem.
  radio.openWritingPipe(pipe);        //sets the address of the receiver to which the program will send data.
}

void loop(void){

  for (int i = 0; i < FINGER_COUNT; i++) {
     mappedFlexValue((Finger)i);
  }
  
  radio.write(radio_msg, sizeof(radio_msg));
}
