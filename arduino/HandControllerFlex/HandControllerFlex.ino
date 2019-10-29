//Receiver Code (Hand) - Mert Arduino and Tech

#include <Servo.h>    //the library which helps us to control the servo motor
#include <SPI.h>      //the communication interface with the modem
#include "RF24.h"     //the library which helps us to control the radio modem

#define CE_PIN 9
#define CSN_PIN 10
#define FINGER_COUNT 5
#define PIPE_COUNT 1

static int FINGER_PINS[FINGER_COUNT] = {15, 16, 17, 18, 19};
static Servo finger_servos[FINGER_COUNT];
int radio_received_msg[FINGER_COUNT];

RF24 radio(CE_PIN, CSN_PIN);     /*This object represents a modem connected to the Arduino. 
                                  Arguments 9 and 10 are a digital pin numbers to which signals 
                                  CE and CSN are connected.*/

const uint64_t pipe = 0xE0E0F0F1E1LL; //the address of the modem,that will receive data from the Arduino.

void read_radio_messages() {
  if(radio.available()){
    bool is_read_all = false;
    while (!is_read_all){
      is_read_all = radio.read(radio_received_msg, sizeof(radio_received_msg));
      for (int i = 0; i < FINGER_COUNT; i++) {
        finger_servos[i].write(radio_received_msg[i]);  
      }
    }
  }
}

void setup(){

  // Record servos for the fingers
  for (int = 0; i < FINGER_COUNT; i++) {
    finger_servos[i].attach(FINGER_PINS[i]);  
  }
  
  radio.begin();                    //it activates the modem.
  radio.openReadingPipe(PIPE_COUNT, pipe);   //determines the address of our modem which receive data.
  radio.startListening();           //enable receiving data via modem
}

void loop(){
  read_radio_messages();
}
