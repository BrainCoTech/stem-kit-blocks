// Ardruino libs
#include <Servo.h>
#include <boarddefs.h>
#include <IRremote.h>
#include <IRremoteInt.h>
#include <ir_Lego_PF_BitStreamEncoder.h>

// PIN 12 and Pin 13
#define LED_PIN 12
#define IR_RECV_PIN 13
#define FINGER_COUNT 5
#define IDLE_TIMEOUT 60000 // milliseconds
#define SERIAL_PORT 9600

#define ENABLE_IR_REMOTE_CONTROL 1
#define ENABLE_SERIAL_PORT_CONTROL 1

IRrecv irrecv(IR_RECV_PIN); // Initiate IR signal input
unsigned long idle_ts;

typedef enum {
    THUMB = 0,
    INDEX = 1,
    MIDDLE = 2,
    RING = 3,
    PINKY = 4
} Finger;

typedef enum {
    MOVE_THUMB = 2295, // btn 1
    MOVE_INDEX = 34935, // btn 2
    MOVE_MIDDLE = 18615, // btn 3
    MOVE_RING = 10455, // btn 4
    MOVE_PINKY = 43095, // btn 5
    MOVE_WAVE = 26775, // btn 6
    MOVE_ROCK = 22695, // btn 7
    MOVE_PAPER = 39015, // btn 8
    MOVE_SCISSOR = 6375, // btn 9
    
} IR_REMOTE_KEYS;

static int FINGER_PINS[5] = {5, 6, 9, 10, 11};
static int FINGER_MAX_DEGS[FINGER_COUNT] = {180, 180, 180, 180, 180};
static int FINGER_MIN_DEGS[FINGER_COUNT] = {0};

static Servo finger_servos[5];
static int current_finger_states[FINGER_COUNT] = {0};
//prevent from receiving constantly coming signal of rock, causing action not finished
static bool thumb_collision_lock = false;

//Receiving several chars from serial report
//https://forum.arduino.cc/index.php?topic=288234.0
static bool receiving_new_data_lock = false;
static int serial_port_finger_degs[FINGER_COUNT]; // an array to store the received data

bool check_thumb_collision(int new_finger_states[]) {
    return new_finger_states[0] && (new_finger_states[1] || new_finger_states[2]);
}

bool check_finger_states(int new_finger_states[]) {
    for (int i = 0; i < FINGER_COUNT; i++) {
        if (new_finger_states[i] != current_finger_states[i]) {
          return false;
        }
    }
    return true;
}

// single-finger control
void finger_control(int finger, int degree) {
    bool should_flip = finger >= RING;
    //enum Finger are int
    float adjusted = ((FINGER_MAX_DEGS[finger] - FINGER_MIN_DEGS[finger]) / 180.0) * degree + FINGER_MIN_DEGS[finger];
    int adjusted_deg = (int) (adjusted + 0.5);

    if(should_flip) adjusted_deg = FINGER_MAX_DEGS[finger] - adjusted_deg;
    finger_servos[finger].write(adjusted_deg);
    //keep track of current state
    current_finger_states[finger] = degree;
}

// multi-finger control
void multi_finger_control(int fingers_states[]) {
    idle_ts = millis();
    for (int i = 0; i < FINGER_COUNT; i++) finger_control((Finger)i, fingers_states[i]);
}

void gesture_rock(){
    //thumb bending first and then bending the other four fingers
    int finger_states_rock_stage1[FINGER_COUNT] = {180, 0, 0, 0, 0};
    int finger_states_rock_stage2[FINGER_COUNT] = {180 ,180, 180, 180, 180};
    bool is_previous_rock = check_finger_states(finger_states_rock_stage2);

    if (!is_previous_rock) {
        thumb_collision_lock = true;
        multi_finger_control(finger_states_rock_stage1);
        delay(200);
        multi_finger_control(finger_states_rock_stage2);
        thumb_collision_lock = false;
    }
}

void gesture_scissor(){
    int finger_states_scissor[5] = {180, 0, 0, 180, 180};
    multi_finger_control(finger_states_scissor);
}

void reset_finger_states(){
    int init_finger_states[5] = {0};
    multi_finger_control(init_finger_states);
}

//https://forum.arduino.cc/index.php?topic=288234.0
void on_received_serial_port() {
    if(Serial.available() > 0){
        static byte ndx = 0;
        char end_marker = '>';
        char received_char;
        
        while (Serial.available() > 0 && receiving_new_data_lock == false) {
            received_char = Serial.read();

            if (received_char != end_marker) {
                if (received_char == '1') {
                    serial_port_finger_degs[ndx] = 0; 
                    Serial.println("Receiving 1");
                }
                else if (received_char == '0') {
                    serial_port_finger_degs[ndx] = 180;
                    Serial.println("Receiving 0");
                } 

                ndx++;
                if (ndx >= FINGER_COUNT) {
                    ndx = FINGER_COUNT - 1;
                }
            }
            else {
                ndx = 0;
                receiving_new_data_lock = true;
            }
        }
    }
}

void on_received_remote_control(){
    decode_results remote_signal; // Save signal structure
    word remote_signal_code;  //remote_signal_code for IR remote control

    if (irrecv.decode(&remote_signal)) {
        remote_signal_code = remote_signal.value, HEX;
        Serial.println(remote_signal_code);
        trigger_hand_control(remote_signal_code);
        irrecv.resume();
    }
}

void start_finger_wave() {
    int finger_states_wave[FINGER_COUNT] = {0};
    for (int i = 0; i < FINGER_COUNT; i++) {
        finger_states_wave[i] = 180 - finger_states_wave[i];
        multi_finger_control(finger_states_wave);
        delay(500);
    }
    for (int i = FINGER_COUNT - 1; i >= 0; i--) {
        finger_states_wave[i] = 180 - finger_states_wave[i];
        multi_finger_control(finger_states_wave);
        delay(500);
    }
}

void trigger_hand_control_by_bits() {
    Serial.println("Trigger Hand Control By Bit");
    
    bool is_collision = check_thumb_collision(serial_port_finger_degs);
    if (is_collision) {
        serial_port_finger_degs[0] = FINGER_MIN_DEGS[0];
        multi_finger_control(serial_port_finger_degs);
        delay(200);
        finger_control(THUMB, FINGER_MAX_DEGS[0]);
    } else {
        multi_finger_control(serial_port_finger_degs);
    }
    receiving_new_data_lock = false;
}

void trigger_hand_control(word remote_signal_code) {
    switch(remote_signal_code) {
        case MOVE_THUMB: {
            finger_control(THUMB, 180 - current_finger_states[THUMB]);
            break;
        }
        case MOVE_INDEX: {
            finger_control(INDEX, 180 - current_finger_states[INDEX]);
            break;
        }
        case MOVE_MIDDLE: {
            finger_control(MIDDLE, 180 - current_finger_states[MIDDLE]);
            break;
        }
        case MOVE_RING: {
            finger_control(RING, 180 - current_finger_states[RING]);
            break;
        }
        case MOVE_PINKY: {
            finger_control(PINKY, 180 - current_finger_states[PINKY]);
            break;
        }
        case MOVE_WAVE: {
            reset_finger_states();
            delay(1000);
            start_finger_wave();
            break;
        }
        case MOVE_SCISSOR: {
            gesture_scissor();
            break;
        }
        case MOVE_PAPER: {
            reset_finger_states();
            break; 
        }
        case MOVE_ROCK: {
            gesture_rock();
            break;
        }
        default: // Error
            break;
    }
}

void setup() {
    Serial.println("Setup Start");
    //corresponding servo for the fingers
    for (int i = 0; i < FINGER_COUNT; i++) {
        finger_servos[i].attach(FINGER_PINS[i]);
    }
    // pinMode(IR_RECV_PIN, INPUT);
    #ifdef ENABLE_SERIAL_PORT_CONTROL
    Serial.begin(SERIAL_PORT);//connect to serial port, baud rate is 9600
    #endif
    reset_finger_states();

    //10 for what
    delay(10);

    #ifdef ENABLE_SERIAL_PORT_CONTROL
    irrecv.blink13(true); // if signal is received, then pin13 led light blink
    irrecv.enableIRIn(); // enable the singal receival function
    #endif

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(A0, HIGH);
    Serial.println("The Hand is ready!!" ) ;
}

void loop() {
    // release the fingers if they idle too long
    if (millis() - idle_ts > IDLE_TIMEOUT)
    {
        idle_ts = millis();
        reset_finger_states();
        Serial.println("Bending the fingers too long, Release!");
    }
    //communicate with python
    #ifdef ENABLE_SERIAL_PORT_CONTROL
    on_received_serial_port(); 
    #endif

    Serial.println(receiving_new_data_lock);
    if (receiving_new_data_lock == true) {
        trigger_hand_control_by_bits();
    }
    //control by IR remote
    #ifdef ENABLE_IR_REMOTE_CONTROL
    on_received_remote_control();
    #endif
}
