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

// IRRemote controller of Carmp3 kind
typedef enum {
    BTN_CH_MINUS = 41565,
    BTN_CH = 25245,
    BTN_CH_PLUS = 57885,
    BTN_PREV = 8925,
    BTN_NEXT = 765,
    BTN_PLAY = 49725,
    BTN_VOL_DOWN = 57375,
    BTN_VOL_UP = 43095,
    BTN_EQ = 36975,
    BTN_0 = 26775,
    BTN_100_PLUS = 39015,
    BTN_200_PLUS = 45135,
    BTN_1 = 12495,
    BTN_2 = 6375,
    BTN_3 = 31365,
    BTN_4 = 4335,
    BTN_5 = 14535,
    BTN_6 = 23205,
    BTN_7 = 17085,
    BTN_8 = 19125,
    BTN_9 = 21165,
} IR_REMOTE_KEYS;

static int FINGER_PINS[5] = {5, 6, 9, 10, 11};
static int FINGER_MAX_DEGS[FINGER_COUNT] = {180, 180, 180, 180, 180};
static int FINGER_MIN_DEGS[FINGER_COUNT] = {0};

static Servo finger_servos[5];
static int current_finger_states[FINGER_COUNT] = {0};
//prevent from receiving constantly coming signal of rock, causing action not finished
static bool thumb_collision_lock = false;
static bool serial_port_data_lock = false;
//Receiving several chars from serial report
static int serial_port_finger_degs[FINGER_COUNT]; // an array to store the received data

bool check_thumb_collision(int new_finger_states[]) {
    // TODO: 90 is the value that may need adjustment or definition
    return new_finger_states[0] > 90 && (new_finger_states[1] > 90 || new_finger_states[2] > 90);
}

bool is_equal_to_prev_finger_states(int new_finger_states[]) {
    for (int i = 0; i < FINGER_COUNT; i++) {
        if (new_finger_states[i] != current_finger_states[i]) return false;
    }
    return true;
}

// Single-finger movement
void move_finger(int finger, int degree) {
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
void move_fingers(int fingers_states[]) {
    idle_ts = millis();
    for (int i = 0; i < FINGER_COUNT; i++) move_finger((Finger)i, fingers_states[i]);
}

void gesture_rock(){
    //thumb bending at last avoid collision
    static int finger_states_rock_stage1[FINGER_COUNT] = {0 ,180, 180, 180, 180};
    static int finger_states_rock_stage2[FINGER_COUNT] = {180, 180, 180, 180, 180};

    if (!is_equal_to_prev_finger_states(finger_states_rock_stage2)) {
        thumb_collision_lock = true;
        move_fingers(finger_states_rock_stage1);
        delay(200);
        move_fingers(finger_states_rock_stage2);
        thumb_collision_lock = false;
    }
}

void gesture_scissor(){
    static int finger_states_scissor[5] = {180, 0, 0, 180, 180};
    move_fingers(finger_states_scissor);
}

void reset_finger_states(){
    static int relaxed_finger_states[5] = {0};
    move_fingers(relaxed_finger_states);
}

void handle_serial_cmd() {
    unsigned char byte;
    if (Serial.available() > 0 && !thumb_collision_lock && !serial_port_data_lock) {
        serial_port_data_lock = true;
        byte = Serial.read();
        unsigned char a = 0b10000000;
        int finger_states[FINGER_COUNT];
        for (int i = 0; i < FINGER_COUNT; i ++){
            finger_states[i] = ((byte & a) >> (7-i)) * 180;
            a >>= 1;
        }

        if (!is_equal_to_prev_finger_states(finger_states)) {
            move_fingers_with_collision_checking(finger_states);
        }
        serial_port_data_lock = false;
    }
}

void handle_ir_remote_cmd() {
    decode_results remote_signal; // Save signal structure
    word remote_signal_code;  //remote_signal_code for IR remote control

    if (irrecv.decode(&remote_signal)) {
        remote_signal_code = remote_signal.value, HEX;
        Serial.println(remote_signal_code);
        move_fingers_with_ir_cmd(remote_signal_code);
        irrecv.resume();
    }
}

void start_finger_wave() {
    int finger_states_wave[FINGER_COUNT] = {0};
    for (int i = 0; i < FINGER_COUNT; i++) {
        finger_states_wave[i] = 180 - finger_states_wave[i];
        move_fingers(finger_states_wave);
        delay(500);
    }
    for (int i = FINGER_COUNT - 1; i >= 0; i--) {
        finger_states_wave[i] = 180 - finger_states_wave[i];
        move_fingers(finger_states_wave);
        delay(500);
    }
}

void move_fingers_with_collision_checking(int finger_states[]) {
    if (check_thumb_collision(finger_states)) {
        thumb_collision_lock = true;
        //TODO: Split original finger states into 2 states and move them. Do not use MIN & MAX
        finger_states[THUMB] = FINGER_MIN_DEGS[THUMB];
        move_fingers(serial_port_finger_degs);
        delay(200);
        finger_states[THUMB] = FINGER_MAX_DEGS[THUMB];
        move_fingers(finger_states);
        thumb_collision_lock = false;
    } else {
        move_fingers(finger_states);
    }
}

void move_fingers_with_ir_cmd(word remote_signal_code) {
    switch(remote_signal_code) {
        case BTN_1: {
            move_finger(THUMB, 180 - current_finger_states[THUMB]);
            break;
        }
        case BTN_2: {
            move_finger(INDEX, 180 - current_finger_states[INDEX]);
            break;
        }
        case BTN_3: {
            move_finger(MIDDLE, 180 - current_finger_states[MIDDLE]);
            break;
        }
        case BTN_4: {
            move_finger(RING, 180 - current_finger_states[RING]);
            break;
        }
        case BTN_5: {
            move_finger(PINKY, 180 - current_finger_states[PINKY]);
            break;
        }
        case BTN_6: {
            reset_finger_states();
            delay(1000);
            start_finger_wave();
            break;
        }
        case BTN_7: {
            gesture_scissor();
            break;
        }
        case BTN_8: {
            reset_finger_states();
            break;
        }
        case BTN_9: {
            //TODO: Add more detailed logging for each case
            gesture_rock();
            break;
        }
        default: // Error
            Serial.print("Unknown IR controller command received:" );
            Serial.println(remote_signal_code);
            break;
    }
}

void setup() {
    Serial.println("Setup Start");
    //corresponding servo for the fingers
    for (int i = 0; i < FINGER_COUNT; i++) {
        finger_servos[i].attach(FINGER_PINS[i]);
    }


    reset_finger_states();
    //TODO: Check delay 10 necessary?
    delay(10);

    #ifdef ENABLE_SERIAL_PORT_CONTROL
    Serial.begin(SERIAL_PORT);//connect to serial port, baud rate is 9600
    #endif

    #ifdef ENABLE_IR_REMOTE_CONTROL
    irrecv.blink13(true); // if signal is received, then pin13 led light blink
    irrecv.enableIRIn(); // enable the singal receival function
    #endif
    // TODO: Check what are these lines for and explain in comments
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(A0, HIGH);
    Serial.println("Setup completed" ) ;
}

void loop() {

    // Reset all fingers if no command is received after IDLE_TIMEOUT
    if (millis() - idle_ts > IDLE_TIMEOUT)
    {
        idle_ts = millis();
        reset_finger_states();
        Serial.println("Bending the fingers too long, Release!");
    }

    //control by serial port
    #ifdef ENABLE_SERIAL_PORT_CONTROL
    handle_serial_cmd();
    #endif

    //control by IR remote
    #ifdef ENABLE_IR_REMOTE_CONTROL
    handle_ir_remote_cmd();
    #endif
}
