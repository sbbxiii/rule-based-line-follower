/**
 * @file rule_based_follower.ino
 * @brief Differential-Drive Rule-Based Line Follower Controller
 * 
 * Implements deterministic O(1) Finite State Machine (FSM) control for a 5-channel
 * reflectance sensor array navigating complex track topographies (curves, intersections,
 * and 90-degree corners).
 * 
 * Hardware:
 *   - Arduino Uno (ATmega328P @ 16 MHz)
 *   - L298N Dual Full-Bridge Motor Driver
 *   - 2x 6V Micro Metal Gearmotors (10:1 ratio, 1000 RPM)
 *   - 5-Channel Analog Reflectance Sensor Array (15 mm spacing)
 *   - 7.4V Li-ion Power Supply
 */

#include "config.h"
#include "fsm_table.h"

// Last executed action for directional memory Γ(S_amb) = Γ(S_t-1)
static RobotAction lastAction = ACTION_FORWARD;

// Raw analog readings & binary threshold buffer
int rawSensors[NUM_SENSORS];
uint8_t binarySensors[NUM_SENSORS];

// Pin mappings array for easy iterative reading
const uint8_t sensorPins[NUM_SENSORS] = {
    PIN_IR_FAR_LEFT,
    PIN_IR_MID_LEFT,
    PIN_IR_CENTER,
    PIN_IR_MID_RIGHT,
    PIN_IR_FAR_RIGHT
};

/**
 * Configure digital GPIO directionality and PWM outputs.
 */
void setup() {
    Serial.begin(115200);

    // Motor driver control pins
    pinMode(PIN_MOTOR_ENA, OUTPUT);
    pinMode(PIN_MOTOR_IN1, OUTPUT);
    pinMode(PIN_MOTOR_IN2, OUTPUT);
    pinMode(PIN_MOTOR_IN3, OUTPUT);
    pinMode(PIN_MOTOR_IN4, OUTPUT);
    pinMode(PIN_MOTOR_ENB, OUTPUT);

    // Sensor pins configured as high-impedance inputs
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensorPins[i], INPUT);
    }

    // Safety halt on boot
    stopMotors();
    delay(1000);
    Serial.println(F("[BOOT] Rule-Based Line Follower Initialized."));
}

/**
 * Low-level motor driving primitive g: A -> Z^4
 * Controls PWM duty cycles and H-Bridge polarity.
 */
void setMotors(int leftSpeed, int rightSpeed, int leftDir, int rightDir) {
    // Left Motor Direction
    if (leftDir == DIR_FORWARD) {
        digitalWrite(PIN_MOTOR_IN1, HIGH);
        digitalWrite(PIN_MOTOR_IN2, LOW);
    } else if (leftDir == DIR_BACKWARD) {
        digitalWrite(PIN_MOTOR_IN1, LOW);
        digitalWrite(PIN_MOTOR_IN2, HIGH);
    } else {
        digitalWrite(PIN_MOTOR_IN1, LOW);
        digitalWrite(PIN_MOTOR_IN2, LOW);
    }

    // Right Motor Direction
    if (rightDir == DIR_FORWARD) {
        digitalWrite(PIN_MOTOR_IN3, HIGH);
        digitalWrite(PIN_MOTOR_IN4, LOW);
    } else if (rightDir == DIR_BACKWARD) {
        digitalWrite(PIN_MOTOR_IN3, LOW);
        digitalWrite(PIN_MOTOR_IN4, HIGH);
    } else {
        digitalWrite(PIN_MOTOR_IN3, LOW);
        digitalWrite(PIN_MOTOR_IN4, LOW);
    }

    // PWM Speed Regulation
    analogWrite(PIN_MOTOR_ENA, constrain(leftSpeed, 0, 255));
    analogWrite(PIN_MOTOR_ENB, constrain(rightSpeed, 0, 255));
}

// Custom movement primitives
void moveForward() {
    setMotors(BASE_SPEED, BASE_SPEED, DIR_FORWARD, DIR_FORWARD);
}

void turnLeft() {
    // Veer Left: reduce speed of left motor, drive right motor at fast speed
    setMotors(TURN_SPEED_SLOW, TURN_SPEED_FAST, DIR_FORWARD, DIR_FORWARD);
}

void turnRight() {
    // Veer Right: drive left motor at fast speed, reduce speed of right motor
    setMotors(TURN_SPEED_FAST, TURN_SPEED_SLOW, DIR_FORWARD, DIR_FORWARD);
}

void sharpLeft() {
    // Pivot Left: contra-rotate wheels (left backward, right forward)
    setMotors(SHARP_TURN_SPEED, SHARP_TURN_SPEED, DIR_BACKWARD, DIR_FORWARD);
}

void sharpRight() {
    // Pivot Right: contra-rotate wheels (left forward, right backward)
    setMotors(SHARP_TURN_SPEED, SHARP_TURN_SPEED, DIR_FORWARD, DIR_BACKWARD);
}

void stopMotors() {
    setMotors(0, 0, DIR_STOP, DIR_STOP);
}

/**
 * Execute commanded FSM action.
 */
void dispatchAction(RobotAction action) {
    switch (action) {
        case ACTION_FORWARD:
            moveForward();
            lastAction = ACTION_FORWARD;
            break;
        case ACTION_SLIGHT_LEFT:
            turnLeft();
            lastAction = ACTION_SLIGHT_LEFT;
            break;
        case ACTION_SLIGHT_RIGHT:
            turnRight();
            lastAction = ACTION_SLIGHT_RIGHT;
            break;
        case ACTION_SHARP_LEFT:
            sharpLeft();
            lastAction = ACTION_SHARP_LEFT;
            break;
        case ACTION_SHARP_RIGHT:
            sharpRight();
            lastAction = ACTION_SHARP_RIGHT;
            break;
        case ACTION_HALT:
            stopMotors();
            break;
        case ACTION_USE_LAST_STATE:
            // Ambiguous sensor reading: maintain trajectory momentum from previous cycle
            dispatchAction(lastAction);
            break;
    }
}

/**
 * Sample sensor array, threshold into binary state, and pack into 5-bit integer index.
 */
uint8_t sampleSensorState() {
    uint8_t stateWord = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        rawSensors[i] = analogRead(sensorPins[i]);
        // Inverted logic: 0 = dark line, 1 = white floor
        binarySensors[i] = (rawSensors[i] < SENSOR_THRESHOLD) ? 0 : 1;
        stateWord = (stateWord << 1) | binarySensors[i];
    }

    return stateWord;
}

/**
 * Main real-time control loop:
 * Evaluated in O(1) time per iteration.
 */
void loop() {
    // 1. Read optical sensor array
    uint8_t stateIndex = sampleSensorState();

    // 2. FSM Lookup: f: S -> A
    RobotAction commandedAction = STATE_LOOKUP_TABLE[stateIndex];

    // 3. Actuate Motors: g: A -> Z^4
    dispatchAction(commandedAction);

    // Optional serial telemetry for real-time monitoring
    #ifdef DEBUG_TELEMETRY
    Serial.print(F("State: ["));
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(binarySensors[i]);
    }
    Serial.print(F("] -> Action: "));
    Serial.println(getActionName(commandedAction));
    delay(10);
    #endif
}
