# include <Arduino.h>
# include <Servo.h>

// Stepper pins
#define STEP_PIN1 5
#define DIR_PIN1 6
#define ENABLE_PIN1 9

#define STEP_PIN2 7
#define DIR_PIN2 8
#define ENABLE_PIN2 10

// Dimension
#define STEPS_PER_CM 1592

// Servo motor
Servo Marker;
int MarkerAngle = 90;
int currentMarkerAngle = -1;
String serialInput = "";

// Timing
unsigned long lastServoMoveTime = 0;
const unsigned long servoMoveInterval = 500; // ms between servo movements

// Stepper config
int motorDelay = 300;  // microseconds
int threshold = 100;

// Direction flags
byte lastDirection1 = 0; // 0 = idle, 1 = right, 2 = left
byte lastDirection2 = 0; // 0 = idle, 1 = up, 2 = down

// Step counters
long motor1Steps = 0;
long motor2Steps = 0;

// Functions
void stepMotor1(bool dir);
void stepMotor2(bool dir);
void moveTo(float x_cm, float y_cm);
void parseSerialCommand(String input);


void setup() {
    // Stepper pins
    pinMode(STEP_PIN1, OUTPUT);
    pinMode(DIR_PIN1, OUTPUT);
    pinMode(ENABLE_PIN1, OUTPUT);

    pinMode(STEP_PIN2, OUTPUT);
    pinMode(DIR_PIN2, OUTPUT);
    pinMode(ENABLE_PIN2, OUTPUT);

    // Servo
    Marker.attach(4);
    Marker.write(MarkerAngle);

    // Enable stepper drivers initially (keep enabled)
    digitalWrite(ENABLE_PIN1, LOW);
    digitalWrite(ENABLE_PIN2, LOW);

    Serial.begin(9600);
}

void loop(){
    // Read serial input
    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();
        if (inChar == '\n' || inChar == '\r') {
            if (serialInput.length() > 0) {
                parseSerialCommand(serialInput);
                serialInput = "";
            }
        } else {
            serialInput += inChar;
        }
    }
}

void stepMotor1(bool dir) {
    digitalWrite(DIR_PIN1, dir ? LOW : HIGH);  // adjust if direction is inverted
    digitalWrite(STEP_PIN1, HIGH);
    delayMicroseconds(motorDelay);
    digitalWrite(STEP_PIN1, LOW);
    delayMicroseconds(motorDelay);
    motor1Steps += dir ? 1 : -1;
}

void stepMotor2(bool dir) {
    digitalWrite(DIR_PIN2, dir ? HIGH : LOW);  // adjust if direction is inverted
    digitalWrite(STEP_PIN2, HIGH);
    delayMicroseconds(motorDelay);
    digitalWrite(STEP_PIN2, LOW);
    delayMicroseconds(motorDelay);
    motor2Steps += dir ? 1 : -1;
}

void moveTo(float x_cm, float y_cm) {

    // Clamp input values to physical limits
    x_cm = constrain(x_cm, 0.0, 13.5);  // Motor 1 range: 0 to 13.5 cm
    y_cm = constrain(y_cm, 0.0, 11.3);  // Motor 2 range: 0 to 11.3 cm

    long targetX = (long)(x_cm * STEPS_PER_CM);
    long targetY = (long)(y_cm * STEPS_PER_CM);

    long x0 = motor1Steps;
    long y0 = motor2Steps;
    long x1 = targetX;
    long y1 = targetY;

    long dx = abs(x1 - x0);
    long dy = abs(y1 - y0);

    int sx = (x1 > x0) ? 1 : -1;
    int sy = (y1 > y0) ? 1 : -1;

    long err = dx - dy;

    while (x0 != x1 || y0 != y1) {
        long e2 = 2 * err;

        if (e2 > -dy) {
            err -= dy;
            stepMotor1(sx == 1);
            x0 += sx;
        }

        if (e2 < dx) {
            err += dx;
            stepMotor2(sy == 1);
            y0 += sy;
        }
    }
}

void parseSerialCommand(String input) {
    input.trim(); // remove whitespace/newlines

    if (input.startsWith("GOTO")) {
        float x_cm, y_cm;
        int matched = sscanf(input.c_str(), "GOTO %f %f", &x_cm, &y_cm);

        if (matched == 2) {
            moveTo(x_cm, y_cm);
            Serial.print("Moving to X: ");
            Serial.print(x_cm);
            Serial.print(" cm, Y: ");
            Serial.println(y_cm);
        } else {
            Serial.println("Invalid GOTO format. Use: GOTO x y");
        }
    } 
    else if (input.equalsIgnoreCase("MARKER UP")) {
        MarkerAngle = 90;
        Marker.write(MarkerAngle);
    } 
    else if (input.equalsIgnoreCase("MARKER DOWN")) {
        MarkerAngle = 0;
        Marker.write(MarkerAngle);
    } 
    else {
        Serial.println("Unknown command.");
    }
}