// Define sensor pins
#define S1 35
#define S2 32
#define S3 33
#define S4 25
#define S5 26

// Define TB6612FNG motor driver pins
#define LEFT_PWM 27
#define LEFT_IN1 14
#define LEFT_IN2 4
#define RIGHT_PWM 13
#define RIGHT_IN1 12
#define RIGHT_IN2 15

// PID control variables
float Kp = 1.2;  // Adjust Proportional gain
float Ki = 0.02; // Small Integral gain for drift correction
float Kd = 0.6;  // Derivative gain for smoother turns

int lastError = 0;
int integral = 0;

void setup() {
    pinMode(S1, INPUT);
    pinMode(S2, INPUT);
    pinMode(S3, INPUT);
    pinMode(S4, INPUT);
    pinMode(S5, INPUT);

    pinMode(LEFT_PWM, OUTPUT);
    pinMode(LEFT_IN1, OUTPUT);
    pinMode(LEFT_IN2, OUTPUT);
    pinMode(RIGHT_PWM, OUTPUT);
    pinMode(RIGHT_IN1, OUTPUT);
    pinMode(RIGHT_IN2, OUTPUT);

    pinMode(2, INPUT_PULLUP); 
    while(digitalRead(2) == HIGH);
    while(digitalRead(2) == LOW);
}

void loop() {
    // Read sensor values (0 for black, 1 for white)
    int s1 = digitalRead(S1);
    int s2 = digitalRead(S2);
    int s3 = digitalRead(S3);
    int s4 = digitalRead(S4);
    int s5 = digitalRead(S5);
    
    // Stop condition: all sensors on white
    if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1) {
        stopMotors();
        return;
    }
    
    // Calculate error
    int error = (s1 * -2) + (s2 * -1) + (s3 * 0) + (s4 * 1) + (s5 * 2);
    
    // PID calculations
    integral += error;
    integral = constrain(integral, -100, 100); // Limit integral windup
    int derivative = error - lastError;
    int correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastError = error;
    
    // Dynamic base speed (slows down for sharp turns)
    int maxSpeed = 200;
    int minSpeed = 80;
    int baseSpeed = map(abs(error), 0, 2, maxSpeed, minSpeed);

    // Adjust motor speeds
    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    // Constrain speed within PWM range
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // Set motor speeds
    moveMotors(leftSpeed, rightSpeed);

    delay(10);
}

// Function to move motors with TB6612FNG
void moveMotors(int leftSpeed, int rightSpeed) {
    // Left motor control
    if (leftSpeed > 0) {
        digitalWrite(LEFT_IN1, HIGH);
        digitalWrite(LEFT_IN2, LOW);
    } else {
        digitalWrite(LEFT_IN1, LOW);
        digitalWrite(LEFT_IN2, HIGH);
        leftSpeed = -leftSpeed; // Convert to positive PWM
    }
    analogWrite(LEFT_PWM, leftSpeed);

    // Right motor control
    if (rightSpeed > 0) {
        digitalWrite(RIGHT_IN1, HIGH);
        digitalWrite(RIGHT_IN2, LOW);
    } else {
        digitalWrite(RIGHT_IN1, LOW);
        digitalWrite(RIGHT_IN2, HIGH);
        rightSpeed = -rightSpeed; // Convert to positive PWM
    }
    analogWrite(RIGHT_PWM, rightSpeed);
}

// Function to stop motors
void stopMotors() {
    analogWrite(LEFT_PWM, 0);
    analogWrite(RIGHT_PWM, 0);
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
}
