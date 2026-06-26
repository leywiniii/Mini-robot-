#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ----------------------------- PIN MAP --------------------------------

// Motor A (Left)
const int AIN1 = 25;
const int AIN2 = 26;
const int PWMA = 27;

// Motor B (Right)
const int BIN1 = 32;
const int BIN2 = 33;
const int PWMB = 14;

// HC-SR04 Ultrasonic Sensor
const int TRIG_PIN = 5;
const int ECHO_PIN = 17;   

// OLED Display (I2C) - uses the ESP32's default I2C pins
const int OLED_SDA = 21;
const int OLED_SCL = 22;
const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;
const int OLED_RESET_PIN = -1;  
const uint8_t OLED_I2C_ADDRESS = 0x3C;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);

// --------------------------- TUNABLE SETTINGS --------------------------

const int DRIVE_SPEED            = 120;   
const int TURN_SPEED             = 150;    
const int OBSTACLE_THRESHOLD_CM  = 20;    

const unsigned long SENSOR_INTERVAL_MS = 100;  
const unsigned long BACKUP_DURATION_MS = 300;  
const unsigned long TURN_DURATION_MIN_MS = 350; 
const unsigned long TURN_DURATION_MAX_MS = 650;

const unsigned long ULTRASONIC_TIMEOUT_US = 30000; 

// ------------------------------ STATE -----------------------------------

enum RobotState {
  STATE_FORWARD,
  STATE_BACKING_UP,
  STATE_TURNING
};

RobotState currentState = STATE_FORWARD;

unsigned long stateStartTime   = 0;  
unsigned long lastSensorCheck  = 0;  
unsigned long turnDuration     = 0; 

// ---- OLED face state ----
enum FaceExpression {
  FACE_HAPPY,
  FACE_SAD
};

bool oledReady = false;                 
FaceExpression lastDrawnFace = FACE_SAD;    

// =========================================================================
// SETUP
// =========================================================================

void setup() {
  Serial.begin(115200);

  setupMotors();
  setupUltrasonic();
  setupOLED();

  randomSeed(analogRead(0));

  Serial.println("Travis: obstacle avoidance starting up.");

  currentState = STATE_FORWARD;
  moveForward(DRIVE_SPEED);
  updateFace(FACE_HAPPY);
}

// =========================================================================
// MAIN LOOP (non-blocking state machine)
// =========================================================================

void loop() {
  unsigned long now = millis();

  switch (currentState) {

    case STATE_FORWARD:
      updateFace(FACE_HAPPY);

      if (now - lastSensorCheck >= SENSOR_INTERVAL_MS) {
        lastSensorCheck = now;

        long distance = getDistanceCM();

        if (distance > 0 && distance < OBSTACLE_THRESHOLD_CM) {
          Serial.print("Obstacle detected at ");
          Serial.print(distance);
          Serial.println(" cm. Backing up.");

          updateFace(FACE_SAD);
          stopMotors();
          moveBackward(DRIVE_SPEED);

          stateStartTime = now;
          currentState = STATE_BACKING_UP;
        }
      }
      break;

    case STATE_BACKING_UP:
      updateFace(FACE_SAD);

      if (now - stateStartTime >= BACKUP_DURATION_MS) {
        stopMotors();

        bool turnRightChosen = (random(0, 2) == 0);
        turnDuration = random(TURN_DURATION_MIN_MS, TURN_DURATION_MAX_MS + 1);

        if (turnRightChosen) {
          Serial.println("Turning right.");
          turnRight(TURN_SPEED);
        } else {
          Serial.println("Turning left.");
          turnLeft(TURN_SPEED);
        }

        stateStartTime = now;
        currentState = STATE_TURNING;
      }
      break;

    case STATE_TURNING:
      updateFace(FACE_SAD);

      if (now - stateStartTime >= turnDuration) {
        stopMotors();
        moveForward(DRIVE_SPEED);
        currentState = STATE_FORWARD;
      }
      break;
  }

}

// =========================================================================
// MOTOR FUNCTIONS (TB6612FNG)
// =========================================================================

void setupMotors() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  stopMotors();
}

void driveMotor(int in1Pin, int in2Pin, int pwmPin, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }

  analogWrite(pwmPin, abs(speed));
}

void moveForward(int speed) {
  driveMotor(AIN1, AIN2, PWMA, speed);
  driveMotor(BIN1, BIN2, PWMB, speed);
}

void moveBackward(int speed) {
  driveMotor(AIN1, AIN2, PWMA, -speed);
  driveMotor(BIN1, BIN2, PWMB, -speed);
}

void turnLeft(int speed) {
  driveMotor(AIN1, AIN2, PWMA, -speed);
  driveMotor(BIN1, BIN2, PWMB, speed);
}

void turnRight(int speed) {
  driveMotor(AIN1, AIN2, PWMA, speed);
  driveMotor(BIN1, BIN2, PWMB, -speed);
}

void stopMotors() {
  driveMotor(AIN1, AIN2, PWMA, 0);
  driveMotor(BIN1, BIN2, PWMB, 0);
}

// =========================================================================
// ULTRASONIC FUNCTIONS (HC-SR04)
// =========================================================================

void setupUltrasonic() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
}

long getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long durationUs = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);

  if (durationUs == 0) {
    return -1; 
  }

  long distanceCm = durationUs * 0.0343 / 2;
  return distanceCm;
}

// =========================================================================
// OLED FUNCTIONS (SSD1306, simple happy/sad face)
// =========================================================================

void setupOLED() {
  Wire.begin(OLED_SDA, OLED_SCL);

  oledReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);

  if (!oledReady) {

    Serial.println("OLED init failed. Continuing without display.");
    return;
  }

  display.clearDisplay();
  display.display();
}

void updateFace(FaceExpression expression) {
  if (!oledReady) return;
  if (expression == lastDrawnFace) return;

  drawFace(expression);
  lastDrawnFace = expression;
}

void drawFace(FaceExpression expression) {
  display.clearDisplay();

  const int eyeRadius = 8;
  const int leftEyeX   = 44;
  const int rightEyeX  = 84;
  const int eyeY       = 24;

  display.fillCircle(leftEyeX, eyeY, eyeRadius, SSD1306_WHITE);
  display.fillCircle(rightEyeX, eyeY, eyeRadius, SSD1306_WHITE);

  const int mouthCenterX  = SCREEN_WIDTH / 2;
  const int mouthTopY     = 42;  
  const int mouthHalfWide = 24;  
  const int mouthDepth    = 10;   

  bool happy = (expression == FACE_HAPPY);

  int prevX = mouthCenterX - mouthHalfWide;
  int prevY = mouthCurveY(prevX - mouthCenterX, mouthHalfWide, mouthTopY, mouthDepth, happy);

  for (int x = mouthCenterX - mouthHalfWide; x <= mouthCenterX + mouthHalfWide; x += 2) {
    int y = mouthCurveY(x - mouthCenterX, mouthHalfWide, mouthTopY, mouthDepth, happy);
    display.drawLine(prevX, prevY, x, y, SSD1306_WHITE);
    prevX = x;
    prevY = y;
  }

  display.display();
}


int mouthCurveY(int xOffset, int halfWidth, int topY, int depth, bool happy) {
  float t = (float)xOffset / halfWidth;        
  float curve = happy ? (1.0 - t * t) : (t * t);
  return topY + (int)(depth * curve);
}
