/*
  =======================================================================
  Travis - Mini ESP32 Companion Robot
  Module: Obstacle Avoidance + Motor Movement
  =======================================================================

  Hardware:
    - ESP32 Dev Module (30-pin)
    - TB6612FNG Motor Driver
    - 2x N20 DC Motors
    - HC-SR04 Ultrasonic Sensor

  Behavior:
    - Drives forward continuously.
    - Periodically pings the ultrasonic sensor (non-blocking via millis()).
    - When an obstacle is detected within OBSTACLE_THRESHOLD_CM:
        1. Stops, backs up briefly.
        2. Randomly turns left or right for a randomized duration.
        3. Resumes driving forward.

  Notes:
    - Motor A (AIN1/AIN2/PWMA) is assumed to be the LEFT motor.
      Motor B (BIN1/BIN2/PWMB) is assumed to be the RIGHT motor.
      If Travis turns the "wrong" way in practice, just swap the
      turnLeft()/turnRight() logic below (or swap the motor wiring).
    - STBY on the TB6612FNG is hardwired to 5V (always enabled),
      so no STBY pin control is needed in software.
    - analogWrite() is used for PWM speed control. This is supported
      natively on modern arduino-esp32 cores. If your installed core
      doesn't support analogWrite() on ESP32, swap to ledcAttach/ledcWrite.
  =======================================================================
*/

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
const int ECHO_PIN = 17;   // through voltage divider (5V -> 3.3V safe)

// --------------------------- TUNABLE SETTINGS --------------------------

const int DRIVE_SPEED            = 100;   // forward speed, PWM 0-255 (keep within ~80-120)
const int TURN_SPEED             = 95;    // turning speed, PWM 0-255
const int OBSTACLE_THRESHOLD_CM  = 15;    // distance that triggers avoidance

const unsigned long SENSOR_INTERVAL_MS = 100;  // how often to ping the sensor
const unsigned long BACKUP_DURATION_MS = 300;  // how long to reverse before turning
const unsigned long TURN_DURATION_MIN_MS = 350; // randomized turn duration range
const unsigned long TURN_DURATION_MAX_MS = 650;

const unsigned long ULTRASONIC_TIMEOUT_US = 30000; // ~5m max range timeout

// ------------------------------ STATE -----------------------------------

enum RobotState {
  STATE_FORWARD,
  STATE_BACKING_UP,
  STATE_TURNING
};

RobotState currentState = STATE_FORWARD;

unsigned long stateStartTime   = 0;  // when the current state began
unsigned long lastSensorCheck  = 0;  // last time we pinged the ultrasonic sensor
unsigned long turnDuration     = 0;  // randomized duration for the current turn

// =========================================================================
// SETUP
// =========================================================================

void setup() {
  Serial.begin(115200);

  setupMotors();
  setupUltrasonic();

  // Seed the random number generator using noise on an unused analog pin.
  randomSeed(analogRead(0));

  Serial.println("Travis: obstacle avoidance starting up.");

  currentState = STATE_FORWARD;
  moveForward(DRIVE_SPEED);
}

// =========================================================================
// MAIN LOOP (non-blocking state machine)
// =========================================================================

void loop() {
  unsigned long now = millis();

  switch (currentState) {

    case STATE_FORWARD:
      // Only ping the sensor every SENSOR_INTERVAL_MS instead of every loop.
      if (now - lastSensorCheck >= SENSOR_INTERVAL_MS) {
        lastSensorCheck = now;

        long distance = getDistanceCM();

        // distance < 0 means no echo / out of range -> treat as "clear"
        if (distance > 0 && distance < OBSTACLE_THRESHOLD_CM) {
          Serial.print("Obstacle detected at ");
          Serial.print(distance);
          Serial.println(" cm. Backing up.");

          stopMotors();
          moveBackward(DRIVE_SPEED);

          stateStartTime = now;
          currentState = STATE_BACKING_UP;
        }
      }
      break;

    case STATE_BACKING_UP:
      if (now - stateStartTime >= BACKUP_DURATION_MS) {
        stopMotors();

        // Randomly choose a turn direction and a randomized duration
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
      if (now - stateStartTime >= turnDuration) {
        stopMotors();
        moveForward(DRIVE_SPEED);
        currentState = STATE_FORWARD;
      }
      break;
  }

  // Future hooks (non-blocking, can run alongside the state machine):
  //   - updateOLEDFace();
  //   - updateSpeakerQueue();
  //   - pollBluetoothController();
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

// Drives a single TB6612 channel. speed range: -255..255 (sign = direction).
void driveMotor(int in1Pin, int in2Pin, int pwmPin, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    // Both low = coast/stop (use both HIGH for active brake if ever needed)
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

// Pivot turn: left motor reverses, right motor drives forward.
void turnLeft(int speed) {
  driveMotor(AIN1, AIN2, PWMA, -speed);
  driveMotor(BIN1, BIN2, PWMB, speed);
}

// Pivot turn: left motor drives forward, right motor reverses.
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

// Returns distance in centimeters, or -1 if no echo was received (timeout).
long getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long durationUs = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);

  if (durationUs == 0) {
    return -1; // timed out, no obstacle within range
  }

  // Speed of sound ~0.0343 cm/us, divide by 2 for round trip.
  long distanceCm = durationUs * 0.0343 / 2;
  return distanceCm;
}
