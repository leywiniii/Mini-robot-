#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include <math.h>

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

// MAX98357A I2S Audio Amplifier
const int SPK_DIN_PIN  = 23;
const int SPK_BCLK_PIN = 18;
const int SPK_LRC_PIN  = 19;
const uint32_t AUDIO_SAMPLE_RATE = 16000;  

i2s_chan_handle_t audioTxHandle;

// --------------------------- TUNABLE SETTINGS --------------------------

const int DRIVE_SPEED            = 120;   
const int TURN_SPEED             = 170;    
const int OBSTACLE_THRESHOLD_CM  = 25;    
const int WAKE_TRIGGER_DISTANCE_CM = 10;  

const unsigned long SENSOR_INTERVAL_MS = 100;  
const unsigned long BACKUP_DURATION_MS = 450;  
const unsigned long TURN_DURATION_MIN_MS = 460; 
const unsigned long TURN_DURATION_MAX_MS = 650;

const unsigned long ULTRASONIC_TIMEOUT_US = 30000; 

// Siren sound (plays while driving forward)
const int   SIREN_CHUNK_SAMPLES         = 200;     
const float SIREN_LOW_FREQ              = 600.0f;  
const float SIREN_HIGH_FREQ             = 1200.0f; 
const uint32_t SIREN_SWEEP_PERIOD_SAMPLES = (uint32_t)(AUDIO_SAMPLE_RATE * 1.2f); 
const int16_t SIREN_AMPLITUDE           = 6000;    

//"Ouch" yelp sound effect (plays once when an obstacle is detected) 
const float OUCH_START_FREQ   = 900.0f;  
const float OUCH_END_FREQ     = 250.0f;  
const int   OUCH_DURATION_MS  = 180;
const int16_t OUCH_AMPLITUDE  = 9000;

const float TWO_PI_F = 6.283185307f;

// ------------------------------ STATE -----------------------------------

enum RobotState {
  STATE_WAITING,    
  STATE_FORWARD,
  STATE_BACKING_UP,
  STATE_TURNING
};

RobotState currentState = STATE_WAITING;

unsigned long stateStartTime   = 0; 
unsigned long lastSensorCheck  = 0;
unsigned long turnDuration     = 0;  

// OLED face state 
enum FaceExpression {
  FACE_HAPPY,
  FACE_SAD
};

bool oledReady = false;                      
FaceExpression lastDrawnFace = FACE_SAD;       

// Siren generator state
float sirenPhase            = 0.0f;  
uint32_t sirenSweepCounter  = 0; 

// =========================================================================
// SETUP
// =========================================================================

void setup() {
  Serial.begin(115200);

  setupMotors();
  setupUltrasonic();
  setupOLED();
  setupSpeaker();

  // Seed the random number generator using noise on an unused analog pin.
  randomSeed(analogRead(0));

  Serial.println("Travis: powered up. Waiting for something nearby before moving.");

  currentState = STATE_WAITING;
  updateFace(FACE_HAPPY);
}

// =========================================================================
// MAIN LOOP (non-blocking state machine)
// =========================================================================

void loop() {
  unsigned long now = millis();

  switch (currentState) {

    case STATE_WAITING:
      updateFace(FACE_HAPPY);

      if (now - lastSensorCheck >= SENSOR_INTERVAL_MS) {
        lastSensorCheck = now;

        long distance = getDistanceCM();

        if (distance > 0 && distance < WAKE_TRIGGER_DISTANCE_CM) {
          Serial.print("Something detected at ");
          Serial.print(distance);
          Serial.println(" cm. Waking up and moving forward.");

          moveForward(DRIVE_SPEED);
          currentState = STATE_FORWARD;
        }
      }
      break;

    case STATE_FORWARD:
      updateFace(FACE_HAPPY);
      updateSirenChunk(); 

      if (now - lastSensorCheck >= SENSOR_INTERVAL_MS) {
        lastSensorCheck = now;

        long distance = getDistanceCM();

        if (distance > 0 && distance < OBSTACLE_THRESHOLD_CM) {
          Serial.print("Obstacle detected at ");
          Serial.print(distance);
          Serial.println(" cm. Backing up.");

          updateFace(FACE_SAD);
          stopSiren();
          playOuchEffect();
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

// Returns distance in centimeters, or -1 if no echo was received (timeout).
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

  // Speed of sound ~0.0343 cm/us, divide by 2 for round trip.
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

// Draws two round eyes plus a smile (happy) or frown (sad) mouth.
void drawFace(FaceExpression expression) {
  display.clearDisplay();

  //Eyes: simple filled circles, same for both expressions 
  const int eyeRadius = 8;
  const int leftEyeX   = 44;
  const int rightEyeX  = 84;
  const int eyeY       = 24;

  display.fillCircle(leftEyeX, eyeY, eyeRadius, SSD1306_WHITE);
  display.fillCircle(rightEyeX, eyeY, eyeRadius, SSD1306_WHITE);

  //Mouth: curve traced point-by-point, connected with short lines 
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

// =========================================================================
// SPEAKER FUNCTIONS (MAX98357A via I2S, synthesized sounds)
// =========================================================================

void setupSpeaker() {
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chanCfg, &audioTxHandle, NULL);

  i2s_std_config_t stdCfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)SPK_BCLK_PIN,
      .ws   = (gpio_num_t)SPK_LRC_PIN,
      .dout = (gpio_num_t)SPK_DIN_PIN,
      .din  = I2S_GPIO_UNUSED,
    },
  };

  i2s_channel_init_std_mode(audioTxHandle, &stdCfg);
  i2s_channel_enable(audioTxHandle);
}

const int MAX_AUDIO_CHUNK = 256;

void writeAudioSamples(const int16_t *monoSamples, int sampleCount) {
  static int16_t stereoBuffer[MAX_AUDIO_CHUNK * 2];

  if (sampleCount > MAX_AUDIO_CHUNK) sampleCount = MAX_AUDIO_CHUNK;

  for (int i = 0; i < sampleCount; i++) {
    stereoBuffer[i * 2]     = monoSamples[i];
    stereoBuffer[i * 2 + 1] = monoSamples[i];
  }

  size_t bytesWritten;
  i2s_channel_write(audioTxHandle, stereoBuffer, sampleCount * 2 * sizeof(int16_t),
                     &bytesWritten, portMAX_DELAY);
}

void updateSirenChunk() {
  int16_t buffer[SIREN_CHUNK_SAMPLES];

  for (int i = 0; i < SIREN_CHUNK_SAMPLES; i++) {
    // Triangle wave between 0 and 1 over SIREN_SWEEP_PERIOD_SAMPLES
    float t = (float)(sirenSweepCounter % SIREN_SWEEP_PERIOD_SAMPLES) / SIREN_SWEEP_PERIOD_SAMPLES;
    float triangle = (t < 0.5f) ? (t * 2.0f) : (2.0f - t * 2.0f);
    float freq = SIREN_LOW_FREQ + triangle * (SIREN_HIGH_FREQ - SIREN_LOW_FREQ);

    sirenPhase += TWO_PI_F * freq / AUDIO_SAMPLE_RATE;
    if (sirenPhase > TWO_PI_F) sirenPhase -= TWO_PI_F;

    buffer[i] = (int16_t)(sinf(sirenPhase) * SIREN_AMPLITUDE);
    sirenSweepCounter++;
  }

  writeAudioSamples(buffer, SIREN_CHUNK_SAMPLES);
}

void stopSiren() {
  int16_t silence[64] = {0};
  writeAudioSamples(silence, 64);
}

void playOuchEffect() {
  const int totalSamples = (AUDIO_SAMPLE_RATE * OUCH_DURATION_MS) / 1000;
  const int chunkSize = 200;

  float phase = 0.0f;
  int16_t buffer[chunkSize];
  int sampleIndex = 0;

  while (sampleIndex < totalSamples) {
    int samplesThisChunk = min(chunkSize, totalSamples - sampleIndex);

    for (int i = 0; i < samplesThisChunk; i++) {
      float progress = (float)sampleIndex / totalSamples; 
      float freq = OUCH_START_FREQ + (OUCH_END_FREQ - OUCH_START_FREQ) * progress;

      phase += TWO_PI_F * freq / AUDIO_SAMPLE_RATE;
      if (phase > TWO_PI_F) phase -= TWO_PI_F;

      buffer[i] = (int16_t)(sinf(phase) * OUCH_AMPLITUDE);
      sampleIndex++;
    }

    writeAudioSamples(buffer, samplesThisChunk);
  }
}


