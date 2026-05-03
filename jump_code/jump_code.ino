#include <Servo.h>
#include <Wire.h>

Servo myServo;
Servo triggerServo;

const int servoPin        = 9;
const int triggerServoPin = 6;

// Trigger servo positions (degrees)
const int           TRIGGER_DEFAULT    = 180;  // resting position
const int           TRIGGER_FIRED      = 90;   // release position
const unsigned long TRIGGER_RETURN_MS  = 1000; // ms before returning to default
bool          triggerActive    = false;
unsigned long triggerFiredTime = 0;
bool          reverseSpool     = false;  // drives main servo back during trigger window - needed for version after one-way bearing broke

const int  HALL_PIN        = A0;
const int  HALL_SAMPLES    = 16;
const int  HALL_NO_MAGNET  = 215;  // ADC centre, no-magnet state (Hardcode ADC values, dynamic sampling extremely unreliable
const int  HALL_MAGNET     = 141;  // ADC centre, magnet-present state
const int  HALL_TOLERANCE  = 25;   // +-counts around each centre

// Delayed stop after magnet detection, otherwise noise stops robot
const unsigned long MAGNET_STOP_DELAY = 1000;
bool          magnetDetected     = false;
unsigned long magnetDetectedTime = 0;

// Continuous servo pulse widths (us)
int STOP = 1500;     // adjust calib screw
const int SLOW_CW  = 1450;
const int SLOW_CCW = 1550;
const int FAST_CW  = 1300;
const int FAST_CCW = 1700;

int currentSignal = STOP;

// MPU6050
const uint8_t MPU_ADDR = 0x68; 

// Jump recording
bool recording = false;
unsigned long recordingStart = 0;
const unsigned long RECORD_DURATION = 2000;

void initMPU6050() {
  // Wake up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // Clear sleep bit
  Wire.endTransmission(true);
  // +-16g range (AFS_SEL = 3), scale = 2048 LSB/g --> Still clips
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x18);
  Wire.endTransmission(true);
}

void readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)6, (uint8_t)true);
  ax = (int16_t)(Wire.read() << 8 | Wire.read());
  ay = (int16_t)(Wire.read() << 8 | Wire.read());
  az = (int16_t)(Wire.read() << 8 | Wire.read());
}

int hallRead() {
  long sum = 0;
  for (int i = 0; i < HALL_SAMPLES; i++) {
    sum += analogRead(HALL_PIN);
    delayMicroseconds(200);
  }
  return (int)(sum / HALL_SAMPLES);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // 400 kHz fast-mode I2C
  initMPU6050();
  myServo.attach(servoPin);
  myServo.writeMicroseconds(STOP);
  triggerServo.attach(triggerServoPin);
  triggerServo.write(TRIGGER_DEFAULT);
  Serial.println(F("# Ready #"));
}

void loop() {
  // Check for key press
  if (Serial.available() > 0) {
    char key = Serial.read();

    switch (key) {
      case 'q': currentSignal = SLOW_CCW; break; // Dont really need this
      case 'e': currentSignal = SLOW_CW;  break; // Dont really need this
      case 'a': currentSignal = FAST_CCW; break;
      case 'd':
        triggerServo.write(TRIGGER_FIRED);
        triggerActive    = true;
        triggerFiredTime = millis();
        currentSignal    = FAST_CW;
        reverseSpool     = true;
        recording        = true;
        recordingStart   = millis();
        Serial.println(F("t_ms,ax,ay,az"));
        break;
      case 's':
        currentSignal  = STOP;
        magnetDetected = false;  // cancel delayed stop
        break;
      default: break; // ignore other keys
    }
  }

  // Trigger servo auto-return after TRIGGER_RETURN_MS
  if (triggerActive && (millis() - triggerFiredTime >= TRIGGER_RETURN_MS)) {
    triggerServo.write(TRIGGER_DEFAULT);
    triggerActive = false;
  }

  // Stop main spool-back once trigger window expires
  if (reverseSpool && !triggerActive) {
    currentSignal = STOP;
    reverseSpool  = false;
  }

  // Hall sensor — only while spooling, ignore when jumping
  if (currentSignal != STOP && !recording) {
    int hallVal     = hallRead();
    bool inNoMagnet = abs(hallVal - HALL_NO_MAGNET) <= HALL_TOLERANCE;
    bool inMagnet   = abs(hallVal - HALL_MAGNET)    <= HALL_TOLERANCE;

    if (inMagnet && !magnetDetected) {
      magnetDetected     = true;
      magnetDetectedTime = millis();
      Serial.print(F("# magnet detected (hall="));
      Serial.print(hallVal);
      Serial.print(F("), stopping in "));
      Serial.print(MAGNET_STOP_DELAY);
      Serial.println(F(" ms"));
    } else if (inNoMagnet && magnetDetected) {
      magnetDetected = false;
      Serial.println(F("# magnet lost, stop cancelled"));
    }
    // readings outside both ranges are ignored (noise/outliers)
  }

  // Execute delayed stop (only if still spooling)
  if (magnetDetected && currentSignal != STOP &&
      (millis() - magnetDetectedTime >= MAGNET_STOP_DELAY)) {
    currentSignal  = STOP;
    magnetDetected = false;
    Serial.println(F("# stopped after magnet delay"));
  }

  // Continuously send signal
  myServo.writeMicroseconds(currentSignal);

  // Stream accel data during recording (~500 Hz with 400kHz I2C)
  if (recording) {
    unsigned long elapsed = millis() - recordingStart;
    if (elapsed <= RECORD_DURATION) {
      int16_t ax, ay, az;
      readAccel(ax, ay, az);
      Serial.print(elapsed);
      Serial.print(',');
      Serial.print(ax);
      Serial.print(',');
      Serial.print(ay);
      Serial.print(',');
      Serial.println(az);
      delay(2); // ~500 Hz
    } else {
      recording = false;
      Serial.println("# recording complete #");
    }
  } else {
    delay(20); // 50 Hz servo refresh when not recording
  }
}