#include <SoftwareSerial.h>
#include "Wire.h" 
#include "MPU6050_6Axis_MotionApps20.h"
/*
////////////////////////////////////////
// == SETTINGS & TUNING ==
// ONLY EDIT THIS SECTION FOR TUNING
////////////////////////////////////////////////////////
*/
// -------------------- GYRO --------------------
MPU6050 mpu;
bool dmpReady = false;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
float currentYaw = 0; // Current yaw angle
unsigned long lastYawPrint = 0;
// --- 1. PINOUTS ---
// Set these pins to match wiring
// The Arduino interrupt pins are 2 and 3 i think, so encoders must be on these pins- just check if they are the interrupt pins.

// Motor Driver Pins (Right Side)
const int RIGHT_MOTOR_EN = 9;   // en1
const int RIGHT_MOTOR_IN1 = 8;  // in1-1
const int RIGHT_MOTOR_IN2 = 7;  // in1-2

// Motor Driver Pins (Left Side)
const int LEFT_MOTOR_EN = 11;  // en2
const int LEFT_MOTOR_IN1 = 12;  // in2-1
const int LEFT_MOTOR_IN2 = 13;  // in2-2

// Encoder Pins (must be 2 and 3 for interrupts-pls check)
const int RIGHT_ENCODER_PIN = 2; // encoder1
const int LEFT_ENCODER_PIN = 3;  // encoder2

// Sensor Arduino Communication (Uncomment the pair you are using)
 const int SENSOR_RX = 4; // Drive RX <- Sensor TX //(Same as before) 
 const int SENSOR_TX = A1; // Drive TX -> Sensor RX //(Same as before)
SoftwareSerial SensorSerial(SENSOR_RX, SENSOR_TX);


// --- 2. MOVEMENT TUNING ---
// These need to be tuned while testing
// 1. Set KP, KI, KD to 0.
// 2. Get COUNTS_PER_INCH: Send w0:12 from joon's testing code and measure how far it went, even if it curves it is okay keep open software serial and see //the encoder count. measure how much it actually moved then do below calculation. can see encoder count in serial monitor or can also add 
//Serial.println(g_countLeft); in the stopMotors() function
//    (COUNTS_PER_INCH = total_counts / inches_moved_actually)
// 3. Same way Get COUNTS_PER_DEGREE: Send r0:360 and see how many encoder counts it took.
//    (COUNTS_PER_DEGREE = total_counts / 360)
//Update the variables and reupload.
// 4. After that stuff is done to get straightness and stop smoothly-Tune PID values
//set KP_straight to a small value like 1.0 or 2.0.
//send w0:24 coz we need a long movement
//robot should now try to correct itself. if it wobbles or jiggle KP_straight is too high, if curving it is too low.

// Encoder Counts (Pulses)
// HOW MANY ENCODER PULSES = 1 INCH OF FORWARD TRAVEL
volatile long COUNTS_PER_INCH = 49; // Tune here!!!!!!

// Speeds (0-255)
const int DRIVE_SPEED = 100; // Base speed for w0 (0-255)
const int TURN_SPEED = 100;  // Base speed for r0 (0-255)

// --- 3. PID CONTROLLER TUNING ---
// These values control how the robot corrects its path.
// This is how to make it drive perfectly straight.

// PID for driving straight (corrects for one wheel being faster)
// Kp = "Proportional": How hard it corrects. Start with 1.0 or 2.0.
// Ki = "Integral": Fixes long-term drift. Start low (0.01).
// Kd = "Derivative": Smooths out oscillations. Start low (0.1).
const float KP_STRAIGHT = 0.8;
const float KI_STRAIGHT = 0;
const float KD_STRAIGHT = 0.1;

// -------------------- GYRO TURN PID --------------------
// PID for turning
const float KP_TURN = 1.2;      // Proportional gain for turning
const float KI_TURN = 0.0;     // Integral gain for turning
const float KD_TURN = 0.6;      // Derivative gain for turning

/*
//////////////////////////////////////////
// == END OF SETTINGS ==
// no editing below this line 
///////////////////////////////////////////////////
*/

// --- Global Encoder Counts ---
// Volatile = safe to use in interrupt routines
volatile long g_countRight = 0;
volatile long g_countLeft = 0;

// --- Encoder Interrupt Service Routines ---
void isrRightEncoder() { g_countRight++; }
void isrLeftEncoder() { g_countLeft++; }

// --- Reset Encoders ---
void resetCounts() {
  g_countRight = 0;
  g_countLeft = 0;
}

// ---  Motor Control ---
void setRightMotor(int speed, int dir) {
  // dir 1 = fwd, -1 = rev, 0 = stop
  if (dir == 1) {
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
  } else if (dir == -1) {
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);

  } else if (dir == 2) {
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);  // FOR BRAKES
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);

  } else {
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
  }
  analogWrite(RIGHT_MOTOR_EN, abs(speed));
}

void setLeftMotor(int speed, int dir) {
  // dir 1 = fwd, -1 = rev, 0 = stop
  // NOTE: Your old code had -1 for forward. We assume 1 is forward.
  // If your left motor spins backward, swap LOW/HIGH here.
  if (dir == 1) {
    digitalWrite(LEFT_MOTOR_IN1, HIGH); // Was LOW in old code
    digitalWrite(LEFT_MOTOR_IN2, LOW);  // Was HIGH in old code
  } else if (dir == -1) {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);

  } else if (dir == 2) {
    digitalWrite(LEFT_MOTOR_IN1, HIGH);  // FOR BRAKES
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    
  } else {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
  }
  analogWrite(LEFT_MOTOR_EN, abs(speed));
}

void stopMotors() {
  setRightMotor(255, 2); //brakes before a stop
  setLeftMotor(255, 2);
  delay(300);
  setRightMotor(0, 0);
  setLeftMotor(0, 0);
  SensorSerial.println("true"); // Tell Sensor/Python we are ready for next command
}

// --- Hard Stop Check ---
// Checks if Sensor Arduino sent "xx"
bool checkForHardStop() {
  if (SensorSerial.available() > 0) {
    String cmd = SensorSerial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "xx") {
      stopMotors();
      return true;
    }
  }
  return false;
}

// --- Closed-Loop Movement:Drive Straight ---
// This function using PID controller to drive straight.
void moveStraight(int inches) {
  SensorSerial.println("false"); // Tell Sensor/Python we are busy
  resetCounts();

  long targetCounts = abs(inches) * COUNTS_PER_INCH;
  int direction = (inches >= 0) ? 1 : -1;

  float integral = 0;
  float lastError = 0;

  // -----------------------------
  //   SLOW START RAMP (NEW)
  // -----------------------------
  const int START_SPEED = 66;       // speed at beginning
  const int RAMP_END_SPEED = DRIVE_SPEED;  
  const int RAMP_TIME = 300;        // total ramp time in ms
  const int RAMP_STEPS = 15;

  for (int i = 0; i < RAMP_STEPS; i++) {
    float t = (float)i / (float)RAMP_STEPS;

    int rampSpeed = START_SPEED + t * (RAMP_END_SPEED - START_SPEED);
    rampSpeed = constrain(rampSpeed, 0, 255);

    setRightMotor(rampSpeed, direction);
    setLeftMotor(rampSpeed, direction);

    delay(RAMP_TIME / RAMP_STEPS);
  }

  // -----------------------------
  //   MAIN PID LOOP (UNCHANGED)
  // -----------------------------
  while (g_countLeft < targetCounts || g_countRight < targetCounts) {
    if (checkForHardStop()) return;

    float error = g_countLeft - g_countRight;
    integral += error;
    float derivative = error - lastError;
    lastError = error;

    float correction = (KP_STRAIGHT * error) +
                       (KI_STRAIGHT * integral) +
                       (KD_STRAIGHT * derivative);

    int rightSpeed = DRIVE_SPEED + (int)correction;
    int leftSpeed  = DRIVE_SPEED - (int)correction;

    rightSpeed = constrain(rightSpeed, 0, 255);
    leftSpeed  = constrain(leftSpeed, 0, 255);

    setRightMotor(rightSpeed, direction);
    setLeftMotor(leftSpeed, direction);

    if ((g_countLeft + g_countRight) / 2 >= targetCounts)
      break;

    delay(10);
  }

  stopMotors();
}

// --- Closed-Loop Movement: Turn in Place (IMPROVED) ---
void turnAngle(int targetDegrees) {
  SensorSerial.println("false"); // Tell Python we're busy

  // ----------------- PID VARIABLES -----------------
  float turnError = 0;
  float lastTurnError = 0;
  float turnIntegral = 0;

  // ----------------- CALCULATE TARGET YAW -----------------
  float targetYaw = currentYaw + targetDegrees;
  if (targetYaw >= 360) targetYaw -= 360;
  if (targetYaw < 0) targetYaw += 360;

  // PID mapping parameters
  const int MIN_TURN_SPEED = 52;     // minimum motor speed for movement
  const float TOLERANCE = 2.5;       // degrees - stop when within this range
  const float FINE_THRESHOLD = 15.0; // switch to fine control below this error
  const int FINE_SPEED = 70;         // slower speed for final approach
  
  unsigned long lastDebugPrint = 0;
  unsigned long stableStartTime = 0;
  const unsigned long STABLE_TIME = 200; // ms to stay stable before accepting
  bool wasStable = false;

  // Reset integral for fresh start
  turnIntegral = 0;

  while (true) {
      // ----------------- FORCE STOP -----------------
      if (checkForHardStop()) {
          stopMotors();
          return;
      }

      // ----------------- READ GYROSCOPE -----------------
      if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          currentYaw = ypr[0] * 180 / M_PI;
          if (currentYaw < 0) currentYaw += 360;
      }

      // ----------------- COMPUTE ERROR (WRAP AROUND) -----------------
      turnError = targetYaw - currentYaw;
      if (turnError > 180) turnError -= 360;
      if (turnError < -180) turnError += 360;

      // ----------------- DEBUG PRINT EVERY 500ms -----------------
      if (millis() - lastDebugPrint >= 500) {
          Serial.print("Yaw: ");
          Serial.print(currentYaw);
          Serial.print(" | Target: ");
          Serial.print(targetYaw);
          Serial.print(" | Error: ");
          Serial.println(turnError);
          lastDebugPrint = millis();
      }

      // ----------------- CHECK IF STABLE -----------------
      if (abs(turnError) < TOLERANCE) {
          if (!wasStable) {
              stableStartTime = millis();
              wasStable = true;
          } else if (millis() - stableStartTime >= STABLE_TIME) {
              // Been stable long enough - we're done
              break;
          }
      } else {
          wasStable = false;
      }

      // ----------------- PID TERMS -----------------
      // Anti-windup: only accumulate integral if error is significant
      if (abs(turnError) > TOLERANCE && abs(turnError) < 45) {
          turnIntegral += turnError;
          // Clamp integral to prevent windup
          turnIntegral = constrain(turnIntegral, -500, 500);
      } else if (abs(turnError) <= TOLERANCE) {
          turnIntegral *= 0.5; // decay integral when close
      }
      
      float derivative = turnError - lastTurnError;
      lastTurnError = turnError;

      float output = KP_TURN * turnError +
                      KI_TURN * turnIntegral +
                      KD_TURN * derivative;

      // ----------------- MOTOR SPEED CALCULATION -----------------
      int motorSpeed;
      
      if (abs(turnError) < FINE_THRESHOLD) {
          // Fine control mode - use slower, more precise speed
          motorSpeed = map(abs(output), 0, 100, MIN_TURN_SPEED, FINE_SPEED);
          motorSpeed = constrain(motorSpeed, MIN_TURN_SPEED, FINE_SPEED);
      } else {
          // Normal mode - full speed range
          motorSpeed = map(abs(output), 0, 255, MIN_TURN_SPEED, TURN_SPEED);
          motorSpeed = constrain(motorSpeed, MIN_TURN_SPEED, TURN_SPEED);
      }

      // Deadband - don't move if error is tiny
      if (abs(turnError) < 0.5) {
          setRightMotor(0, 0);
          setLeftMotor(0, 0);
      }
      // ----------------- MOTOR OUTPUT WITH DIRECTION -----------------
      else if (turnError > 0) {   // turn CW (right)
          setRightMotor(motorSpeed, 1);   // right forward
          setLeftMotor(motorSpeed, -1);   // left backward
      } else {                    // turn CCW (left)
          setRightMotor(motorSpeed, -1);  // right backward
          setLeftMotor(motorSpeed, 1);    // left forward
      }

      delay(10);
  }

  stopMotors();
}
/* =================== Setup / Loop =================== */
void setup() {
  Serial.begin(9600);           // USB Debug only
  SensorSerial.begin(9600);     // Communication with Sensor Arduino

  // Motor pins
  pinMode(RIGHT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);

  // Encoder pins (Must be 2 and 3 for interrupts)
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), isrRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), isrLeftEncoder, RISING);

  Serial.println("Drive Arduino V3 (Encoder Control) Ready.");

  // Gyroscope Setup 
  Wire.begin();
  Serial.println("Initializing MPU...");
  mpu.initialize();
  if (!mpu.testConnection()) {
      Serial.println("MPU6050 connection failed");
      while (1);
  }

  uint8_t devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(-1);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
      mpu.setDMPEnabled(true);
      dmpReady = true;
      Serial.println("DMP ready!");
  } else {
      Serial.print("DMP init failed: "); Serial.println(devStatus);
      while (1);
  }

}

void loop() {
     // --- Read gyro ---
  if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      currentYaw = ypr[0] * 180 / M_PI;
      if (currentYaw < 0) currentYaw += 360; // optional wrap
  }

  // --- Print yaw every second ---
  if (millis() - lastYawPrint >= 1000) {
      lastYawPrint = millis();
      Serial.print("Yaw: ");
      Serial.println(currentYaw);
  }

  String command = "";
  // if (Serial.available() > 0) {
  //   command = Serial.readStringUntil('\n'); /// DEBUGGING PURPOSE
  if (SensorSerial.available() > 0) {
    command = SensorSerial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();

    Serial.print("Received from Sensor: ");
    Serial.println(command);
  }

  // --- New Encoder-based Commands ---
  if (command.startsWith("w0:")) {
    int inches = command.substring(3).toInt();
    moveStraight(inches);
  }
  else if (command.startsWith("r0:")) {
    int degrees = command.substring(3).toInt();
    turnAngle(degrees);
  }
  
  // --- Old Gyro commands are removed since using encoders for pid instead of gyro now.
  // not handling dl and dr since no longer doing strafe
}
