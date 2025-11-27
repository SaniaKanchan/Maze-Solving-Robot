#include <SoftwareSerial.h>
#include <NewPing.h>
#include <Servo.h>

/* ============================================================
   === PACKET CONSTANTS (SimMeR-compatible) ====================
   ============================================================ */
char FRAMESTART = '[';
char FRAMEEND   = ']';
int TIMEOUT      = 250;
int MAX_PACKET_LENGTH = 200;

/* ============================================================
   === SERVO CONTROL ==========================================
   ============================================================ */
Servo leftServo;
Servo rightServo;
Servo clawServo;

const int LEFT_SERVO_PIN  = 6;
const int RIGHT_SERVO_PIN = 5;
const int CLAW_SERVO_PIN  = 3;

const int LEFT_HOME  = 100;
const int RIGHT_HOME = 120;

const int DOWN_OFFSET = 80;
const int UP_OFFSET   = 0;

const int CLAW_OPEN_ANGLE  = 20;
const int CLAW_CLOSE_ANGLE = 110;

void moveRack(int offset){
  int leftAngle  = constrain(LEFT_HOME  + offset, 0, 180);
  int rightAngle = constrain(RIGHT_HOME - offset, 0, 180);

  leftServo.write(leftAngle);
  rightServo.write(rightAngle);
}

void basicPickUp(){
  clawServo.write(CLAW_OPEN_ANGLE);
  delay(300);

  moveRack(DOWN_OFFSET);
  delay(800);

  clawServo.write(CLAW_CLOSE_ANGLE);
  delay(800);

  moveRack(UP_OFFSET);
}

void basicDrop(){
  clawServo.write(CLAW_OPEN_ANGLE);
}

/* ============================================================
   === DRIVE ARDUINO COMMUNICATION =============================
   ============================================================ */
const int DRIVE_RX = 4;
const int DRIVE_TX = A1;
SoftwareSerial DriveSerial(DRIVE_RX, DRIVE_TX);

bool driveEnabled = true;

/* ============================================================
   === ULTRASONIC SETUP ========================================
   ============================================================ */
#define MAX_DISTANCE 200
const float US_TO_INCH = 0.0135;

const int trigPinMain = 7;
const int trigPinSide = A0;

const int echoPins[6] = {8,9,10,11,12,13};

NewPing sonar[6] = {
  NewPing(trigPinMain, echoPins[0], MAX_DISTANCE),
  NewPing(trigPinMain, echoPins[1], MAX_DISTANCE),
  NewPing(trigPinSide, echoPins[2], MAX_DISTANCE),
  NewPing(trigPinMain, echoPins[3], MAX_DISTANCE),
  NewPing(trigPinSide, echoPins[4], MAX_DISTANCE),
  NewPing(trigPinMain, echoPins[5], MAX_DISTANCE)
};

const int blockTrigPin = A2;
const int blockEchoPin = A3;
NewPing blockSensor(blockTrigPin, blockEchoPin, MAX_DISTANCE);

float readUltrasonic(int sensorNum){ // Takes the Median of 9 Readings
    if (sensorNum < 0 || sensorNum > 5) return -1.0;

    float readings[9];
    
    // Take 9 readings
    for (int i = 0; i < 9; i++) {
        unsigned long pingTime = sonar[sensorNum].ping();
        float dist_in = (pingTime / 2.0) * US_TO_INCH;
        readings[i] = (dist_in == 0) ? -1.0 : dist_in;
        delay(5); // small delay between pings to avoid crosstalk
    }

    // Sort the array to get median
    for (int i = 0; i < 8; i++) {
        for (int j = i + 1; j < 9; j++) {
            if (readings[j] < readings[i]) {
                float temp = readings[i];
                readings[i] = readings[j];
                readings[j] = temp;
            }
        }
    }

    // Return the median (5th value in sorted array)
    return readings[4];
}

float readBlockSensor(){
  unsigned long t = blockSensor.ping();
  float d = (t/2.0) * US_TO_INCH;
  return d == 0 ? -1.0 : d;
}

/* ============================================================
   === PACKET FUNCTIONS (receive, depacketize, parse, etc.) ===
   ============================================================ */

String receiveSerial() {
  String front = "";
  String msg = "";

  if (!Serial.available()) return "";

  unsigned long startTime = millis();
  while (millis() < startTime + TIMEOUT) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == FRAMESTART) {
        msg += c;
        break;
      } else {
        front += c;
      }
    }
  }

  while (millis() < startTime + TIMEOUT) {
    if (Serial.available()) {
      char c = Serial.read();
      msg += c;
      if (c == FRAMEEND) break;
    }
  }

  if (msg.length() < 2) return "";
  if (msg[0] != FRAMESTART || msg[msg.length()-1] != FRAMEEND) return "";

  return msg.substring(1, msg.length()-1);
}

String packetize(String s){
  return String(FRAMESTART) + s + String(FRAMEEND);
}

String parseCmd(String cmd){
  
  // ---------------- ULTRASONIC u0–u5 ----------------
  if (cmd == "u0") { float d = readUltrasonic(0); return "u0:" + String(d, 2); }
  if (cmd == "u1") { float d = readUltrasonic(1); return "u1:" + String(d, 2); }
  if (cmd == "u2") { float d = readUltrasonic(2); return "u2:" + String(d, 2); }
  if (cmd == "u3") { float d = readUltrasonic(3); return "u3:" + String(d, 2); }
  if (cmd == "u4") { float d = readUltrasonic(4); return "u4:" + String(d, 2); }
  if (cmd == "u5") { float d = readUltrasonic(5); return "u5:" + String(d, 2); }

  // ---------------- BLOCK SENSOR ub ----------------
  if (cmd == "ub") { float d = readBlockSensor(); return "ub:" + String(d, 2); }

  // ---------------- PICKUP / DROP ------------------
  if (cmd == "bp") { basicPickUp(); return "bp:True"; }
  if (cmd == "bd") { basicDrop(); return "bd:True"; }
  // ---------------- DRIVE COMMANDS ------------------
  if (cmd == "xx") { 
      DriveSerial.println(cmd); 
      return "xx:FORCE STOP"; 
  }
  if (cmd.startsWith("w0:") || cmd.startsWith("r0:")) { 
      // forward to Drive Arduino
      DriveSerial.println(cmd);  
      // strip everything after the first colon
      int colonIndex = cmd.indexOf(':');
      String cmdID = (colonIndex >= 0) ? cmd.substring(0, colonIndex) : cmd;
      return cmdID + ":" + (driveEnabled ? "True" : "False"); 
  }
  // DRIVE STATUS (no colon, don't forward)
  if (cmd == "w0" || cmd == "r0") { 
      return cmd + ":" + (driveEnabled ? "True" : "False"); 
  }
  // default response if nothing matches
  return cmd + ":Not Found";
} // <-- close parseCmd() here

String parsePacket(String pkt){
  String result = "";
  int start = 0;

  for (int i=0; i<pkt.length(); i++){
    if (pkt[i] == ',') {
      String segment = pkt.substring(start, i);
      result += parseCmd(segment) + ",";
      start = i+1;
    }
  }

  result += parseCmd(pkt.substring(start));
  return result;
}


/* ============================================================
   === SETUP ===================================================
   ============================================================ */

void setup(){
  Serial.begin(9600);
  DriveSerial.begin(9600);

  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);
  clawServo.attach(CLAW_SERVO_PIN);

  leftServo.write(LEFT_HOME);
  rightServo.write(RIGHT_HOME);
  clawServo.write(CLAW_OPEN_ANGLE);

  pinMode(trigPinMain, OUTPUT);
  pinMode(trigPinSide, OUTPUT);
  for (int i=0; i<6; i++) pinMode(echoPins[i], INPUT);

  pinMode(blockTrigPin, OUTPUT);
  pinMode(blockEchoPin, INPUT);

  Serial.println("Sensor Arduino Ready");
}

/* ============================================================
   === MAIN LOOP ==============================================
   ============================================================ */

void loop(){

  String pkt = receiveSerial();

  if (pkt.length()) {
    String resp = parsePacket(pkt);
    Serial.print(packetize(resp));
  }

  if (DriveSerial.available()){
    String reply = DriveSerial.readStringUntil('\n');
    reply.trim();
    if      (reply == "true")  driveEnabled = true;
    else if (reply == "false") driveEnabled = false;
  }
}
