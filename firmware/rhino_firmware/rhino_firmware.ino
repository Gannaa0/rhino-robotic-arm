/*
 * Rhino - Industrial 5-DOF Robotic Arm for Pharmaceutical Filling-Line Automation
 * Arduino Mega 2560 firmware
 *
 * Reconstructed from the project's graduation documentation. Implements the
 * documented architecture:
 *   - serial command interpreter (115200 baud): SIM, LIVE, HOME, HOME1,
 *     XYZ x y z, J1..J4 <deg>, L <cm>, G <deg>, GO, CYCLE, RUN, STOP, ESTOP
 *   - geometric inverse kinematics (law of cosines, wrist kept level)
 *   - non-blocking motion on six TB6600-driven steppers via AccelStepper
 *   - limit-switch homing, IR auto-trigger, E-stop handling
 *
 * Conventions (must match the dashboard):
 *   World frame: X = side (along the track), Y = forward, Z = up (cm).
 *   J1 slider 0..260  <=> base yaw -130..+130 deg (theta1 = atan2(Y, X)).
 *   J2 = elevation of link L2 from horizontal (90 = vertical, "ruler" pose).
 *   J3 = 90 + signed elbow bend relative to L2.
 *   J4 = 90 + absolute wrist elevation (90 = gripper level).
 *   L  = track position in cm (0..140). G = gripper angle (70 open..160 closed).
 */

#include <AccelStepper.h>
#include <Servo.h>

// ---------------- Mechanical constants (from the documentation) ------------
const float L1 = 23.2;   // base height (cm)
const float L2 = 22.0;   // shoulder link (cm)
const float L3 = 22.5;   // elbow link (cm)
const float L4 = 13.5;   // wrist link (cm)
const float TRACK_MIN = 0.0, TRACK_MAX = 140.0;
const float MAX_REACH = L2 + L3;       // 44.5 cm
const float MIN_REACH = abs(L2 - L3);

// ---------------- Pin map (adjust to the actual wiring) --------------------
// TB6600 STEP/DIR pairs: track, base, shoulder A+B (dual), elbow, wrist
const uint8_t PIN_STEP_TRACK = 22, PIN_DIR_TRACK = 23;
const uint8_t PIN_STEP_J1    = 24, PIN_DIR_J1    = 25;
const uint8_t PIN_STEP_J2A   = 26, PIN_DIR_J2A   = 27;
const uint8_t PIN_STEP_J2B   = 28, PIN_DIR_J2B   = 29;
const uint8_t PIN_STEP_J3    = 30, PIN_DIR_J3    = 31;
const uint8_t PIN_STEP_J4    = 32, PIN_DIR_J4    = 33;
const uint8_t PIN_SERVO      = 9;    // gripper servo (PWM)
const uint8_t PIN_LIM_TRACK  = 34;   // limit switches to GND (INPUT_PULLUP)
const uint8_t PIN_LIM_J1     = 35;
const uint8_t PIN_LIM_J2     = 36;
const uint8_t PIN_LIM_J3     = 37;
const uint8_t PIN_LIM_J4     = 38;
const uint8_t PIN_IR         = 39;   // IR bottle sensor at the pickup station
const uint8_t PIN_ESTOP      = 2;    // hardware E-stop (interrupt capable)

// ---------------- Motion calibration (measure on the real build) -----------
// TB6600 set to 8x microstepping -> 1600 steps/rev at the motor shaft.
const float MICROSTEPS = 8.0;
const float MOTOR_STEPS_REV = 200.0 * MICROSTEPS;   // 1600
// T5 belt reduction per joint (motor revs per joint rev) - CALIBRATE:
const float RED_TRACK = 1.0;    // track: 20T pulley, 5 mm pitch -> 100 mm/rev
const float RED_J1 = 3.0;
const float RED_J2 = 3.0;
const float RED_J3 = 3.0 * 5.0; // elbow motor has a 5:1 planetary gearbox
const float RED_J4 = 3.0;

const float STEPS_PER_CM_TRACK = (MOTOR_STEPS_REV * RED_TRACK) / 10.0;  // 10 cm/rev
const float STEPS_PER_DEG_J1 = (MOTOR_STEPS_REV * RED_J1) / 360.0;
const float STEPS_PER_DEG_J2 = (MOTOR_STEPS_REV * RED_J2) / 360.0;
const float STEPS_PER_DEG_J3 = (MOTOR_STEPS_REV * RED_J3) / 360.0;
const float STEPS_PER_DEG_J4 = (MOTOR_STEPS_REV * RED_J4) / 360.0;

// Gripper servo angles (documentation: 70 = open, 160 = closed)
const float GRIP_OPEN = 70.0, GRIP_CLOSED = 160.0;

// Pick & place cell coordinates, world frame cm - CALIBRATE to the cell:
const float PICK_X = 30.0,  PICK_Y = 25.0,  PICK_Z = 5.0;
const float PLACE_X = 110.0, PLACE_Y = 25.0, PLACE_Z = 5.0;
const float APPROACH_LIFT = 15.0;   // cm above the station for approach

// ---------------- Hardware objects -----------------------------------------
AccelStepper stTrack(AccelStepper::DRIVER, PIN_STEP_TRACK, PIN_DIR_TRACK);
AccelStepper stJ1(AccelStepper::DRIVER, PIN_STEP_J1, PIN_DIR_J1);
AccelStepper stJ2A(AccelStepper::DRIVER, PIN_STEP_J2A, PIN_DIR_J2A);
AccelStepper stJ2B(AccelStepper::DRIVER, PIN_STEP_J2B, PIN_DIR_J2B);
AccelStepper stJ3(AccelStepper::DRIVER, PIN_STEP_J3, PIN_DIR_J3);
AccelStepper stJ4(AccelStepper::DRIVER, PIN_STEP_J4, PIN_DIR_J4);
Servo gripper;

// ---------------- State -----------------------------------------------------
bool liveMode = false;          // SIM (default, safe) until LIVE is sent
bool autoMode = false;          // RUN / STOP continuous auto
volatile bool estopTriggered = false; // set by the E-stop ISR
bool estopLatched = false;      // cleared only by the SIM command
bool homed = false;
bool lastIr = false;            // IR edge detection

float curJ1 = 130, curJ2 = 90, curJ3 = 90, curJ4 = 90;  // slider degrees
float curTrack = 0, curGrip = GRIP_CLOSED;

// ---------------- Helpers ---------------------------------------------------
float deg2rad(float d) { return d * DEG_TO_RAD; }
float rad2deg(float r) { return r * RAD_TO_DEG; }

void estopISR() { estopTriggered = true; }

void applyEstop() {
  stTrack.stop(); stJ1.stop(); stJ2A.stop(); stJ2B.stop();
  stJ3.stop(); stJ4.stop();
  autoMode = false;
  liveMode = false;             // documented: E-STOP drops the robot to SIM
  Serial.println(F("ESTOP ACTIVE - mode forced to SIM, send SIM then HOME"));
  homed = false;
}

// Convert slider angles to stepper targets and command a coordinated move.
void commandJoints(float j1, float j2, float j3, float j4) {
  curJ1 = j1; curJ2 = j2; curJ3 = j3; curJ4 = j4;
  if (!liveMode) return;
  long s1 = lround(j1 * STEPS_PER_DEG_J1);
  long s2 = lround(j2 * STEPS_PER_DEG_J2);
  long s3 = lround(j3 * STEPS_PER_DEG_J3);
  long s4 = lround(j4 * STEPS_PER_DEG_J4);
  stJ1.moveTo(s1);
  stJ2A.moveTo(s2);
  stJ2B.moveTo(s2);      // dual shoulder motors run mirrored on the same target
  stJ3.moveTo(s3);
  stJ4.moveTo(s4);
}

void commandTrack(float cm) {
  curTrack = constrain(cm, TRACK_MIN, TRACK_MAX);
  if (!liveMode) return;
  stTrack.moveTo(lround(curTrack * STEPS_PER_CM_TRACK));
}

void commandGripper(float deg) {
  curGrip = constrain(deg, GRIP_OPEN, GRIP_CLOSED);
  if (!liveMode) return;
  gripper.write(lround(curGrip));
}

bool anyMoving() {
  return stTrack.isRunning() || stJ1.isRunning() || stJ2A.isRunning() ||
         stJ2B.isRunning() || stJ3.isRunning() || stJ4.isRunning();
}

void waitMotion() {
  while (anyMoving() && !estopped) runAllSteppers();
}

// ---------------- Geometric IK (law of cosines, gripper stays level) --------
bool moveToXYZ(float x, float y, float z) {
  // Slide the track toward the target when it is out of the arm's reach.
  float lx = x - curTrack;
  float rW = hypot(lx, y) - L4;       // level gripper: L4 lies horizontal
  float s = z - L1;
  float dist = hypot(rW, s);
  if (dist > MAX_REACH || dist < MIN_REACH) {
    float newTrack = constrain(x, TRACK_MIN, TRACK_MAX);
    if (fabs(newTrack - curTrack) > 0.5f) {
      commandTrack(newTrack);
      waitMotion();
      lx = x - curTrack;
      rW = hypot(lx, y) - L4;
      dist = hypot(rW, s);
    }
  }
  if (dist > MAX_REACH || dist < MIN_REACH) {
    Serial.println(F("ERR unreachable target"));
    return false;
  }

  // 1. base angle from X and Y (arm-local frame)
  float theta1 = atan2(y, lx);                       // rad
  // 2. shoulder & elbow from the link triangle (law of cosines)
  float d = (rW * rW + s * s - L2 * L2 - L3 * L3) / (2.0f * L2 * L3);
  d = constrain(d, -1.0f, 1.0f);
  float theta3 = atan2(-sqrt(fmax(0.0f, 1.0f - d * d)), d);   // elbow-down branch
  float phi2 = atan2(s, rW) - atan2(L3 * sin(theta3), L2 + L3 * cos(theta3));
  // 3. wrist so the gripper stays horizontal (J4 slider = 90 means level)

  float j1 = rad2deg(theta1) + 130.0;
  float j2 = rad2deg(phi2);
  float j3 = 90.0 + rad2deg(theta3);
  if (j1 < 0 || j1 > 260 || j2 < 0 || j2 > 200 || j3 < 0 || j3 > 250) {
    Serial.println(F("ERR joint limit"));
    return false;
  }
  // 4. convert each joint angle to steps and command the motors
  commandJoints(j1, j2, j3, 90.0);
  Serial.print(F("OK XYZ -> J1=")); Serial.print(j1, 1);
  Serial.print(F(" J2="));          Serial.print(j2, 1);
  Serial.print(F(" J3="));          Serial.print(j3, 1);
  Serial.print(F(" J4=90 L="));     Serial.println(curTrack, 1);
  return true;
}

// ---------------- Homing ----------------------------------------------------
// Each joint is driven slowly toward its limit switch; contact defines zero
// (slider 0), giving the absolute reference the IK relies on.
bool homeAxis(AccelStepper &st, uint8_t limPin, float sign, unsigned long maxTravel) {
  st.setMaxSpeed(400);
  st.setAcceleration(300);
  st.move((long)(sign * maxTravel));
  while (digitalRead(limPin) == HIGH) {   // switch wired to GND, active LOW
    st.run();
    if (estopLatched) return false;
  }
  st.stop();
  while (st.isRunning()) st.run();
  st.setCurrentPosition(0);               // slider 0 at the limit
  return true;
}

void homeAllJoints() {
  Serial.println(F("Homing..."));
  if (!liveMode) { Serial.println(F("OK HOME (SIM - no motion)")); homed = true; return; }
  bool ok = true;
  ok &= homeAxis(stJ2A, PIN_LIM_J2, -1, 210L * (long)STEPS_PER_DEG_J2);
  if (ok) stJ2B.setCurrentPosition(0);    // dual shoulder shares the zero
  ok &= homeAxis(stJ3, PIN_LIM_J3, -1, 260L * (long)STEPS_PER_DEG_J3);
  ok &= homeAxis(stJ4, PIN_LIM_J4, -1, 210L * (long)STEPS_PER_DEG_J4);
  ok &= homeAxis(stJ1, PIN_LIM_J1, -1, 270L * (long)STEPS_PER_DEG_J1);
  ok &= homeAxis(stTrack, PIN_LIM_TRACK, -1, 150L * (long)STEPS_PER_CM_TRACK);
  if (!ok) { Serial.println(F("ERR homing aborted")); return; }
  homed = true;
  // move to the documented ruler pose (arm vertical, gripper level)
  curTrack = 0; curJ1 = 130; curJ2 = 90; curJ3 = 90; curJ4 = 90;
  commandJoints(130, 90, 90, 90);
  waitMotion();
  Serial.println(F("OK HOME"));
}

void softHome() {  // HOME1: quick return to the ruler pose, no re-homing
  if (!homed) { Serial.println(F("ERR not homed")); return; }
  commandTrack(0); commandJoints(130, 90, 90, 90);
  Serial.println(F("OK HOME1"));
}

// ---------------- Pick & place cycle (firmware automation) ------------------
void pickPlaceCycle() {
  Serial.println(F("CYCLE start"));
  // approach & pick
  moveToXYZ(PICK_X, PICK_Y, PICK_Z + APPROACH_LIFT); waitMotion();
  commandGripper(GRIP_OPEN);
  moveToXYZ(PICK_X, PICK_Y, PICK_Z);                 waitMotion();
  commandGripper(GRIP_CLOSED);  delay(400);          // grip the bottle
  moveToXYZ(PICK_X, PICK_Y, PICK_Z + APPROACH_LIFT); waitMotion();
  // transfer & place
  moveToXYZ(PLACE_X, PLACE_Y, PLACE_Z + APPROACH_LIFT); waitMotion();
  moveToXYZ(PLACE_X, PLACE_Y, PLACE_Z);                waitMotion();
  commandGripper(GRIP_OPEN);    delay(300);          // release
  moveToXYZ(PLACE_X, PLACE_Y, PLACE_Z + APPROACH_LIFT); waitMotion();
  Serial.println(F("CYCLE done"));
}

bool irDetected() { return digitalRead(PIN_IR) == LOW; }  // module is active LOW

// ---------------- Command interpreter ---------------------------------------
void parseCommand() {
  static char buf[48];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) { buf[n] = 0; handleCommand(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

void handleCommand(char *cmd) {
  // while the E-stop is latched only SIM (release) and ESTOP are accepted
  if (estopLatched && strcmp(cmd, "SIM") != 0 && strcmp(cmd, "ESTOP") != 0) return;

  if (!strcmp(cmd, "SIM")) {
    liveMode = false; autoMode = false; estopLatched = false; estopTriggered = false;
    Serial.println(F("OK SIM"));
  } else if (!strcmp(cmd, "LIVE")) {
    if (estopLatched) { Serial.println(F("ERR estop latched")); return; }
    liveMode = true;
    Serial.println(F("OK LIVE"));
  } else if (!strcmp(cmd, "ESTOP")) {
    estopLatched = true; applyEstop();
  } else if (!strcmp(cmd, "HOME")) {
    homeAllJoints();
  } else if (!strcmp(cmd, "HOME1")) {
    softHome();
  } else if (!strncmp(cmd, "XYZ ", 4)) {
    float x, y, z;
    if (sscanf(cmd + 4, "%f %f %f", &x, &y, &z) == 3) moveToXYZ(x, y, z);
    else Serial.println(F("ERR XYZ syntax"));
  } else if (cmd[0] == 'J' && cmd[1] >= '1' && cmd[1] <= '4' && cmd[2] == ' ') {
    float deg = atof(cmd + 3);
    switch (cmd[1]) {
      case '1': commandJoints(deg, curJ2, curJ3, curJ4); break;
      case '2': commandJoints(curJ1, deg, curJ3, curJ4); break;
      case '3': commandJoints(curJ1, curJ2, deg, curJ4); break;
      case '4': commandJoints(curJ1, curJ2, curJ3, deg); break;
    }
    Serial.print(F("OK J")); Serial.println(cmd[1]);
  } else if (cmd[0] == 'L' && cmd[1] == ' ') {
    commandTrack(atof(cmd + 2));
    Serial.println(F("OK L"));
  } else if (cmd[0] == 'G' && cmd[1] == ' ') {
    commandGripper(atof(cmd + 2));
    Serial.println(F("OK G"));
  } else if (!strcmp(cmd, "GO")) {
    if (irDetected()) pickPlaceCycle();
    else Serial.println(F("OK GO skipped (no bottle at IR)"));
  } else if (!strcmp(cmd, "CYCLE")) {
    pickPlaceCycle();
  } else if (!strcmp(cmd, "RUN")) {
    autoMode = true;
    Serial.println(F("OK RUN (auto)"));
  } else if (!strcmp(cmd, "STOP")) {
    autoMode = false;
    Serial.println(F("OK STOP"));
  } else {
    Serial.print(F("ERR unknown: ")); Serial.println(cmd);
  }
}

// ---------------- Setup & main loop -----------------------------------------
void configureSteppers() {
  // max speed + acceleration per joint (smooth ramps protect the bottles)
  stTrack.setMaxSpeed(4000); stTrack.setAcceleration(2000);
  stJ1.setMaxSpeed(1500);    stJ1.setAcceleration(800);
  stJ2A.setMaxSpeed(1500);   stJ2A.setAcceleration(800);
  stJ2B.setMaxSpeed(1500);   stJ2B.setAcceleration(800);
  stJ3.setMaxSpeed(2500);    stJ3.setAcceleration(1200);
  stJ4.setMaxSpeed(1500);    stJ4.setAcceleration(800);
}

void setup() {
  Serial.begin(115200);
  configureSteppers();
  gripper.attach(PIN_SERVO);
  pinMode(PIN_LIM_TRACK, INPUT_PULLUP);
  pinMode(PIN_LIM_J1, INPUT_PULLUP);
  pinMode(PIN_LIM_J2, INPUT_PULLUP);
  pinMode(PIN_LIM_J3, INPUT_PULLUP);
  pinMode(PIN_LIM_J4, INPUT_PULLUP);
  pinMode(PIN_IR, INPUT_PULLUP);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), estopISR, FALLING);
  // Boot in SIM: the operator homes and goes LIVE from the dashboard.
  Serial.println(F("RHINO firmware ready (SIM). Send HOME, then LIVE."));
}

void runAllSteppers() {
  stTrack.run(); stJ1.run(); stJ2A.run();
  stJ2B.run(); stJ3.run(); stJ4.run();
}

void loop() {
  if (estopTriggered && !estopLatched) { estopLatched = true; applyEstop(); }
  parseCommand();          // XYZ, J, G, GO, RUN, STOP ...
  runAllSteppers();        // AccelStepper: smooth, non-blocking steps
  bool ir = irDetected();
  if (ir && !lastIr) {
    Serial.println(F("IR_DETECT"));    // dashboard can trigger its GUI sequence
    if (autoMode) pickPlaceCycle();    // firmware auto mode (RUN)
  }
  lastIr = ir;
}
