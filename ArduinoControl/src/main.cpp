#include <Arduino.h>
#include "SerialPort.h"
#include "Gpio.h"
#include "RoboClaw.h"
#include <RoboClawMotor.h>
#include "Command.h"  // Use Command.h to parse incoming messages

//============== Instantiate RoboClaw objects. ======================
#define rclawSerial Serial1
RoboClaw rClaw(&rclawSerial,1000);
RoboClawMotor motor_frontRight(rClaw,0x80, 2);
RoboClawMotor motor_frontLeft (rClaw,0x80, 1);
RoboClawMotor motor_backRight (rClaw,0x81, 2);
RoboClawMotor motor_backLeft  (rClaw,0x81, 1);

//=================== Define Helper Functions ====================================
void run(const String &message);
void move(float vx, float wz);

// ============== Relays for tool manipulation ====================================
#define tool_state  2
#define tool_active 3

//======== SerialPort instance for receiving commands (from ROS2 via serial) =====
SerialPort port(115200, run);

// ================== Drive model & ramping parameters ===========================
// Kinematics
static constexpr float WHEELBASE_L = 1.0f;   // effective track (tune to your base)
static constexpr float INPUT_LIMIT  = 1.0f;  // expected |vx|, |wz| max

// Ramping (units: normalized command per second, i.e., 1.0 = full scale change per second)
static constexpr float ACCEL_RATE   = 2.5f;  // how fast you speed up
static constexpr float DECEL_RATE   = 3.5f;  // how fast you slow down (usually >= ACCEL_RATE)

// Control loop timing
static constexpr uint32_t CONTROL_HZ       = 100;               // 100 Hz control loop
static constexpr uint32_t CONTROL_DT_MS    = 1000 / CONTROL_HZ; // 10 ms
static constexpr uint32_t CMD_TIMEOUT_MS   = 200;               // if no cmd for 200ms, deramp to 0

// Targets (from latest move command)
volatile float tgt_left  = 0.0f;
volatile float tgt_right = 0.0f;

// Current commanded values actually sent to motors (slewed)
float cmd_left  = 0.0f;
float cmd_right = 0.0f;

// Bookkeeping
uint32_t last_update_ms   = 0;
uint32_t last_cmd_rx_ms   = 0;

// ============== Setup function ======================
void setup() {
  Serial.begin(115200);
  rClaw.begin(38400);

  pinMode(tool_state, OUTPUT);
  pinMode(tool_active, OUTPUT);

  while (!Serial) { ; }
  port.setup();

  motor_frontLeft .speed(0.0f);
  motor_backLeft  .speed(0.0f);
  motor_frontRight.speed(0.0f);
  motor_backRight .speed(0.0f);

  last_update_ms = millis();
  last_cmd_rx_ms = millis();

  Serial.println("Teensy robot control ready...");
}

static inline float clamp1(float x) {
  if (x >  1.0f) return  1.0f;
  if (x < -1.0f) return -1.0f;
  return x;
}

// Slew one channel toward its target with separate accel/decel limits
static inline float slewToward(float current, float target, float dt_sec) {
  float delta = target - current;
  if (delta == 0.0f) return current;

  float rate = (fabs(target) < fabs(current)) ? DECEL_RATE : ACCEL_RATE; // decel if magnitude decreasing
  float max_step = rate * dt_sec;

  if (delta >  max_step) return current + max_step;
  if (delta < -max_step) return current - max_step;
  return target; // can reach in this step
}

void loop() {
  // Handle incoming serial
  port.update();

  // Control loop at CONTROL_HZ
  uint32_t now = millis();
  if (now - last_update_ms >= CONTROL_DT_MS) {
    float dt = (now - last_update_ms) / 1000.0f;
    last_update_ms = now;

    // Command timeout → gracefully deramp to zero
    if (now - last_cmd_rx_ms > CMD_TIMEOUT_MS) {
      tgt_left  = 0.0f;
      tgt_right = 0.0f;
    }

    // Slew current commands toward targets
    cmd_left  = slewToward(cmd_left,  tgt_left,  dt);
    cmd_right = slewToward(cmd_right, tgt_right, dt);

    // Clamp for safety
    cmd_left  = clamp1(cmd_left);
    cmd_right = clamp1(cmd_right);

    // Send to motors
    motor_frontLeft .speed(cmd_left);
    motor_backLeft  .speed(cmd_left);
    motor_frontRight.speed(cmd_right);
    motor_backRight .speed(cmd_right);

    // Optional debug (comment out if too chatty)
    // Serial.printf("tgtL:%.2f tgtR:%.2f | cmdL:%.2f cmdR:%.2f\n", tgt_left, tgt_right, cmd_left, cmd_right);
  }
}

void run(const String &message) {
  String cmd = command(message);  // Get the first token

  if (cmd.equalsIgnoreCase("move")) {
    float vx = argument(message, 1, 0);
    float wz = argument(message, 2, 0);

    // record last time we got a command
    last_cmd_rx_ms = millis();

    // Update motion targets (deramping handled in loop())
    move(vx, wz);
  }
  else if (cmd.equalsIgnoreCase("halt")) {
    // Graceful stop: set targets to zero; control loop deramps
    last_cmd_rx_ms = millis(); // treat as fresh command
    move(0.0f, 0.0f);
  }
  else if (cmd.equalsIgnoreCase("tool_lock")) {
    digitalWrite(tool_state, HIGH);
  }
  else if (cmd.equalsIgnoreCase("tool_unlock")) {
    digitalWrite(tool_state, LOW);
  }
  else if (cmd.equalsIgnoreCase("tool_on")) {
    digitalWrite(tool_active, HIGH);
  }
  else if (cmd.equalsIgnoreCase("tool_off")) {
    digitalWrite(tool_active, LOW);
  }
  else {
    port.send("Error: Command format not recognized!");
  }
}

// Map (vx, wz) → left/right targets, with normalization, but DO NOT write motors here.
// The control loop slews cmd_* toward tgt_*.
void move(float vx, float wz) {
  // Limit inputs
  float v = constrain(vx, -INPUT_LIMIT, INPUT_LIMIT);
  float w = constrain(wz, -INPUT_LIMIT, INPUT_LIMIT);

  // Differential drive mapping
  float left  = v - (w * WHEELBASE_L / 2.0f);
  float right = v + (w * WHEELBASE_L / 2.0f);

  // Normalize to keep within [-1,1]
  float m = max(fabs(left), fabs(right));
  if (m > INPUT_LIMIT && m > 0.0f) {
    left  /= m;
    right /= m;
  }

  // Set targets; loop() will ramp toward these
  tgt_left  = clamp1(left);
  tgt_right = clamp1(right);

  // Optional debug
  // Serial.printf("New targets -> L: %.2f  R: %.2f\n", tgt_left, tgt_right);
}
