/*
 * ============================================================================
 * ROBOT DRIVER — Production Arduino Firmware
 * ============================================================================
 * 
 * Project:  Vision-Based Autonomous Robotic Arm
 * File:     robot_driver.ino
 * Location: firmware/robot_driver/
 * 
 * PURPOSE:
 *   Production firmware that runs on the Arduino Uno during ALL phases of
 *   the project: teleoperation, dataset collection, and autonomous execution.
 *   It acts as a dumb, fast serial-to-PWM bridge:
 *     1. Receives 6-byte binary packets from the host (MacBook or RPi5)
 *     2. Commands the PCA9685 to set each servo angle immediately
 *     3. Monitors for communication loss and auto-returns to Home
 * 
 * SERIAL PROTOCOL (Decision #6 in decisions.md):
 *   - Baud Rate: 115200
 *   - Packet Format: 6 raw bytes, one per servo (value 0–180)
 *   - Byte Order: [Base, Shoulder, Elbow, WristPitch, WristRoll, Gripper]
 *   - Target Frequency: 30 Hz (sent by the Python host)
 *   - Why binary over ASCII: Lower latency, minimal parsing, critical for
 *     smooth teleoperation demonstrations
 * 
 * SAFETY (Decision #5 in decisions.md):
 *   - 500ms watchdog timer: if no serial packet arrives within 500ms,
 *     the firmware assumes the host has crashed or disconnected and
 *     automatically returns all servos to the Home Position.
 *   - This is the SOFTWARE layer of safety. The HARDWARE layer is a
 *     physical emergency stop switch that cuts the 6V battery power.
 * 
 * SERVO CHANNEL MAPPING (defined in architecture.md):
 *   Channel 0 = Base         (MG996R)
 *   Channel 1 = Shoulder     (MG996R)
 *   Channel 2 = Elbow        (MG996R)
 *   Channel 3 = Wrist Pitch  (MG90S)
 *   Channel 4 = Wrist Roll   (MG90S)
 *   Channel 5 = Gripper      (MG90S)
 * 
 * HARDWARE REQUIRED:
 *   - Arduino Uno Rev3
 *   - PCA9685 16-Channel PWM Servo Driver (I2C address 0x40)
 *   - 6V battery connected to PCA9685 V+ and GND
 *   - 6 servos on PCA9685 channels 0–5
 * 
 * DEPENDENCIES:
 *   - Wire.h (built-in Arduino I2C)
 *   - Adafruit_PWMServoDriver (install via Library Manager)
 * 
 * NOTE ON SERVO FEEDBACK:
 *   MG996R and MG90S are open-loop PWM servos — they do NOT report their
 *   actual position back. We must assume commanded_angle ≈ actual_angle.
 *   This is acceptable as long as payloads remain lightweight (sponge blocks,
 *   per Decision #7 in decisions.md).
 * ============================================================================
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize PCA9685 at default I2C address (0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pulse width limits — tune these if a specific servo doesn't reach 0° or 180°
#define SERVOMIN  150 // Pulse count for 0 degrees (out of 4096)
#define SERVOMAX  600 // Pulse count for 180 degrees (out of 4096)
#define SERVO_FREQ 50 // 50 Hz — standard for analog hobby servos

const int NUM_SERVOS = 6;

// Home Position angles (0–180) for each servo
// These define the safe resting configuration the robot returns to on startup
// and whenever communication is lost. Adjust after physical assembly.
// NOTE: These may need to be updated once the IK solver defines the true home pose.
const uint8_t HOME_ANGLES[NUM_SERVOS] = {90, 90, 90, 90, 90, 10}; 

// Track the last commanded angle for each servo
// Since we have no encoder feedback, this is our only "state" estimate
uint8_t current_angles[NUM_SERVOS];

// Watchdog timer variables (Decision #5 — Safety)
unsigned long last_command_time = 0;
const unsigned long TIMEOUT_MS = 500; // 500ms without a command = connection lost

// Buffer to store incoming serial data (exactly 6 bytes per command packet)
byte serialBuffer[NUM_SERVOS];

void setup() {
  // High baud rate is critical for low-latency teleoperation (Decision #6)
  Serial.begin(115200);
  Serial.setTimeout(10); // Don't block on incomplete reads

  pwm.begin();
  pwm.setOscillatorFrequency(27000000); // Standard PCA9685 internal oscillator
  pwm.setPWMFreq(SERVO_FREQ);

  delay(10);
  
  // Always start in a known, safe position
  moveToHome();
}

// Convert a 0–180 degree angle to the PCA9685 pulse width count
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Command a single servo to a specific angle, with safety clamping
void setServoAngle(uint8_t servoNum, uint8_t angle) {
  // Hard clamp to 0–180 to prevent invalid PWM signals
  angle = constrain(angle, 0, 180);
  pwm.setPWM(servoNum, 0, angleToPulse(angle));
  current_angles[servoNum] = angle;
}

// Move all servos to the predefined Home Position
void moveToHome() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, HOME_ANGLES[i]);
  }
}

void loop() {
  // Check if we have a complete 6-byte packet waiting
  if (Serial.available() >= NUM_SERVOS) {
    Serial.readBytes(serialBuffer, NUM_SERVOS);
    
    // Apply commanded angles to all 6 servos immediately
    for (int i = 0; i < NUM_SERVOS; i++) {
       setServoAngle(i, serialBuffer[i]);
    }
    
    // Reset the watchdog timer — we just received a valid command
    last_command_time = millis();
    
    // Flush any extra bytes that accumulated in the buffer
    // This prevents lag buildup if the host sends faster than we process
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
  
  // SAFETY WATCHDOG: If no command received for 500ms, assume host crashed
  // and return to a safe Home Position to prevent the arm from staying in
  // an awkward or dangerous pose indefinitely
  if (millis() - last_command_time > TIMEOUT_MS && last_command_time != 0) {
    moveToHome();
    last_command_time = 0; // Reset so we don't spam moveToHome every loop
  }
}
