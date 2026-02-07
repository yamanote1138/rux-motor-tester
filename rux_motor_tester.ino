/*
 * Teddy Ruxpin 5-Wire Servo Tester
 *
 * Arduino-based tester for characterizing vintage Teddy Ruxpin servos.
 * 5-wire servos consist of: 2 motor wires + 3 potentiometer wires (VCC, GND, Wiper)
 *
 * Hardware Requirements:
 * - Arduino Uno/Nano (ATmega328P, 5V, 16MHz)
 * - TB6612FNG H-Bridge motor driver (1.2A dual channel)
 * - External 5V power supply for motor (1-2A capable)
 * - Protection circuit (see CIRCUIT.md)
 *
 * Serial Interface: 115200 baud
 * Type "HELP" for command list
 *
 * WARNING: Always start with POWER OFF and minimum motor speeds when testing
 *          unknown servos. Monitor current draw continuously.
 */

#include "ServoTester.h"
#include "CommandParser.h"

// Pin definitions (match PLAN.md) - TB6612FNG Motor Driver
#define PIN_MOTOR_EN        5   // TB6612 PWMA (PWM speed control)
#define PIN_MOTOR_IN1       6   // TB6612 AIN1 (direction)
#define PIN_MOTOR_IN2       7   // TB6612 AIN2 (direction)
#define PIN_EMERGENCY_CUT   8   // TB6612 STBY (standby - HIGH=enabled, LOW=emergency stop)
#define PIN_STATUS_LED      13  // Status indicator LED

#define PIN_POT_WIPER       A0  // Potentiometer position feedback
#define PIN_POT_VCC_MON     A1  // Potentiometer VCC monitor
#define PIN_CURRENT_SENSE   A2  // Current sense via shunt resistor
#define PIN_VOLTAGE_MON     A3  // Motor supply voltage monitor

// Global objects
ServoTester servo;
CommandParser parser;

// Timing
unsigned long lastSafetyCheck = 0;
const unsigned long SAFETY_CHECK_INTERVAL = 10; // 10ms = 100Hz

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for native USB)
  }

  // Print startup banner
  Serial.println(F("======================================"));
  Serial.println(F("Teddy Ruxpin 5-Wire Servo Tester"));
  Serial.println(F("Version 1.0"));
  Serial.println(F("======================================"));
  Serial.println();

  // Initialize servo tester
  servo.begin();

  Serial.println(F("System initialized. Type 'HELP' for commands."));
  Serial.println(F("WARNING: Power is OFF. Use 'POWER ON' to enable."));
  Serial.println();
  Serial.print(F("> "));
}

void loop() {
  // Check for incoming serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      parser.parse(command, servo);
      Serial.print(F("> ")); // Print prompt
    }
  }

  // Periodic safety monitoring (100Hz)
  unsigned long currentTime = millis();
  if (currentTime - lastSafetyCheck >= SAFETY_CHECK_INTERVAL) {
    lastSafetyCheck = currentTime;
    servo.monitorSafety();
  }

  // Update position control if active
  servo.update();
}
