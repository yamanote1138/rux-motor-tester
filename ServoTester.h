/*
 * ServoTester.h
 *
 * Class for controlling and characterizing 5-wire servos
 * Handles motor control via TB6612FNG H-Bridge, position feedback from potentiometer,
 * safety monitoring, and closed-loop position control with PID.
 */

#ifndef SERVO_TESTER_H
#define SERVO_TESTER_H

#include <Arduino.h>

// Pin definitions (from main sketch) - TB6612FNG configuration
#define PIN_MOTOR_EN        5   // TB6612 PWMA
#define PIN_MOTOR_IN1       6   // TB6612 AIN1
#define PIN_MOTOR_IN2       7   // TB6612 AIN2
#define PIN_EMERGENCY_CUT   8   // TB6612 STBY (active HIGH)
#define PIN_STATUS_LED      13
#define PIN_POT_WIPER       A0
#define PIN_POT_VCC_MON     A1
#define PIN_CURRENT_SENSE   A2
#define PIN_VOLTAGE_MON     A3

// Safety limits (configurable)
#define DEFAULT_CURRENT_LIMIT_MA    500.0
#define DEFAULT_VOLTAGE_MIN         4.5
#define DEFAULT_VOLTAGE_MAX         5.5
#define DEFAULT_POSITION_DEADBAND   5
#define DEFAULT_TIMEOUT_MS          30000

// Motor control constants
#define MOTOR_STOP          0
#define MOTOR_FORWARD       1
#define MOTOR_REVERSE       2

class ServoTester {
public:
  // Lifecycle
  ServoTester();
  void begin();
  void update();              // Call in loop() for position control updates
  void monitorSafety();       // Checks limits, triggers emergency stop if needed

  // Power Control
  void powerOn();
  void powerOff();
  void emergencyStop();
  bool isPowerOn();

  // Motor Control (H-Bridge)
  void setMotorSpeed(int speed);      // -255 to +255 (negative = reverse)
  void setMotorRaw(int dir, int pwm); // Direct H-Bridge control
  void stopMotor();

  // Position Control (Closed-Loop)
  void setTargetPosition(int target);  // Target pot ADC value (0-1023)
  void updatePositionControl();        // PID loop, called by update()
  void calibrateRange();               // Find min/max pot values
  void enablePositionControl(bool enable);
  bool isPositionControlEnabled();

  // Measurement
  int readPosition();          // Current potentiometer ADC value (0-1023)
  float readCurrent();         // Milliamps via shunt resistor
  float readVoltage();         // Motor supply voltage
  float readPotVCC();          // Potentiometer supply voltage
  int getTargetPosition();     // Get current target
  int getPositionError();      // Target - Current

  // Characterization Tests
  void runManualSweep(int startSpeed, int endSpeed, int step, int delayMs);
  void runPositionSweep(int startPos, int endPos, int step, int delayMs);
  void runStepResponse(int targetPos);
  void runPositionMapping();
  void runSpeedTest();

  // Configuration
  void setCurrentLimit(float maxMilliamps);
  void setPIDGains(float Kp, float Ki, float Kd);
  void setDeadband(int adcUnits);
  void setTimeout(unsigned long timeoutMs);

  // Status reporting
  void printStatus();
  void printConfiguration();

private:
  // State
  bool _powerOn;
  bool _positionControlEnabled;
  int _currentMotorSpeed;
  int _targetPosition;
  int _lastPosition;

  // PID control
  float _Kp, _Ki, _Kd;
  float _integral;
  float _lastError;
  unsigned long _lastPIDUpdate;

  // Calibration
  int _minPosition;
  int _maxPosition;
  bool _calibrated;

  // Safety limits
  float _currentLimitMA;
  float _voltageMin;
  float _voltageMax;
  int _deadband;
  unsigned long _timeoutMs;

  // Test state
  bool _testRunning;
  unsigned long _testStartTime;

  // Helper functions
  void updateMotorDirection(int dir, int pwm);
  int constrainMotorSpeed(int speed);
  bool checkSafetyLimits();
  void blinkLED(int times);
};

#endif // SERVO_TESTER_H
