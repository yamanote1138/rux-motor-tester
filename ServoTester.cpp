/*
 * ServoTester.cpp
 *
 * Implementation of ServoTester class for 5-wire servo control and characterization
 */

#include "ServoTester.h"

// Constructor
ServoTester::ServoTester() {
  _powerOn = false;
  _positionControlEnabled = false;
  _currentMotorSpeed = 0;
  _targetPosition = 512;
  _lastPosition = 512;

  // PID defaults (conservative P-only start)
  _Kp = 1.0;
  _Ki = 0.0;
  _Kd = 0.0;
  _integral = 0.0;
  _lastError = 0.0;
  _lastPIDUpdate = 0;

  // Calibration
  _minPosition = 0;
  _maxPosition = 1023;
  _calibrated = false;

  // Safety defaults
  _currentLimitMA = DEFAULT_CURRENT_LIMIT_MA;
  _voltageMin = DEFAULT_VOLTAGE_MIN;
  _voltageMax = DEFAULT_VOLTAGE_MAX;
  _deadband = DEFAULT_POSITION_DEADBAND;
  _timeoutMs = DEFAULT_TIMEOUT_MS;

  _testRunning = false;
  _testStartTime = 0;
}

// Initialize hardware
void ServoTester::begin() {
  // Configure motor control pins
  pinMode(PIN_MOTOR_EN, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);

  // Configure emergency cutoff
  pinMode(PIN_EMERGENCY_CUT, OUTPUT);
  digitalWrite(PIN_EMERGENCY_CUT, LOW); // Start with power OFF

  // Configure status LED
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  // Configure analog inputs (already in high-impedance by default)
  // No pinMode needed for analog inputs

  // Ensure motor is stopped
  stopMotor();

  // Blink LED to indicate successful initialization
  blinkLED(3);
}

// Main update function - call in loop()
void ServoTester::update() {
  if (_positionControlEnabled && _powerOn) {
    updatePositionControl();
  }
}

// Safety monitoring - checks all limits
void ServoTester::monitorSafety() {
  if (!_powerOn) {
    return; // No monitoring needed when power is off
  }

  if (!checkSafetyLimits()) {
    emergencyStop();
    Serial.println(F("!!! EMERGENCY STOP - Safety limit exceeded !!!"));
    printStatus();
  }

  // Check test timeout
  if (_testRunning && (millis() - _testStartTime > _timeoutMs)) {
    stopMotor();
    _testRunning = false;
    Serial.println(F("Test timeout - motor stopped"));
  }
}

// Power control
void ServoTester::powerOn() {
  if (_powerOn) {
    Serial.println(F("Power already ON"));
    return;
  }

  digitalWrite(PIN_EMERGENCY_CUT, HIGH);
  _powerOn = true;
  digitalWrite(PIN_STATUS_LED, HIGH);
  Serial.println(F("Power ON"));
  delay(100); // Allow power to stabilize
}

void ServoTester::powerOff() {
  if (!_powerOn) {
    Serial.println(F("Power already OFF"));
    return;
  }

  stopMotor();
  digitalWrite(PIN_EMERGENCY_CUT, LOW);
  _powerOn = false;
  _positionControlEnabled = false;
  digitalWrite(PIN_STATUS_LED, LOW);
  Serial.println(F("Power OFF"));
}

void ServoTester::emergencyStop() {
  stopMotor();
  digitalWrite(PIN_EMERGENCY_CUT, LOW);
  _powerOn = false;
  _positionControlEnabled = false;
  digitalWrite(PIN_STATUS_LED, LOW);
  blinkLED(5); // Rapid blinks to indicate emergency
}

bool ServoTester::isPowerOn() {
  return _powerOn;
}

// Motor control
void ServoTester::setMotorSpeed(int speed) {
  if (!_powerOn) {
    Serial.println(F("Error: Power is OFF"));
    return;
  }

  // Constrain speed to valid range
  speed = constrain(speed, -255, 255);
  _currentMotorSpeed = speed;

  if (speed > 0) {
    // Forward
    updateMotorDirection(MOTOR_FORWARD, speed);
  } else if (speed < 0) {
    // Reverse
    updateMotorDirection(MOTOR_REVERSE, -speed);
  } else {
    // Stop
    stopMotor();
  }
}

void ServoTester::setMotorRaw(int dir, int pwm) {
  if (!_powerOn) {
    Serial.println(F("Error: Power is OFF"));
    return;
  }

  pwm = constrain(pwm, 0, 255);
  updateMotorDirection(dir, pwm);
}

void ServoTester::stopMotor() {
  analogWrite(PIN_MOTOR_EN, 0);
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  _currentMotorSpeed = 0;
}

// Position control
void ServoTester::setTargetPosition(int target) {
  _targetPosition = constrain(target, _minPosition, _maxPosition);
  _integral = 0.0; // Reset integral term when changing target
  Serial.print(F("Target position set to: "));
  Serial.println(_targetPosition);
}

void ServoTester::updatePositionControl() {
  unsigned long currentTime = millis();
  float dt = (currentTime - _lastPIDUpdate) / 1000.0; // Convert to seconds

  if (dt < 0.01) {
    return; // Don't update faster than 100Hz
  }

  _lastPIDUpdate = currentTime;

  // Read current position
  int currentPos = readPosition();
  int error = _targetPosition - currentPos;

  // Check if within deadband
  if (abs(error) < _deadband) {
    stopMotor();
    return;
  }

  // PID calculation
  _integral += error * dt;
  _integral = constrain(_integral, -1000, 1000); // Prevent integral windup

  float derivative = (error - _lastError) / dt;
  _lastError = error;

  float output = (_Kp * error) + (_Ki * _integral) + (_Kd * derivative);

  // Convert PID output to motor speed
  int motorSpeed = constrain((int)output, -255, 255);
  setMotorSpeed(motorSpeed);
}

void ServoTester::calibrateRange() {
  if (!_powerOn) {
    Serial.println(F("Error: Power is OFF"));
    return;
  }

  Serial.println(F("Starting calibration..."));
  Serial.println(F("Moving to find minimum position..."));

  // Move slowly in reverse to find minimum
  setMotorSpeed(-100);
  delay(2000);
  stopMotor();
  delay(500);
  _minPosition = readPosition();
  Serial.print(F("Min position: "));
  Serial.println(_minPosition);

  Serial.println(F("Moving to find maximum position..."));

  // Move slowly forward to find maximum
  setMotorSpeed(100);
  delay(2000);
  stopMotor();
  delay(500);
  _maxPosition = readPosition();
  Serial.print(F("Max position: "));
  Serial.println(_maxPosition);

  _calibrated = true;
  _targetPosition = (_minPosition + _maxPosition) / 2;

  Serial.println(F("Calibration complete!"));
  Serial.print(F("Range: "));
  Serial.print(_maxPosition - _minPosition);
  Serial.println(F(" ADC units"));
}

void ServoTester::enablePositionControl(bool enable) {
  _positionControlEnabled = enable;
  if (enable) {
    _integral = 0.0;
    _lastError = 0.0;
    _lastPIDUpdate = millis();
    Serial.println(F("Position control ENABLED"));
  } else {
    stopMotor();
    Serial.println(F("Position control DISABLED"));
  }
}

bool ServoTester::isPositionControlEnabled() {
  return _positionControlEnabled;
}

// Measurement functions
int ServoTester::readPosition() {
  return analogRead(PIN_POT_WIPER);
}

float ServoTester::readCurrent() {
  // Read voltage across 0.5Ω shunt resistor
  // V = I * R, so with R=0.5Ω, I = V / 0.5Ω = 2V
  // ADC resolution: 5V / 1024 = 0.00488V per unit
  // Current = ADC_value * 0.00488V / 0.5Ω = ADC_value * 9.77mA
  int adcValue = analogRead(PIN_CURRENT_SENSE);
  return adcValue * 9.77; // Result in milliamps
}

float ServoTester::readVoltage() {
  // Assumes 2:1 voltage divider
  // Actual voltage = ADC reading * (5V / 1024) * 2
  int adcValue = analogRead(PIN_VOLTAGE_MON);
  return adcValue * 0.00977; // Result in volts
}

float ServoTester::readPotVCC() {
  // Direct reading of potentiometer VCC line
  int adcValue = analogRead(PIN_POT_VCC_MON);
  return adcValue * 0.00488; // Result in volts
}

int ServoTester::getTargetPosition() {
  return _targetPosition;
}

int ServoTester::getPositionError() {
  return _targetPosition - readPosition();
}

// Characterization tests
void ServoTester::runManualSweep(int startSpeed, int endSpeed, int step, int delayMs) {
  if (!_powerOn) {
    Serial.println(F("Error: Power is OFF"));
    return;
  }

  Serial.println(F("Starting motor speed sweep..."));
  Serial.println(F("Speed,Position,Current(mA),Voltage(V)"));

  _testRunning = true;
  _testStartTime = millis();

  int increment = (endSpeed > startSpeed) ? step : -step;

  for (int speed = startSpeed;
       (increment > 0 && speed <= endSpeed) || (increment < 0 && speed >= endSpeed);
       speed += increment) {

    setMotorSpeed(speed);
    delay(delayMs);

    // Print data
    Serial.print(speed);
    Serial.print(F(","));
    Serial.print(readPosition());
    Serial.print(F(","));
    Serial.print(readCurrent());
    Serial.print(F(","));
    Serial.println(readVoltage());

    // Check safety
    if (!checkSafetyLimits()) {
      Serial.println(F("Safety limit exceeded - stopping sweep"));
      break;
    }
  }

  stopMotor();
  _testRunning = false;
  Serial.println(F("Sweep complete"));
}

void ServoTester::runPositionSweep(int startPos, int endPos, int step, int delayMs) {
  if (!_powerOn) {
    Serial.println(F("Error: Power is OFF"));
    return;
  }

  if (!_positionControlEnabled) {
    Serial.println(F("Error: Position control not enabled"));
    return;
  }

  Serial.println(F("Starting position sweep..."));
  Serial.println(F("Target,Actual,Error,Current(mA)"));

  _testRunning = true;
  _testStartTime = millis();

  int increment = (endPos > startPos) ? step : -step;

  for (int pos = startPos;
       (increment > 0 && pos <= endPos) || (increment < 0 && pos >= endPos);
       pos += increment) {

    setTargetPosition(pos);
    delay(delayMs);

    int actual = readPosition();
    Serial.print(pos);
    Serial.print(F(","));
    Serial.print(actual);
    Serial.print(F(","));
    Serial.print(pos - actual);
    Serial.print(F(","));
    Serial.println(readCurrent());

    if (!checkSafetyLimits()) {
      Serial.println(F("Safety limit exceeded - stopping sweep"));
      break;
    }
  }

  stopMotor();
  _testRunning = false;
  Serial.println(F("Position sweep complete"));
}

void ServoTester::runStepResponse(int targetPos) {
  if (!_powerOn) {
    Serial.println(F("Error: Power is OFF"));
    return;
  }

  Serial.println(F("Starting step response test..."));
  Serial.print(F("Initial position: "));
  Serial.println(readPosition());
  Serial.print(F("Target position: "));
  Serial.println(targetPos);

  unsigned long startTime = millis();
  setTargetPosition(targetPos);
  enablePositionControl(true);

  Serial.println(F("Time(ms),Position,Error,MotorSpeed"));

  _testRunning = true;
  _testStartTime = millis();

  // Record for 5 seconds or until settled
  while (millis() - startTime < 5000) {
    unsigned long elapsed = millis() - startTime;
    int pos = readPosition();
    int error = getPositionError();

    Serial.print(elapsed);
    Serial.print(F(","));
    Serial.print(pos);
    Serial.print(F(","));
    Serial.print(error);
    Serial.print(F(","));
    Serial.println(_currentMotorSpeed);

    delay(50); // Sample at 20Hz

    // Check if settled (within deadband for 500ms)
    if (abs(error) < _deadband) {
      static unsigned long settledTime = 0;
      if (settledTime == 0) {
        settledTime = millis();
      } else if (millis() - settledTime > 500) {
        Serial.println(F("Position settled"));
        break;
      }
    }

    if (!checkSafetyLimits()) {
      Serial.println(F("Safety limit exceeded - stopping test"));
      break;
    }
  }

  _testRunning = false;
  Serial.println(F("Step response test complete"));
}

void ServoTester::runPositionMapping() {
  if (!_powerOn) {
    Serial.println(F("Error: Power is OFF"));
    return;
  }

  Serial.println(F("Starting position mapping..."));
  Serial.println(F("This will sweep through the full range and record position values."));

  runPositionSweep(_minPosition, _maxPosition, 50, 1000);
}

void ServoTester::runSpeedTest() {
  Serial.println(F("Starting speed characterization..."));
  runManualSweep(0, 255, 25, 1000);
}

// Configuration
void ServoTester::setCurrentLimit(float maxMilliamps) {
  _currentLimitMA = maxMilliamps;
  Serial.print(F("Current limit set to: "));
  Serial.print(maxMilliamps);
  Serial.println(F(" mA"));
}

void ServoTester::setPIDGains(float Kp, float Ki, float Kd) {
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
  _integral = 0.0; // Reset integral when gains change
  Serial.print(F("PID gains set: Kp="));
  Serial.print(Kp);
  Serial.print(F(", Ki="));
  Serial.print(Ki);
  Serial.print(F(", Kd="));
  Serial.println(Kd);
}

void ServoTester::setDeadband(int adcUnits) {
  _deadband = adcUnits;
  Serial.print(F("Deadband set to: "));
  Serial.print(adcUnits);
  Serial.println(F(" ADC units"));
}

void ServoTester::setTimeout(unsigned long timeoutMs) {
  _timeoutMs = timeoutMs;
  Serial.print(F("Timeout set to: "));
  Serial.print(timeoutMs);
  Serial.println(F(" ms"));
}

// Status reporting
void ServoTester::printStatus() {
  Serial.println(F("--- STATUS ---"));
  Serial.print(F("Power: "));
  Serial.println(_powerOn ? F("ON") : F("OFF"));

  Serial.print(F("Position Control: "));
  Serial.println(_positionControlEnabled ? F("ENABLED") : F("DISABLED"));

  Serial.print(F("Motor Speed: "));
  Serial.println(_currentMotorSpeed);

  Serial.print(F("Current Position: "));
  Serial.println(readPosition());

  Serial.print(F("Target Position: "));
  Serial.println(_targetPosition);

  Serial.print(F("Position Error: "));
  Serial.println(getPositionError());

  Serial.print(F("Current Draw: "));
  Serial.print(readCurrent());
  Serial.println(F(" mA"));

  Serial.print(F("Motor Voltage: "));
  Serial.print(readVoltage());
  Serial.println(F(" V"));

  Serial.print(F("Pot VCC: "));
  Serial.print(readPotVCC());
  Serial.println(F(" V"));

  Serial.print(F("Calibrated: "));
  Serial.println(_calibrated ? F("YES") : F("NO"));

  if (_calibrated) {
    Serial.print(F("Position Range: "));
    Serial.print(_minPosition);
    Serial.print(F(" - "));
    Serial.println(_maxPosition);
  }

  Serial.println();
}

void ServoTester::printConfiguration() {
  Serial.println(F("--- CONFIGURATION ---"));
  Serial.print(F("Current Limit: "));
  Serial.print(_currentLimitMA);
  Serial.println(F(" mA"));

  Serial.print(F("Voltage Range: "));
  Serial.print(_voltageMin);
  Serial.print(F(" - "));
  Serial.print(_voltageMax);
  Serial.println(F(" V"));

  Serial.print(F("Deadband: "));
  Serial.print(_deadband);
  Serial.println(F(" ADC units"));

  Serial.print(F("Test Timeout: "));
  Serial.print(_timeoutMs);
  Serial.println(F(" ms"));

  Serial.print(F("PID Gains: Kp="));
  Serial.print(_Kp);
  Serial.print(F(", Ki="));
  Serial.print(_Ki);
  Serial.print(F(", Kd="));
  Serial.println(_Kd);

  Serial.println();
}

// Private helper functions
void ServoTester::updateMotorDirection(int dir, int pwm) {
  pwm = constrain(pwm, 0, 255);

  switch (dir) {
    case MOTOR_STOP:
      digitalWrite(PIN_MOTOR_IN1, LOW);
      digitalWrite(PIN_MOTOR_IN2, LOW);
      analogWrite(PIN_MOTOR_EN, 0);
      break;

    case MOTOR_FORWARD:
      digitalWrite(PIN_MOTOR_IN1, HIGH);
      digitalWrite(PIN_MOTOR_IN2, LOW);
      analogWrite(PIN_MOTOR_EN, pwm);
      break;

    case MOTOR_REVERSE:
      digitalWrite(PIN_MOTOR_IN1, LOW);
      digitalWrite(PIN_MOTOR_IN2, HIGH);
      analogWrite(PIN_MOTOR_EN, pwm);
      break;

    default:
      stopMotor();
      break;
  }
}

int ServoTester::constrainMotorSpeed(int speed) {
  return constrain(speed, -255, 255);
}

bool ServoTester::checkSafetyLimits() {
  // Check current limit
  float current = readCurrent();
  if (current > _currentLimitMA) {
    Serial.print(F("Current limit exceeded: "));
    Serial.print(current);
    Serial.println(F(" mA"));
    return false;
  }

  // Check voltage range
  float voltage = readVoltage();
  if (voltage < _voltageMin || voltage > _voltageMax) {
    Serial.print(F("Voltage out of range: "));
    Serial.print(voltage);
    Serial.println(F(" V"));
    return false;
  }

  return true;
}

void ServoTester::blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_STATUS_LED, HIGH);
    delay(100);
    digitalWrite(PIN_STATUS_LED, LOW);
    delay(100);
  }
}
