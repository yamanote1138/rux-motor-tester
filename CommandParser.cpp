/*
 * CommandParser.cpp
 *
 * Implementation of command parser for serial terminal interface
 */

#include "CommandParser.h"

CommandParser::CommandParser() {
  // Constructor
}

void CommandParser::parse(String command, ServoTester& servo) {
  command.trim();
  command.toUpperCase();

  if (command.length() == 0) {
    return;
  }

  // Extract first word (the command)
  String cmd = getFirstWord(command);
  String args = command.substring(cmd.length());
  args.trim();

  // Route to appropriate handler
  if (cmd == "POWER") {
    handlePower(args, servo);
  } else if (cmd == "EMERGENCY" || cmd == "STOP!") {
    handleEmergency(servo);
  } else if (cmd == "STATUS") {
    handleStatus(servo);
  } else if (cmd == "MOTOR") {
    handleMotor(args, servo);
  } else if (cmd == "STOP") {
    handleStop(servo);
  } else if (cmd == "RAW") {
    handleRaw(args, servo);
  } else if (cmd == "TARGET") {
    handleTarget(args, servo);
  } else if (cmd == "GOTO") {
    handleGoto(args, servo);
  } else if (cmd == "POSITION" || cmd == "POS") {
    handlePosition(servo);
  } else if (cmd == "CALIBRATE" || cmd == "CAL") {
    handleCalibrate(servo);
  } else if (cmd == "PID") {
    handlePID(args, servo);
  } else if (cmd == "SWEEP") {
    handleSweep(args, servo);
  } else if (cmd == "STEP") {
    handleStep(args, servo);
  } else if (cmd == "MAP") {
    handleMap(servo);
  } else if (cmd == "SPEED") {
    handleSpeed(servo);
  } else if (cmd == "CONFIG") {
    handleConfig(args, servo);
  } else if (cmd == "HELP" || cmd == "?") {
    handleHelp();
  } else if (cmd == "RESET") {
    handleReset(servo);
  } else {
    Serial.print(F("Unknown command: "));
    Serial.println(cmd);
    Serial.println(F("Type HELP for command list"));
  }
}

// Command handlers
void CommandParser::handlePower(String args, ServoTester& servo) {
  args.trim();
  if (args == "ON") {
    servo.powerOn();
  } else if (args == "OFF") {
    servo.powerOff();
  } else {
    Serial.println(F("Usage: POWER ON|OFF"));
  }
}

void CommandParser::handleEmergency(ServoTester& servo) {
  Serial.println(F("!!! EMERGENCY STOP !!!"));
  servo.emergencyStop();
}

void CommandParser::handleStatus(ServoTester& servo) {
  servo.printStatus();
}

void CommandParser::handleMotor(String args, ServoTester& servo) {
  int speed = parseInt(args);
  if (speed < -255 || speed > 255) {
    Serial.println(F("Error: Speed must be -255 to 255"));
    return;
  }
  servo.setMotorSpeed(speed);
  Serial.print(F("Motor speed set to: "));
  Serial.println(speed);
}

void CommandParser::handleStop(ServoTester& servo) {
  servo.stopMotor();
  servo.enablePositionControl(false);
  Serial.println(F("Motor stopped"));
}

void CommandParser::handleRaw(String args, ServoTester& servo) {
  // Parse "dir pwm"
  String dirStr = getFirstWord(args);
  String pwmStr = args.substring(dirStr.length());
  pwmStr.trim();

  int dir = parseInt(dirStr);
  int pwm = parseInt(pwmStr);

  if (dir < 0 || dir > 2) {
    Serial.println(F("Error: dir must be 0 (stop), 1 (forward), or 2 (reverse)"));
    return;
  }

  if (pwm < 0 || pwm > 255) {
    Serial.println(F("Error: pwm must be 0-255"));
    return;
  }

  servo.setMotorRaw(dir, pwm);
  Serial.print(F("Raw motor control: dir="));
  Serial.print(dir);
  Serial.print(F(", pwm="));
  Serial.println(pwm);
}

void CommandParser::handleTarget(String args, ServoTester& servo) {
  int target = parseInt(args);
  if (target < 0 || target > 1023) {
    Serial.println(F("Error: Target must be 0-1023"));
    return;
  }

  servo.setTargetPosition(target);
  if (!servo.isPositionControlEnabled()) {
    servo.enablePositionControl(true);
  }
}

void CommandParser::handleGoto(String args, ServoTester& servo) {
  int target = parseInt(args);
  if (target < 0 || target > 1023) {
    Serial.println(F("Error: Target must be 0-1023"));
    return;
  }

  Serial.print(F("Moving to position "));
  Serial.println(target);

  servo.setTargetPosition(target);
  servo.enablePositionControl(true);

  // Wait for position to be reached
  unsigned long startTime = millis();
  while (abs(servo.getPositionError()) > 5 && (millis() - startTime < 10000)) {
    servo.update();
    delay(50);
  }

  if (abs(servo.getPositionError()) <= 5) {
    Serial.println(F("Position reached"));
  } else {
    Serial.println(F("Timeout - position not reached"));
  }

  servo.printStatus();
}

void CommandParser::handlePosition(ServoTester& servo) {
  int pos = servo.readPosition();
  Serial.print(F("Current position: "));
  Serial.println(pos);
}

void CommandParser::handleCalibrate(ServoTester& servo) {
  servo.calibrateRange();
}

void CommandParser::handlePID(String args, ServoTester& servo) {
  // Parse "Kp Ki Kd"
  String kpStr = getFirstWord(args);
  args = args.substring(kpStr.length());
  args.trim();

  String kiStr = getFirstWord(args);
  args = args.substring(kiStr.length());
  args.trim();

  String kdStr = args;

  float Kp = parseFloat(kpStr);
  float Ki = parseFloat(kiStr);
  float Kd = parseFloat(kdStr);

  servo.setPIDGains(Kp, Ki, Kd);
}

void CommandParser::handleSweep(String args, ServoTester& servo) {
  // Parse "MOTOR|POS start end step delay"
  String type = getFirstWord(args);
  args = args.substring(type.length());
  args.trim();

  String startStr = getFirstWord(args);
  args = args.substring(startStr.length());
  args.trim();

  String endStr = getFirstWord(args);
  args = args.substring(endStr.length());
  args.trim();

  String stepStr = getFirstWord(args);
  args = args.substring(stepStr.length());
  args.trim();

  String delayStr = args;

  int start = parseInt(startStr);
  int end = parseInt(endStr);
  int step = parseInt(stepStr);
  int delayMs = parseInt(delayStr);

  if (step <= 0) {
    Serial.println(F("Error: step must be positive"));
    return;
  }

  if (delayMs < 0) {
    Serial.println(F("Error: delay must be non-negative"));
    return;
  }

  if (type == "MOTOR") {
    servo.runManualSweep(start, end, step, delayMs);
  } else if (type == "POS" || type == "POSITION") {
    servo.enablePositionControl(true);
    servo.runPositionSweep(start, end, step, delayMs);
  } else {
    Serial.println(F("Usage: SWEEP MOTOR|POS start end step delay"));
  }
}

void CommandParser::handleStep(String args, ServoTester& servo) {
  int target = parseInt(args);
  if (target < 0 || target > 1023) {
    Serial.println(F("Error: Target must be 0-1023"));
    return;
  }

  servo.enablePositionControl(true);
  servo.runStepResponse(target);
}

void CommandParser::handleMap(ServoTester& servo) {
  servo.enablePositionControl(true);
  servo.runPositionMapping();
}

void CommandParser::handleSpeed(ServoTester& servo) {
  servo.runSpeedTest();
}

void CommandParser::handleConfig(String args, ServoTester& servo) {
  // Parse "param value"
  String param = getFirstWord(args);
  String value = args.substring(param.length());
  value.trim();

  if (param == "CURRENT") {
    float limit = parseFloat(value);
    servo.setCurrentLimit(limit);
  } else if (param == "DEADBAND") {
    int deadband = parseInt(value);
    servo.setDeadband(deadband);
  } else if (param == "TIMEOUT") {
    unsigned long timeout = (unsigned long)parseInt(value);
    servo.setTimeout(timeout);
  } else if (param == "SHOW") {
    servo.printConfiguration();
  } else {
    Serial.println(F("Usage: CONFIG CURRENT|DEADBAND|TIMEOUT value"));
    Serial.println(F("       CONFIG SHOW"));
  }
}

void CommandParser::handleHelp() {
  Serial.println(F("======================================"));
  Serial.println(F("       COMMAND REFERENCE"));
  Serial.println(F("======================================"));
  Serial.println();
  Serial.println(F("POWER CONTROL:"));
  Serial.println(F("  POWER ON|OFF       - Enable/disable motor power"));
  Serial.println(F("  EMERGENCY          - Immediate emergency stop"));
  Serial.println(F("  STATUS             - Display all current readings"));
  Serial.println();
  Serial.println(F("MANUAL MOTOR CONTROL:"));
  Serial.println(F("  MOTOR <speed>      - Set motor speed (-255 to 255)"));
  Serial.println(F("  STOP               - Stop motor immediately"));
  Serial.println(F("  RAW <dir> <pwm>    - Direct H-Bridge control"));
  Serial.println(F("                       dir: 0=stop, 1=fwd, 2=rev"));
  Serial.println();
  Serial.println(F("POSITION CONTROL:"));
  Serial.println(F("  TARGET <value>     - Set target position (0-1023)"));
  Serial.println(F("  GOTO <value>       - Move to position and wait"));
  Serial.println(F("  POSITION           - Read current position"));
  Serial.println(F("  CALIBRATE          - Auto-calibrate position range"));
  Serial.println(F("  PID <Kp> <Ki> <Kd> - Set PID gains"));
  Serial.println();
  Serial.println(F("CHARACTERIZATION TESTS:"));
  Serial.println(F("  SWEEP MOTOR <start> <end> <step> <delay>"));
  Serial.println(F("                     - Motor speed sweep test"));
  Serial.println(F("  SWEEP POS <start> <end> <step> <delay>"));
  Serial.println(F("                     - Position sweep test"));
  Serial.println(F("  STEP <target>      - Step response test"));
  Serial.println(F("  MAP                - Complete position mapping"));
  Serial.println(F("  SPEED              - Speed characterization"));
  Serial.println();
  Serial.println(F("CONFIGURATION:"));
  Serial.println(F("  CONFIG CURRENT <mA>     - Set current limit"));
  Serial.println(F("  CONFIG DEADBAND <units> - Set position deadband"));
  Serial.println(F("  CONFIG TIMEOUT <ms>     - Set test timeout"));
  Serial.println(F("  CONFIG SHOW             - Show configuration"));
  Serial.println();
  Serial.println(F("UTILITY:"));
  Serial.println(F("  HELP               - Show this help"));
  Serial.println(F("  RESET              - Reset to safe defaults"));
  Serial.println(F("======================================"));
  Serial.println();
}

void CommandParser::handleReset(ServoTester& servo) {
  Serial.println(F("Resetting to safe defaults..."));
  servo.powerOff();
  servo.stopMotor();
  servo.enablePositionControl(false);
  servo.setPIDGains(1.0, 0.0, 0.0);
  servo.setCurrentLimit(DEFAULT_CURRENT_LIMIT_MA);
  servo.setDeadband(DEFAULT_POSITION_DEADBAND);
  servo.setTimeout(DEFAULT_TIMEOUT_MS);
  Serial.println(F("Reset complete"));
}

// Utility functions
String CommandParser::getFirstWord(String& str) {
  str.trim();
  int spaceIndex = str.indexOf(' ');
  if (spaceIndex == -1) {
    return str;
  }
  return str.substring(0, spaceIndex);
}

int CommandParser::parseInt(String str) {
  str.trim();
  return str.toInt();
}

float CommandParser::parseFloat(String str) {
  str.trim();
  return str.toFloat();
}
