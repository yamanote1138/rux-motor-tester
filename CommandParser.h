/*
 * CommandParser.h
 *
 * Serial command parser for the servo tester
 * Handles parsing and execution of user commands via serial terminal
 */

#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <Arduino.h>
#include "ServoTester.h"

class CommandParser {
public:
  CommandParser();

  // Main parsing function
  void parse(String command, ServoTester& servo);

private:
  // Command handlers
  void handlePower(String args, ServoTester& servo);
  void handleEmergency(ServoTester& servo);
  void handleStatus(ServoTester& servo);
  void handleMotor(String args, ServoTester& servo);
  void handleStop(ServoTester& servo);
  void handleRaw(String args, ServoTester& servo);
  void handleTarget(String args, ServoTester& servo);
  void handleGoto(String args, ServoTester& servo);
  void handlePosition(ServoTester& servo);
  void handleCalibrate(ServoTester& servo);
  void handlePID(String args, ServoTester& servo);
  void handleSweep(String args, ServoTester& servo);
  void handleStep(String args, ServoTester& servo);
  void handleMap(ServoTester& servo);
  void handleSpeed(ServoTester& servo);
  void handleConfig(String args, ServoTester& servo);
  void handleHelp();
  void handleReset(ServoTester& servo);

  // Utility functions
  String getFirstWord(String& str);
  int parseInt(String str);
  float parseFloat(String str);
};

#endif // COMMAND_PARSER_H
