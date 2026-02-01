# Teddy Ruxpin 5-Wire Servo Tester

An Arduino-based testing and characterization tool for vintage Teddy Ruxpin 5-wire servos. This project provides an interactive serial interface for reverse engineering and fully characterizing servos with unknown specifications.

## Features

- **Interactive Serial Terminal**: Control and test servos via simple text commands
- **Multiple Test Modes**: Sweep tests, step response, frequency analysis, duty cycle mapping
- **Real-time Monitoring**: Current, voltage, and position feedback measurements
- **Safety Protection**: Hardware and software safeguards including current limiting and emergency cutoff
- **Signal Generation**: Flexible PWM generation with adjustable frequency and duty cycle
- **Characterization Tools**: Systematic testing procedures to map control signals to servo behavior

## Hardware Requirements

- Arduino Uno or Nano (ATmega328P, 5V, 16MHz)
- N-channel MOSFET (IRLZ44N or similar)
- 1Ω 2W shunt resistor (for current sensing)
- Schottky diode 1N5819 (reverse polarity protection)
- Resistors: 10kΩ (×4), 1kΩ (×2), 100kΩ (×2)
- Zener diodes: 5.1V (×2) (overvoltage protection)
- Teddy Ruxpin 5-wire servo (specifications unknown)

## Circuit Overview

**Pin Connections:**
- **Pin 9**: Control Signal 1 output (PWM capable)
- **Pin 10**: Control Signal 2 output (PWM capable)
- **Pin A0**: Position feedback input
- **Pin A1**: Auxiliary feedback input
- **Pin A2**: Current sense (via 1Ω shunt)
- **Pin A3**: Voltage monitor
- **Pin 8**: Emergency cutoff (MOSFET gate)
- **Pin 13**: Status LED

See `CIRCUIT.md` for complete wiring diagrams and connection details.

## Getting Started

1. **Assemble Circuit**: Follow the wiring diagram in `CIRCUIT.md`
2. **Upload Sketch**: Load `rux_motor_tester.ino` to your Arduino
3. **Connect Serial**: Open Serial Monitor at 115200 baud
4. **Follow Testing Guide**: Use `TESTING_GUIDE.md` for systematic characterization

## Usage

### Basic Commands

```
POWER ON              # Enable servo power
STATUS                # Display current readings
SET 9 128             # Set control pin 9 to value 128 (0-255)
PWM 9 50 50           # Generate 50Hz PWM at 50% duty cycle on pin 9
READ A0               # Read analog value from pin A0
SWEEP 9 0 255 10 100  # Sweep pin 9 from 0 to 255, step 10, delay 100ms
EMERGENCY             # Emergency stop
HELP                  # Show all commands
```

### Testing Workflow

1. **Wire Identification**: Determine which wire is power, ground, control, and feedback
2. **Signal Characterization**: Find optimal frequency and duty cycle ranges
3. **Position Mapping**: Map control signals to physical positions
4. **Documentation**: Record findings for future reference

See `TESTING_GUIDE.md` for detailed step-by-step procedures.

## Safety Features

- **Current Limiting**: Software limit at 500mA (configurable)
- **Hardware Cutoff**: MOSFET-based emergency stop (<1ms response)
- **Voltage Monitoring**: Detects supply issues before damage occurs
- **Continuous Monitoring**: Safety checks every 10ms
- **Timeout Protection**: Prevents runaway tests

## Warning Signs - Stop Immediately If:

- ⚠️ Smoke or burning smell
- ⚠️ Excessive heat (>60°C)
- ⚠️ Grinding or unusual mechanical sounds
- ⚠️ Current draw >300mA for >5 seconds

## Project Status

This is a reverse engineering tool for vintage electronics. Servo specifications are unknown and must be discovered through systematic testing. Always start with minimum power levels and short test durations.

## Documentation

- `CIRCUIT.md` - Detailed wiring diagrams and circuit design
- `TESTING_GUIDE.md` - Step-by-step characterization procedures
- `RESEARCH_NOTES.md` - Template for documenting servo findings

## License

This project is provided as-is for educational and reverse engineering purposes. Use at your own risk when working with vintage electronics.

## Contributing

Contributions welcome! If you've successfully characterized Teddy Ruxpin servos using this tool, please share your findings.
