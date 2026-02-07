# Circuit Wiring Diagram

## TB6612FNG H-Bridge Motor Driver Configuration

This document describes the complete wiring for the Teddy Ruxpin 5-wire servo tester using the TB6612FNG motor driver.

---

## Overview

The TB6612FNG is a dual-channel H-Bridge motor driver with excellent performance characteristics:
- **Current Capability**: 1.2A continuous per channel (3.2A peak)
- **Voltage Drop**: ~0.5V (much better than L293D's 1.4V)
- **Built-in Protection**: Thermal shutdown, under-voltage lockout
- **Standby Feature**: STBY pin for emergency cutoff (no external MOSFET needed!)

We use only **Motor A channel** for the servo, leaving Motor B available for future expansion.

---

## Pin Connections

### Arduino Uno → TB6612FNG

| Arduino Pin | TB6612 Pin | Function | Notes |
|-------------|------------|----------|-------|
| 5V          | VCC        | Logic power | Powers TB6612 logic |
| GND         | GND        | Ground | Common ground |
| Pin 5 (PWM) | PWMA       | Motor A speed | PWM signal (0-255) |
| Pin 6       | AIN1       | Motor A direction | Direction control |
| Pin 7       | AIN2       | Motor A direction | Direction control |
| Pin 8       | STBY       | Standby enable | HIGH=enabled, LOW=emergency stop |

### TB6612FNG → Servo Motor (2 wires)

| TB6612 Pin | Servo Wire | Function |
|------------|------------|----------|
| AO1        | Motor wire 1 | Typically Red or Blue |
| AO2        | Motor wire 2 | Typically Black or Grey |

**Note**: If motor runs backwards, simply swap AO1 and AO2 connections.

### TB6612FNG Motor Power

| TB6612 Pin | Connection | Function |
|------------|------------|----------|
| VM         | +5V (via 0.5Ω shunt) | Motor power supply |
| GND        | Ground (common) | Motor ground return |

---

## Servo Potentiometer Connections (3 wires)

| Arduino Pin | Servo Pot Wire | Function | Notes |
|-------------|----------------|----------|-------|
| 5V          | Pot VCC | Potentiometer power | Typically Red/Yellow |
| GND         | Pot GND | Potentiometer ground | Typically Black/Brown |
| A0          | Pot Wiper | Position feedback | Typically White/Green/Grey |
| A1          | Pot VCC (tap) | VCC monitor | Optional test point |

**Wire Identification Tips**:
- Measure resistance: Motor wires = 5-50Ω, Pot wires = 1-10kΩ
- Pot VCC and GND will measure ~5-10kΩ between them
- Wiper will measure variable resistance to VCC and GND as motor moves

---

## Current and Voltage Monitoring

### Current Sense Circuit

```
External 5V Supply (+)
    |
    |--- [Schottky Diode 1N5819] --- (anode to +, cathode forward)
    |
    +--- [0.5Ω 2W Shunt Resistor] ---+
    |                                 |
    +-- (To Arduino A2) ------------ (voltage sense)
    |
    +--- TB6612 VM pin
```

**Current Calculation**:
- Voltage across 0.5Ω resistor: V = I × 0.5Ω
- Arduino ADC reading: 0-1023 (0-5V range)
- Current (mA) = ADC_value × 9.77

### Voltage Monitor Circuit

```
External 5V Supply (+)
    |
    +--- [10kΩ] ---+--- (To Arduino A3)
                   |
                   +--- [10kΩ] --- GND
```

**Voltage Calculation**:
- 2:1 voltage divider
- Actual voltage = ADC_value × 0.00977

---

## Protection Circuit

### 1. Reverse Polarity Protection
- **Component**: 1N5819 Schottky diode
- **Connection**: Anode to external 5V supply, cathode to shunt resistor
- **Purpose**: Prevents damage if power supply connected backwards

### 2. Emergency Cutoff
- **Component**: TB6612 built-in STBY pin
- **Connection**: Arduino Pin 8 to STBY, 10kΩ pulldown to GND
- **Purpose**: Pulling STBY LOW immediately disables all motor outputs
- **Advantage**: No external MOSFET needed!

### 3. Overvoltage Protection (Pot Wiper)
- **Components**: 10kΩ series resistor + 5.1V zener diode
- **Connection**: Between pot wiper and Arduino A0
- **Purpose**: Protects Arduino if pot gets faulty voltage

### 4. Motor Noise Suppression
- **Components**: 100nF ceramic capacitors (2x)
- **Connection**: Across motor terminals (AO1 to AO2) and across motor power (VM to GND)
- **Purpose**: Reduces electrical noise and EMI

---

## Complete Wiring Diagram (ASCII Art)

```
                    External 5V Supply (1-2A)
                           |
                           +--- [1N5819 Diode] ---+
                           |                       |
                           |        VM Power      |
                           |          Path        |
                      [0.5Ω, 2W] ←── Current ─────┘
                       Shunt       Measurement
                           |           ↓
                           |      (To A2 via 10kΩ)
                           |
                    +------+------+
                    |   TB6612    |
                    |   DRIVER    |
                    +-------------+
Arduino Uno         |             |        5-Wire Servo
                    | VCC  ←  5V  |
+---------+         | GND  ←  GND |        Motor (2 wires)
|         |         | VM   ←  5V* |        +-------+
| Pin 5 ──┼─ PWM ──→ PWMA         |        |       |
| Pin 6 ──┼────────→ AIN1         |   AO1 ─┼─ Red  |
| Pin 7 ──┼────────→ AIN2         |        |       |
| Pin 8 ──┼────────→ STBY         |   AO2 ─┼─ Blk  |
|         |         |      [100nF] |        +-------+
|         |         | BIN1   cap   |
|         |         | BIN2  ⊥ GND  |        Pot (3 wires)
|         |         | PWMB         |        +--------+
|         |         +-------------+    5V ─┼─ VCC   |
|         |                                 |        |
| A0  ←───┼─── [10kΩ + Zener] ←─── Wiper ─┼─ WHT   |
| A1  ←───┼─────────────────────── VCC ────┼─ (tap) |
| A2  ←───┼─── Current Sense (via 10kΩ)    |        |
| A3  ←───┼─── Voltage Sense (2:1 divider) |        |
|         |                            GND ─┼─ BLK   |
| 5V  ────┼───────────────────────────────→ (Pot)   |
| GND ────┼───────────────────────────────→ (Common)|
|         |                                 +--------+
| Pin 13 ─┼─── [LED + 220Ω] ─── GND
+---------+       Status LED

* VM connected via 0.5Ω shunt resistor (see above)
```

---

## TB6612FNG Truth Table

Motor control is determined by AIN1, AIN2, and PWMA:

| STBY | AIN1 | AIN2 | PWMA | Motor Action |
|------|------|------|------|--------------|
| LOW  | X    | X    | X    | **Motor OFF (emergency stop)** |
| HIGH | HIGH | LOW  | PWM  | **Forward (CW)** - Normal operation |
| HIGH | LOW  | HIGH | PWM  | **Reverse (CCW)** - Normal operation |
| HIGH | LOW  | LOW  | X    | **Short brake** (fast stop) |
| HIGH | HIGH | HIGH | X    | **Short brake** (fast stop) |

**Code Implementation**:
- Forward: `digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, speed);`
- Reverse: `digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, speed);`
- Stop: `digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0);`
- Emergency: `digitalWrite(STBY, LOW);`

---

## Parts List

### Required Components

| Component | Quantity | Specs | Purpose |
|-----------|----------|-------|---------|
| Arduino Uno | 1 | ATmega328P | Microcontroller |
| TB6612FNG | 1 | Dual H-Bridge | Motor driver |
| Power Supply | 1 | 5V, 1-2A | Motor power |
| Shunt Resistor | 1 | 0.5Ω, 2W | Current sensing |
| Schottky Diode | 1 | 1N5819 | Reverse polarity protection |
| Ceramic Caps | 2 | 100nF | Noise suppression |
| Resistors | 4 | 10kΩ | Voltage divider, protection |
| Zener Diode | 1 | 5.1V | Overvoltage protection |
| Status LED | 1 | Any color | Visual indicator |
| LED Resistor | 1 | 220Ω | LED current limiting |

### Optional but Recommended

| Component | Purpose |
|-----------|---------|
| Breadboard | Prototyping |
| Jumper wires | Connections |
| Barrel jack | Power input |
| Terminal blocks | Servo connections |
| Heat sink | TB6612 cooling (if needed) |

---

## Assembly Notes

### Step 1: Power Supply
1. Connect external 5V supply to barrel jack
2. Add 1N5819 diode for reverse polarity protection (stripe to +)
3. Add 0.5Ω shunt resistor in series with VM
4. Connect Arduino 5V and GND to breadboard power rails

### Step 2: TB6612FNG Module
1. Connect VCC to Arduino 5V (logic power)
2. Connect VM to external 5V via shunt resistor (motor power)
3. Connect all GND pins together (common ground is critical!)
4. Add 100nF capacitor across VM and GND on TB6612
5. Connect STBY pin to Arduino Pin 8 with 10kΩ pulldown to GND

### Step 3: Control Signals
1. Connect Arduino Pin 5 (PWM) to PWMA
2. Connect Arduino Pin 6 to AIN1
3. Connect Arduino Pin 7 to AIN2
4. Leave BIN1, BIN2, PWMB unconnected (or tie to GND)

### Step 4: Servo Motor
1. Identify 2 motor wires using ohmmeter (5-50Ω between them)
2. Connect to AO1 and AO2 (polarity doesn't matter yet)
3. Add 100nF capacitor across motor terminals
4. If motor runs backwards later, just swap the wires

### Step 5: Servo Potentiometer
1. Identify 3 pot wires using ohmmeter:
   - VCC-GND pair: ~5-10kΩ
   - Wiper: Variable resistance to both
2. Connect VCC to Arduino 5V
3. Connect GND to Arduino GND
4. Connect Wiper to A0 via 10kΩ resistor and 5.1V zener protection
5. Optional: Connect VCC tap to A1 for monitoring

### Step 6: Monitoring Circuits
1. Current sense: Connect A2 to junction between shunt and VM via 10kΩ
2. Voltage sense: Create 2:1 divider from 5V supply to A3
3. Status LED: Connect Pin 13 to LED + 220Ω resistor to GND

### Step 7: Testing
1. Upload Arduino code
2. Open Serial Monitor (115200 baud)
3. Type `STATUS` - should show Power OFF, 0mA current
4. Type `POWER ON` - STBY should go HIGH, status LED on
5. Type `MOTOR 50` - motor should turn slowly
6. Type `POSITION` - should show pot reading changing with movement
7. Type `POWER OFF` - everything stops

---

## Troubleshooting

### Motor doesn't move
- Check STBY is HIGH (Pin 8)
- Verify VM has 5V power
- Check AIN1/AIN2 connections
- Measure motor resistance (should be 5-50Ω)

### Motor runs backwards
- Simply swap AO1 and AO2 connections
- Or invert AIN1/AIN2 in code

### Position feedback doesn't change
- Check pot power (should be 5V between VCC and GND)
- Verify wiper connection to A0
- Manually rotate motor shaft and monitor A0 reading
- May need to swap pot VCC/GND if reading is inverted

### Current reading always zero
- Check shunt resistor placement (must be in VM power path)
- Verify A2 connection to voltage sense point
- Ensure 10kΩ series resistor for protection

### TB6612 gets hot
- Reduce motor speed
- Add heat sink to TB6612
- Check for motor stall or mechanical binding
- Verify current is below 500mA limit

---

## Safety Warnings

⚠️ **IMPORTANT SAFETY NOTES**:

1. **Current Limit**: Software default is 500mA. TB6612 can handle 1.2A, but vintage servos may not!
2. **Emergency Stop**: STBY pin (Pin 8) pulls LOW for instant motor cutoff
3. **Initial Testing**: Always start with `MOTOR 50` (low speed) to test direction and movement
4. **Smoke = Stop**: If you see smoke or smell burning, immediately type `EMERGENCY`
5. **Heat Check**: Touch servo and TB6612 after 10 seconds of operation - should be warm, not hot
6. **Wire Verification**: ALWAYS verify 5-wire identification with ohmmeter before connecting power
7. **Polarity**: Double-check power supply polarity (diode provides protection but still verify!)

---

## Next Steps

Once wiring is complete:
1. See **TESTING_GUIDE.md** for step-by-step characterization procedures
2. See **README.md** for command reference and usage
3. Document your findings in **RESEARCH_NOTES.md** (to be created)

---

*Document Version: 1.0*
*Last Updated: 2026-02-07*
*Hardware: Arduino Uno + TB6612FNG + Teddy Ruxpin 5-wire servo*
