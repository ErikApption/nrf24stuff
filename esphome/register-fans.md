# Register Fans - ESP32-C3

Two 12V register (duct booster) fans controlled together via a single IRLZ44N MOSFET with RPM feedback.

## Wiring

### Power
- 12V supply powers the fans directly
- ESP32-C3 powered from USB or a buck converter off the 12V rail

### Fan Connections (per fan)
Typical 12V fan with tach wire has 3 pins:
| Wire   | Function | Notes                          |
|--------|----------|--------------------------------|
| Red    | +12V     | Direct to 12V supply           |
| Black  | GND      | To MOSFET drain                |
| Yellow | Tach     | RPM signal to ESP GPIO         |

### MOSFET (IRLZ44N) - single, shared by both fans
Both fan GND wires connect to the same MOSFET drain:

```
Fan 1 GND (black) ──┐
                     ├── MOSFET Drain
Fan 2 GND (black) ──┘
                         MOSFET Source → GND
                         MOSFET Gate  → GPIO0 (with 10k pulldown to GND)
```

The IRLZ44N is logic-level, so 3.3V from the ESP gate drive is sufficient.

### GPIO Assignments
| GPIO | Function       |
|------|----------------|
| 0    | Fan PWM (gate) |
| 2    | Fan 1 Tach/RPM |
| 3    | Fan 2 Tach/RPM |
| 8    | Status LED     |

## Veroboard Layout

The ESP32-C3 is off-board, connected via a 4-pin SIP header (J2) with wires.

### Components on board
| Ref | Footprint | Value      | Description                    |
|-----|-----------|------------|--------------------------------|
| Q1  | TO220     | IRLZ44N   | Low-side MOSFET switch         |
| R1  | RESISTOR4 | 10k        | Gate pulldown                  |
| R2  | RESISTOR4 | 10k        | Fan 1 tach pullup to 3.3V     |
| R3  | RESISTOR4 | 10k        | Fan 2 tach pullup to 3.3V     |
| J1  | SIP2      | 12V_IN     | 12V DC power input             |
| J2  | SIP4      | ESP32_WIRES| Wires to ESP32-C3 (off-board)  |
| J3  | SIP3      | FAN1_CONN  | Fan 1 connector (12V/GND/Tach) |
| J4  | SIP3      | FAN2_CONN  | Fan 2 connector (12V/GND/Tach) |

### J2 (ESP32 wires) pinout
| Pin | Signal    | ESP32 GPIO |
|-----|-----------|------------|
| 1   | PWM_GATE  | GPIO0      |
| 2   | FAN1_TACH | GPIO2      |
| 3   | FAN2_TACH | GPIO3      |
| 4   | 3V3       | 3V3 out    |

### Netlist
The netlist file `register-fans.net` is in OrcadPCB2 format for direct import into VeroRoute.
Footprint names use VeroRoute built-in types (RESISTOR4 = 400mil resistor, SIP2/3/4, TO220).

## Notes
- PWM frequency is 1kHz — quiet enough for duct fans and works well with low-side MOSFET switching.
- External 10k pullups (R2, R3) on tach lines are more reliable than ESP internal pullups.
- `multiply: 0.5` in the pulse counter filter accounts for 2 pulses per revolution (standard for most fans). If your fans output 1 pulse/rev, change to `1.0`.
- Both RPM sensors are kept so you can detect if one fan stalls or has issues.
- ESP32 GND must be connected to board GND (via one of the fan connector grounds or J1).

## Netlist (register-fans.net)

The netlist is in **OrcadPCB2 format**, which is what VeeCAD imports.

### Format details
- File starts with `( {` followed by a comment, then component entries
- Each component: `( RefDes Value Footprint` followed by pin entries `( pin# netname )`
- File ends with `)` on its own line, then `*` to mark end-of-file
- Pin numbering: For the IRLZ44N (TO-220), pin 1 = Gate, pin 2 = Drain, pin 3 = Source

### Importing into VeeCAD
1. Open VeeCAD
2. File → Import Netlist
3. Select `register-fans.net`
4. VeeCAD will create components based on the footprint names — you may need to map them to VeeCAD library outlines:
   - `AXIAL-0.4` → standard axial resistor, 4 holes spacing
   - `TO-220` → 3-pin TO-220 package
   - `BARREL_JACK` → 2-pin power connector
   - `FAN1` / `FAN2` → 3-pin headers
   - `ESP32C3` → will need a custom outline or use a DIP-like multi-pin representation

### Net names
| Net              | Description                              |
|------------------|------------------------------------------|
| GND              | Common ground                            |
| +12V             | 12V supply rail                          |
| 3V3              | 3.3V from ESP32-C3 regulator             |
| PWM_GATE         | GPIO0 → MOSFET gate (with R1 pulldown)   |
| FAN_GND_SWITCHED | MOSFET drain → both fan ground wires     |
| FAN1_TACH        | Fan 1 tach → GPIO2 (with R2 pullup)      |
| FAN2_TACH        | Fan 2 tach → GPIO3 (with R3 pullup)      |
| GPIO0            | ESP32 pin (same net as PWM_GATE)         |
| GPIO2            | ESP32 pin (same net as FAN1_TACH)        |
| GPIO3            | ESP32 pin (same net as FAN2_TACH)        |
| GPIO8            | Status LED pin                           |
