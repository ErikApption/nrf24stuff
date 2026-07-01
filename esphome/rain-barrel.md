# ESP32-C3 Motorized Ball Valve Control

> **Note:** The ESP32-C4 does not exist as a product. The closest small/low-pin-count options are the **ESP32-C3** (RISC-V, single core, WiFi + BLE) or the **ESP32-C6** (adds Thread/Zigbee). This document uses the ESP32-C3 as it's the most common. Swap for C6 if you need Thread support.

## Overview

Control a 12V normally-closed motorized ball valve using an ESP32-C3 dev board and a dual high-power MOSFET trigger module, powered by a 12V LiFePO4 battery with solar charging. The ESP32 stays awake continuously, maintaining a WiFi connection to Home Assistant for remote control and sensor reporting.

The valve is **normally closed** (spring-return) — applying 12V opens it, removing power allows it to close. The MOSFET module switches 12V to the valve, controlled by a GPIO signal from the ESP32.

---

## Motorized Valve Control

- Model: yyzk-4005
- Specs: 6W 12V DC


## PWM MOSFET Trigger Module

- 400W Dual High-Power MOSFET Trigger Module
- 0-20KHz PWM capable
- 5V-36V input, 15A continuous (30A max)
- Trigger input: 3.3V/5V logic compatible

The module acts as a high-current switch. When the trigger input is HIGH, the MOSFETs conduct and pass 12V to the valve. When LOW, the valve loses power and spring-returns to closed.

> **Note:** Although this module supports PWM speed control, we use it as a simple on/off switch (GPIO HIGH/LOW) since the ball valve only needs full power to open.


## XKC-Y25-NPN

- 5-12V version

Wiring:

- Brown wire (VCC) Power +5V~24V (Power+)
- Yellow wire (OUT) Signal output
- Black wire (M) Output level control
	- When the black wire connects high level, yellow wire is positive output signal wire, output is high level when object is sensed (NPN closed)
	- When the black wire connects low level, yellow wire is negative output signal wire, output is low level when object is sensed (NPN disconnect)
- Blue wire (GND) Ground wire (connect Power-)


## Parts List

| Component | Qty | Example | Notes |
|-----------|-----|---------|-------|
| ESP32-C3 dev board | 1 | ESP32-C3 SuperMini, Seeed XIAO ESP32-C3 | 3.3V logic |
| Dual high-power MOSFET trigger module | 1 | 400W 5V-36V 15A PWM module | Switches 12V to valve, 3.3V logic input |
| 12V normally-closed motorized ball valve | 1 | yyzk-4005, 2-wire, NC spring-return | Apply 12V to open, remove to close. 6W (~500mA at 12V) |
| XKC-Y25-NPN non-contact liquid level sensor | 2 | XKC-Y25-NPN (5–12V version) | NPN open-collector output, needs pull-up to 3.3V |
| 12V LiFePO4 battery | 1 | 4S 12.8V 5Ah | ~64 Wh capacity |
| Solar charge controller | 1 | Victron 75/10, EPever 10A | Must support LiFePO4 profile |
| Solar panel | 1 | 20–30W 18V monocrystalline | Sized for ~5–8 Wh/day load |
| Buck converter 12V → 5V | 1 | Mini-360, MP1584 module | Powers ESP32 dev board via 5V/VIN pin |
| Momentary push button (TOGGLE) | 1 | Normally-open, panel mount | Manual valve open/close toggle |
| LED indicator | 1 | 3mm or 5mm, any color (green/red) | Indicates valve is open |
| LED current-limiting resistor | 1 | 470Ω (for 3.3V) or 1kΩ | Limits LED current to ~5mA |
| Pull-up resistors | 2 | 10kΩ | Pull NPN sensor output up to 3.3V for ESP32 GPIO |
| Voltage divider resistor (high) | 1 | 100kΩ | Solar voltage sense — top of divider |
| Voltage divider resistor (low) | 1 | 15kΩ | Solar voltage sense — bottom of divider |
| Filter capacitor | 1 | 100nF | Smooths ADC reading on voltage divider |
| Wiring | — | 18–22 AWG for valve/power, 24 AWG for logic | |

> **Note:** No external pull-down resistor needed for the button — the ESP32-C3 internal pull-down (~45kΩ) is used.

---

## Circuit Diagram (ASCII)

```
    ┌──────────────┐       ┌────────────────┐
    │  Solar Panel │──────►│  Solar Charge  │
    │  18V 20-30W  │       │  Controller    │
    └──────────────┘       │  (LiFePO4)     │
                           └───────┬────────┘
                                   │
                           ┌───────┴────────┐
                           │ 12.8V LiFePO4  │
                           │ 4S 5Ah Battery │
                           └───────┬────────┘
                                   │ 12V bus
              ┌────────────────────┼────────────────┐
              │                    │                │
         ┌────┴────┐         ┌────┴────┐      ┌────┴────┐
         │ Buck 5V │         │ MOSFET  │      │ Sensor  │
         │ Conv.   │         │ Module  │      │ VCC     │
         └────┬────┘         └────┬────┘      └────┬────┘
              │                   │                │
              ▼                   ▼                ▼
        ┌──────────┐      Valve Motor       XKC-Y25 ×2
        │ ESP32-C3 │
        │          │
        │  GPIO4 ──────► MOSFET Module Trigger (valve power)
        │          │
        │  GPIO6 ◄────── XKC-Y25 #1 Signal (HIGH level)
        │  GPIO7 ◄────── XKC-Y25 #2 Signal (MID level)
        │          │
        │  GPIO2 ◄────── Manual TOGGLE button (internal pull-down)
        │  GPIO10──────► LED indicator (valve open)
        │          │
        │  GPIO0 ◄────── Solar voltage divider (100k/15k)
        │          │
        │  3V3  ───────► Pull-up resistors (2× 10kΩ), button VCC
        │  GND  ───────► MOSFET module GND, sensor GND
        │  5V/VIN ◄───── Buck converter 5V out
        │  GND  ───────── Common GND
        └──────────┘
```

### MOSFET Module Valve Wiring

The dual MOSFET trigger module switches 12V to the normally-closed valve motor.

```
    12V+ ──────────── MOSFET Module VIN+
    GND  ──────────── MOSFET Module VIN-

    MOSFET Module OUT+ ──── Valve Wire +
    MOSFET Module OUT- ──── Valve Wire - (or GND)

    ESP32 GPIO4 ─────────── MOSFET Module TRIG (trigger/signal input)
    ESP32 GND ───────────── MOSFET Module GND (common ground)

    State table:
    ┌──────────┬──────────────────────────┐
    │ GPIO4    │ Result                   │
    ├──────────┼──────────────────────────┤
    │  LOW     │ MOSFET off → valve CLOSED│
    │  HIGH    │ MOSFET on → valve OPEN   │
    └──────────┴──────────────────────────┘
```

**Wiring:**
- MOSFET module VIN+ → 12V battery +
- MOSFET module VIN- → GND
- MOSFET module OUT+ → Valve wire +
- MOSFET module OUT- → Valve wire - (GND)
- MOSFET module TRIG → ESP32 GPIO4
- MOSFET module GND → ESP32 GND (common ground)

When GPIO4 goes HIGH, the MOSFET module conducts and passes 12V to the valve motor, opening it. When GPIO4 goes LOW, the valve loses power and spring-returns to closed.

> **Note:** The valve stays open only while GPIO4 is HIGH. This draws continuous current (~500mA from the valve at 6W/12V). Plan your duty cycle accordingly for battery life.

---

## XKC-Y25-NPN Level Sensor Wiring

The XKC-Y25-NPN is a non-contact capacitive liquid level sensor. It detects water through the container wall (up to ~20mm plastic/glass). The NPN version has an open-collector output — it pulls the signal line to GND when water is detected.

### Sensor Pinout

| Wire Color | Function |
|------------|----------|
| Brown | VCC (5–12V) |
| Blue | GND |
| Yellow | Signal output (NPN open-collector) |
| Black | Mode select (tie to GND for NPN active-low output) |

### Pull-Up Resistor Circuit

Since the output is NPN open-collector, you need a pull-up resistor to 3.3V for the ESP32 to read it properly:

```
    ESP32 3.3V ──────┐
                     │
                   ┌─┴─┐
                   │10k│  (pull-up resistor)
                   └─┬─┘
                     │
                     ├──────► ESP32 GPIO (6 or 7)
                     │
              ┌──────┴──────┐
              │  XKC-Y25    │
              │  Yellow     │
              │  (Signal)   │
              └─────────────┘

    XKC-Y25 Brown (VCC) ───► 12V bus
    XKC-Y25 Blue  (GND) ───► Common GND
    XKC-Y25 Black (Mode) ──► GND (NPN active-low)
```

### Logic Levels

| Water Present? | NPN Transistor | GPIO Reading |
|---------------|----------------|--------------|
| No | OFF (open) | HIGH (pulled to 3.3V) |
| Yes | ON (sinks to GND) | LOW |

> **Note:** The sensor reads **inverted** — LOW = water detected. The ESPHome config uses `inverted: true` in the pin config so that `ON` = water present.

### Sensor Placement (Rain Barrel)

```
    ┌─────────────────────┐
    │                     │  ← XKC #1 (HIGH) — near top / full
    │   ~~~water~~~       │
    │                     │
    │                     │  ← XKC #2 (MID)  — half level
    │                     │
    │                     │
    └─────────────────────┘
         ▼ valve outlet
```

---

## Manual Override Button & LED Indicator

A single momentary push button toggles the valve open/closed. An LED lights up whenever the valve is in the open position, providing visual feedback.

### Button Wiring

The button uses the ESP32-C3's internal pull-down resistor — no external resistor needed.

```
    ESP32 3.3V ─────────────────┐
                                │
                           ┌────┴────┐
                           │  Button │ (momentary, normally open)
                           └────┬────┘
                                │
                                └──────► ESP32 GPIO2 (TOGGLE, internal pull-down)
```

When not pressed, the internal pull-down holds GPIO2 LOW. Pressing the button connects GPIO2 to 3.3V (HIGH), triggering the toggle.

### LED Indicator Wiring

```
    ESP32 GPIO10 ────┐
                     │
                   ┌─┴─┐
                   │470Ω│  (current-limiting resistor)
                   └─┬─┘
                     │
                  ┌──┴──┐
                  │ LED  │  (anode)
                  └──┬──┘
                     │      (cathode)
                    GND
```

- LED ON = valve is open
- LED OFF = valve is closed

### Solar Panel Voltage Sensing

A resistor voltage divider scales the solar panel voltage (0–21V open-circuit) down to the ESP32-C3's ADC range (0–3.3V). The divider uses 100kΩ (high-side) and 15kΩ (low-side), giving a division ratio of 15/115 = 0.13. A 100nF capacitor across the low-side resistor filters noise.

```
    Solar Panel + ────┐
                      │
                    ┌─┴─┐
                    │100k│  R1 (high-side)
                    └─┬─┘
                      │
                      ├──────► ESP32 GPIO0 (ADC)
                      │
                    ┌─┴─┐         ┌───┐
                    │15k │  R2    ─┤100n├─  C1 (filter)
                    └─┬─┘         └─┬─┘
                      │             │
                     GND           GND
```

| Solar Panel Voltage | ADC Voltage (at GPIO0) |
|--------------------|------------------------|
| 0V (night) | 0V |
| 12V (cloudy) | 1.57V |
| 18V (nominal) | 2.35V |
| 21V (open circuit) | 2.74V |

---

## ESPHome YAML Configuration

See [`rain-barrel.yaml`](rain-barrel.yaml) for the full ESPHome configuration.

Key features of the config:
- **Always-on**: No deep sleep — stays connected to WiFi/Home Assistant continuously
- **MOSFET module valve control**: GPIO4 triggers the MOSFET module to switch 12V to the normally-closed valve; LOW = closed
- **Manual toggle button**: GPIO2 with internal pull-down toggles the valve open/closed
- **LED indicator**: GPIO10 lights up while the valve is open
- **Two water level sensors**: HIGH and MID levels on GPIO6 and GPIO7

---

## Safety Notes

1. **Continuous current while open** — the normally-closed valve draws 6W (~500mA at 12V) the entire time it's open. Factor this into your battery budget.

2. **Common ground** — all GND connections (buck converter, MOSFET module, ESP32, battery, sensors) must share a common ground.

3. **OTA updates** — since there's no deep sleep, the ESP32 is always reachable for OTA flashing.

4. **MOSFET module heat** — at 500mA continuous the module will barely get warm (rated for 15A). No heatsink needed for this application.

---

## Wiring Summary

| ESP32-C3 Pin | Connects To |
|--------------|-------------|
| GPIO0 | Solar voltage divider midpoint (100kΩ/15kΩ + 100nF cap) |
| GPIO2 | Manual TOGGLE button (internal pull-down, button connects to 3.3V) |
| GPIO4 | MOSFET trigger module signal input (valve power) |
| GPIO6 | XKC-Y25 #1 signal (HIGH level) + 10kΩ pull-up to 3.3V |
| GPIO7 | XKC-Y25 #2 signal (MID level) + 10kΩ pull-up to 3.3V |
| GPIO10 | LED indicator (+ 470Ω resistor → LED → GND) |
| 5V (VIN) | Buck converter 5V output |
| 3V3 | Pull-up resistors (2× 10kΩ), button VCC |
| GND | Common ground rail |

| 12V Battery | Connects To |
|-------------|-------------|
| 12V+ | Buck converter input +, MOSFET module VIN+, XKC-Y25 VCC (brown) ×2 |
| GND | Buck converter input -, MOSFET module VIN-, XKC-Y25 GND (blue) ×2, Common ground |

## Power Budget (Estimated)

| State | Current Draw (12V) | Notes |
|-------|-------------------|-------|
| Idle (WiFi connected) | ~80 mA | ESP32 + buck converter + sensors |
| Valve open | ~580 mA | + 500mA valve motor |

**Idle battery life** (no solar): ~2.6 days on 5Ah LiFePO4.
With a 20–30W solar panel, the system is self-sustaining for continuous operation.
