# ESP32-C3 Motorized Ball Valve Control

> **Note:** The ESP32-C4 does not exist as a product. The closest small/low-pin-count options are the **ESP32-C3** (RISC-V, single core, WiFi + BLE) or the **ESP32-C6** (adds Thread/Zigbee). This document uses the ESP32-C3 as it's the most common. Swap for C6 if you need Thread support.

## Overview

Control a 12V normally-closed motorized ball valve using an ESP32-C3 dev board and a single relay, powered by a 12V LiFePO4 battery with solar charging. The system uses deep sleep to minimize power draw, waking periodically to read water level sensors and report status.

The valve is **normally closed** (spring-return or latching) — applying 12V opens it, removing power allows it to close. A single relay switches 12V to the valve. A MOSFET high-side switch powers the XKC-Y25 sensors only during the wake cycle.

---

## Motorized Valve Control

- Model: yyzk-4005
- Specs: 6W 12v DC 


## XKC-Y25-NPN

- 5-12v version

Wiring:

- Brown wire(VCC) Power +5V~24V (Power+)
- Yellow wire(OUT) Signal output
- Black wire(M) Output level(positive output or negative output) control
	- When the black wire connect the high level, yellow wire is positive output signal wire, the output is high level when the object is sensed.(NPN closed)
	- When the black wire connect the low level, yellow wire is negative output signal wire, the output is low level when the object is sensed(NPN disconnect)
- Blue wire(GND) Ground wire(connect Power-)


## Parts List

| Component | Qty | Example | Notes |
|-----------|-----|---------|-------|
| ESP32-C3 dev board | 1 | ESP32-C3 SuperMini, Seeed XIAO ESP32-C3 | 3.3V logic |
| 1-channel relay module (5V coil, 3.3V logic compatible) | 1 | SRD-05VDC, single-channel | Switches 12V to valve |
| 12V normally-closed motorized ball valve | 1 | 2-wire, NC spring-return | Apply 12V to open, remove to close. 6VA (~500mA at 12V) |
| XKC-Y25-NPN non-contact liquid level sensor | 3 | XKC-Y25-NPN (5–12V version) | NPN open-collector output, needs pull-up to 3.3V |
| 12V LiFePO4 battery | 1 | 4S 12.8V 5Ah | ~64 Wh capacity |
| Solar charge controller | 1 | Victron 75/10, EPever 10A | Must support LiFePO4 profile |
| Solar panel | 1 | 20–30W 18V monocrystalline | Sized for ~5–8 Wh/day load |
| Buck converter 12V → 5V | 1 | Mini-360, MP1584 module | Powers ESP32 dev board via 5V/VIN pin |
| P-channel MOSFET (high-side switch) | 1 | IRF9540N, AO3401 (SOT-23) | Switches 12V to sensors |
| N-channel MOSFET (gate driver) | 1 | 2N7000, BSS138 | Level-shifts 3.3V GPIO to drive P-FET gate |
| Gate pull-up resistor | 1 | 100kΩ | Holds P-FET OFF when GPIO is low |
| Gate pull-down resistor | 1 | 10kΩ | Pulls N-FET gate to GND when ESP32 is asleep |
| Flyback diode (optional) | 1 | 1N4007 | Across relay coil if using bare relay |
| Pull-up resistors | 3 | 10kΩ | Pull NPN sensor output up to 3.3V for ESP32 GPIO |
| Momentary push button (TOGGLE) | 1 | Normally-open, panel mount | Manual valve open/close toggle |
| LED indicator | 1 | 3mm or 5mm, any color (green/red) | Indicates valve is open |
| LED current-limiting resistor | 1 | 470Ω (for 3.3V) or 1kΩ | Limits LED current to ~5mA |
| Button pull-down resistor | 1 | 10kΩ | Hold GPIO LOW when button not pressed |
| Voltage divider resistor (high) | 1 | 100kΩ | Solar voltage sense — top of divider |
| Voltage divider resistor (low) | 1 | 15kΩ | Solar voltage sense — bottom of divider |
| Filter capacitor | 1 | 100nF | Smooths ADC reading on voltage divider |
| Wiring | — | 18–22 AWG for valve/power, 24 AWG for logic | |

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
         │ Buck 5V │         │  Relay  │      │ P-FET   │
         │ Conv.   │         │  (valve)│      │ (sensor)│
         └────┬────┘         └────┬────┘      └────┬────┘
              │                   │                │
              ▼                   ▼                ▼
        ┌──────────┐        Relay Contact     Sensor VCC
        │ ESP32-C3 │
        │          │
        │  GPIO4 ──────► Relay IN (valve power)
        │          │
        │  GPIO3 ──────► MOSFET sensor power switch
        │          │
        │  GPIO6 ◄────── XKC-Y25 #1 Signal (HIGH level)
        │  GPIO7 ◄────── XKC-Y25 #2 Signal (MID level)
        │  GPIO8 ◄────── XKC-Y25 #3 Signal (LOW level)
        │          │
        │  GPIO2 ◄────── Manual TOGGLE button (+ 10kΩ pull-down)
        │  GPIO10──────► LED indicator (valve open)
        │          │
        │  GPIO0 ◄────── Solar voltage divider (100k/15k)
        │          │
        │  3V3  ───────► Pull-up resistors (3× 10kΩ), button VCC
        │  GND  ───────► Relay GND
        │  5V/VIN ◄───── Buck converter 5V out
        │  GND  ───────── Common GND
        └──────────┘
```

### Relay Valve Wiring (Single SPDT Relay)

The normally-closed valve only needs power applied to open. A single relay switches 12V to the valve.

```
    12V+ ──────────┐
                   │
              ┌────┴────┐
              │  Relay  │
              │  COM    │
              ├─────────┤
              │ NC (unused)
              │ NO ─────────── Valve Wire + 
              └─────────┘
                                    │
    GND ────────────────────── Valve Wire -

    State table:
    ┌────────┬──────────────────────────┐
    │ Relay  │ Result                   │
    ├────────┼──────────────────────────┤
    │  OFF   │ No power → valve CLOSED │
    │  ON    │ 12V applied → valve OPEN│
    └────────┴──────────────────────────┘
```

**Wiring:**
- Relay COM → 12V+
- Relay NO (normally open) → Valve wire +
- Valve wire - → GND
- Relay NC → unused

When the relay is energized (GPIO4 HIGH), 12V is applied to the valve and it opens. When the relay is de-energized (GPIO4 LOW), the valve loses power and its spring returns it to closed.

> **Note:** The valve stays open only while the relay is energized. This draws continuous current (~500mA from the valve at 6VA/12V + ~70mA for the relay coil) while open. Plan your duty cycle accordingly for battery life.

---

## XKC-Y25-NPN Level Sensor Wiring

The XKC-Y25-NPN is a non-contact capacitive liquid level sensor. It detects water through the container wall (up to ~20mm plastic/glass). The NPN version has an open-collector output — it pulls the signal line to GND when water is detected.

### Sensor Pinout (3 wires)

| Wire Color | Function |
|------------|----------|
| Brown | VCC (5–12V) |
| Blue | GND |
| Yellow | Signal output (NPN open-collector) |

### Pull-Up Resistor Circuit

Since the output is NPN open-collector, you need a pull-up resistor to 3.3V for the ESP32 to read it properly:

```
    ESP32 3.3V ──────┐
                     │
                   ┌─┴─┐
                   │10k│  (pull-up resistor)
                   └─┬─┘
                     │
                     ├──────► ESP32 GPIO (6, 7, or 8)
                     │
              ┌──────┴──────┐
              │  XKC-Y25    │
              │  Yellow     │
              │  (Signal)   │
              └─────────────┘
              
    XKC-Y25 Brown (VCC) ───► 12V+ (or 5V rail)
    XKC-Y25 Blue  (GND) ───► Common GND
```

### Logic Levels

| Water Present? | NPN Transistor | GPIO Reading |
|---------------|----------------|--------------|
| No | OFF (open) | HIGH (pulled to 3.3V) |
| Yes | ON (sinks to GND) | LOW |

> **Note:** The sensor reads **inverted** — LOW = water detected. The ESPHome config below uses `inverted: true` in the pin config so that `ON` = water present.

### Sensor Placement (Rain Barrel)

```
    ┌─────────────────────┐
    │                     │  ← XKC #1 (HIGH) — near top / full
    │   ~~~water~~~       │
    │                     │  ← XKC #2 (MID)  — half level
    │                     │
    │                     │  ← XKC #3 (LOW)  — near bottom / low
    │                     │
    └─────────────────────┘
         ▼ valve outlet
```

---

## Manual Override Button & LED Indicator

A single momentary push button toggles the valve open/closed. An LED lights up whenever the valve is in the open position, providing visual feedback even when the ESP32 is in deep sleep (the LED is driven by a GPIO that retains state through sleep on ESP32-C3 with `esp-idf`).

The button also serves as a **wake-up source** during deep sleep — pressing it wakes the ESP32, which toggles the valve and updates the LED.

### Button Wiring

```
    ESP32 3.3V ─────────────────┐
                                │
                           ┌────┴────┐
                           │  Button │ (momentary, normally open)
                           └────┬────┘
                                │
                                ├──────► ESP32 GPIO2 (TOGGLE)
                                │
                              ┌─┴─┐
                              │10k│  (pull-down to GND)
                              └─┬─┘
                                │
                               GND
```

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
                    │100k│  R8 (high-side)
                    └─┬─┘
                      │
                      ├──────► ESP32 GPIO0 (ADC)
                      │
                    ┌─┴─┐         ┌───┐
                    │15k │  R9    ─┤100n├─  C1 (filter)
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

> **Note:** The high-impedance divider (115kΩ total) draws only ~0.18mA from the panel — negligible. The 100nF cap prevents ADC jitter from PWM noise if a charge controller is connected.

### MOSFET Sensor Power Switch

```
    12V bus ────┐
                │
           Source┐
         ┌──────┤ P-FET (IRF9540N / AO3401)
         │  Gate├────────────────────┐
         │      │                    │
         │ Drain┘                  ┌─┴─┐
         │   │                     │100k│ (pull-up to 12V)
         │   │                     └─┬─┘
         │   └──► Sensor VCC         │
         │        (brown ×3)         │
         │                           │
         │                      Drain┐
         │               ┌──────────┤ N-FET (2N7000 / BSS138)
         │               │     Gate├────┐
         │               │          │   │
         │              GND    Source┘  ┌┴┐
         │                       │     │10k│ (pull-down)
         │                      GND    └┬┘
         │                              │
         │                             GND
         │
         └─── ESP32 GPIO3 ─────────────┘
                                   (to N-FET gate)

    GPIO3 HIGH → N-FET ON → pulls P-FET gate LOW → P-FET ON → 12V to sensors
    GPIO3 LOW  → N-FET OFF → P-FET gate pulled HIGH (100kΩ) → P-FET OFF → sensors unpowered
```

---

## ESPHome YAML Configuration

See [`rain-barrel.yaml`](rain-barrel.yaml) for the full ESPHome configuration.

Key features of the config:
- **Deep sleep**: Wakes every 10 minutes, stays awake for 30 seconds to connect to WiFi, read sensors, and report state
- **MOSFET sensor power**: GPIO3 powers on the XKC-Y25 sensors at boot, waits 800ms for stabilization, then reads
- **Single relay valve control**: GPIO4 energizes the relay to open the normally-closed valve; de-energizing closes it
- **Manual toggle button**: GPIO2 wakes the ESP32 from deep sleep and toggles the valve open/closed
- **LED indicator**: GPIO10 lights up while the valve is open
- **Sensor shutoff before sleep**: A script turns off sensor power before entering deep sleep

---

## Safety Notes

1. **Continuous current while open** — the normally-closed valve draws 6VA (~500mA at 12V) the entire time it's open, plus ~70mA for the relay coil. Factor this into your battery budget. If the valve will be open for extended periods, consider a latching valve instead.

2. **Flyback protection** — if using a bare relay without a module, place a 1N4007 diode reverse-biased across the relay coil.

3. **Power supply sizing** — the valve draws 6VA (500mA at 12V) continuously while open. The LiFePO4 battery handles this fine, but be mindful of total open time for battery life planning.

4. **Common ground** — all GND connections (buck converter, relay module, ESP32, battery) must share a common ground.

5. **Deep sleep and OTA** — while in deep sleep the ESP32 is unreachable. To flash OTA updates, either wait for a wake cycle or press the manual button to wake it. The 30s run duration gives enough time for OTA to begin (ESPHome will prevent sleep during an active OTA).

6. **Deep sleep and valve state** — if the ESP32 enters deep sleep while the valve is open, the relay GPIO will go LOW and the valve will close. The ESPHome config uses `deep_sleep.prevent` to keep the system awake while the valve is commanded open.

---

## Wiring Summary

| ESP32-C3 Pin | Connects To |
|--------------|-------------|
| GPIO0 | Solar voltage divider midpoint (100kΩ/15kΩ + 100nF cap) |
| GPIO2 | Manual TOGGLE button (+ 10kΩ pull-down) — also deep sleep wake |
| GPIO3 | MOSFET gate driver (N-FET gate → controls sensor power) |
| GPIO4 | Relay module IN (valve power) |
| GPIO6 | XKC-Y25 #1 signal (HIGH level) + 10kΩ pull-up to 3.3V |
| GPIO7 | XKC-Y25 #2 signal (MID level) + 10kΩ pull-up to 3.3V |
| GPIO8 | XKC-Y25 #3 signal (LOW level) + 10kΩ pull-up to 3.3V |
| GPIO10 | LED indicator (+ 470Ω resistor → LED → GND) |
| 5V (VIN) | Buck converter 5V output |
| 3V3 | Pull-up resistors, button VCC |
| GND | Common ground rail |

| 12V Battery | Connects To |
|-------------|-------------|
| 12V+ | Buck converter input +, Relay contacts (per diagram), P-FET source (sensor power) |
| GND | Buck converter input -, Relay module GND, XKC-Y25 GND (blue) ×3, Common ground |

## Power Budget (Estimated)

| State | Current Draw (12V) | Duration | Energy |
|-------|-------------------|----------|--------|
| Deep sleep | ~0.5 mA (ESP32 + buck quiescent) | ~9.5 min/cycle | ~0.6 mWh |
| Awake (WiFi + sensors) | ~120 mA | ~30s/cycle | ~6 mWh |
| Valve open | ~570 mA (500mA valve + 70mA relay coil) | varies | ~6.8 Wh/hour |

**Average draw (valve closed)**: ~5–8 mA → battery life without solar: 26–42 days on 5Ah LiFePO4.
**With valve open**: 570mA continuous → ~8.8 hours max on a full battery before depletion.
With a 20–30W solar panel, the system is self-sustaining for typical intermittent watering use.
