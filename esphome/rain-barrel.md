# ESP32-C3 Motorized Ball Valve Control

> **Note:** The ESP32-C4 does not exist as a product. The closest small/low-pin-count options are the **ESP32-C3** (RISC-V, single core, WiFi + BLE) or the **ESP32-C6** (adds Thread/Zigbee). This document uses the ESP32-C3 as it's the most common. Swap for C6 if you need Thread support.

## Overview

Control a 12V motorized ball valve using an ESP32-C3 dev board and a 2-channel relay module, powered by a 12V LiFePO4 battery with solar charging. The system uses deep sleep to minimize power draw, waking periodically to read water level sensors and report status.

The valve has two wires — applying 12V in one polarity opens it, reversing polarity closes it. A DPDT (double-pole double-throw) relay pair handles the polarity reversal. A MOSFET high-side switch powers the XKC-Y25 sensors only during the wake cycle.

---

## Parts List

| Component | Qty | Example | Notes |
|-----------|-----|---------|-------|
| ESP32-C3 dev board | 1 | ESP32-C3 SuperMini, Seeed XIAO ESP32-C3 | 3.3V logic |
| 2-channel relay module (5V coil, 3.3V logic compatible) | 1 | HW-307, SRD-05VDC | Must trigger from 3.3V HIGH or have optoisolated inputs |
| 12V motorized ball valve | 1 | CR02/CR04 style, 2-wire | Typically draws 0.5–1.5A stall |
| XKC-Y25-NPN non-contact liquid level sensor | 3 | XKC-Y25-NPN (5–12V version) | NPN open-collector output, needs pull-up to 3.3V |
| 12V LiFePO4 battery | 1 | 4S 12.8V 5Ah | ~64 Wh capacity |
| Solar charge controller | 1 | Victron 75/10, EPever 10A | Must support LiFePO4 profile |
| Solar panel | 1 | 20–30W 18V monocrystalline | Sized for ~5–8 Wh/day load |
| Buck converter 12V → 5V | 1 | Mini-360, MP1584 module | Powers ESP32 dev board via 5V/VIN pin |
| P-channel MOSFET (high-side switch) | 1 | IRF9540N, AO3401 (SOT-23) | Switches 12V to sensors |
| N-channel MOSFET (gate driver) | 1 | 2N7000, BSS138 | Level-shifts 3.3V GPIO to drive P-FET gate |
| Gate pull-up resistor | 1 | 100kΩ | Holds P-FET OFF when GPIO is low |
| Gate pull-down resistor | 1 | 10kΩ | Pulls N-FET gate to GND when ESP32 is asleep |
| Flyback diodes (optional) | 2 | 1N4007 | Across relay coils if using bare relays |
| Pull-up resistors | 3 | 10kΩ | Pull NPN sensor output up to 3.3V for ESP32 GPIO |
| Momentary push button (TOGGLE) | 1 | Normally-open, panel mount | Manual valve open/close toggle |
| LED indicator | 1 | 3mm or 5mm, any color (green/red) | Indicates valve is open |
| LED current-limiting resistor | 1 | 470Ω (for 3.3V) or 1kΩ | Limits LED current to ~5mA |
| Button pull-down resistor | 1 | 10kΩ | Hold GPIO LOW when button not pressed |
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
              ┌────────────────────┼───────────────┐
              │                    │               │
         ┌────┴────┐         ┌────┴────┐     ┌────┴────┐
         │ Buck 5V │         │  Relay  │     │  Relay  │
         │ Conv.   │         │  CH1    │     │  CH2    │
         └────┬────┘         └────┬────┘     └────┬────┘
              │                   │               │
              │              Coil powered         │
              │              from 5V rail         │
              │                   │               │
              ▼                   ▼               ▼
        ┌──────────┐        Relay Contacts   Relay Contacts
        │ ESP32-C3 │
        │          │
        │  GPIO4 ──────► Relay CH1 IN (open valve)
        │  GPIO5 ──────► Relay CH2 IN (close valve)
        │          │
        │  GPIO3 ──────► MOSFET sensor power switch (see below)
        │          │
        │  GPIO6 ◄────── XKC-Y25 #1 Signal (HIGH level)
        │  GPIO7 ◄────── XKC-Y25 #2 Signal (MID level)
        │  GPIO8 ◄────── XKC-Y25 #3 Signal (LOW level)
        │          │
        │  GPIO2 ◄────── Manual TOGGLE button (+ 10kΩ pull-down)
        │  GPIO10──────► LED indicator (valve open)
        │          │
        │  3V3  ───────► Pull-up resistors (3× 10kΩ), button VCC
        │  GND  ───────► Relay GND
        │  5V/VIN ◄───── Buck converter 5V out
        │  GND  ───────── Common GND
        └──────────┘
```

### Relay Contact Wiring (H-Bridge via 2 SPDT Relays)

```
    12V+ ────┬──────────────────────┬──── 12V+
             │                      │
        ┌────┴────┐           ┌────┴────┐
        │  Relay1 │           │  Relay2 │
        │  COM    │           │  COM    │
        ├─────────┤           ├─────────┤
        │ NC ─┐   │           │ NC ─┐   │
        │ NO ─┼─┐ │           │ NO ─┼─┐ │
        └─────┘ │ │           └─────┘ │ │
                │ │                   │ │
    GND ────┬───┘ │                   │ └───┬──── GND
            │     │                   │     │
            │     └───── VALVE + ─────┘     │
            │                               │
            └──────────── VALVE - ──────────┘

    State table:
    ┌────────┬────────┬─────────────────────────┐
    │ Relay1 │ Relay2 │ Result                  │
    ├────────┼────────┼─────────────────────────┤
    │  OFF   │  OFF   │ No power (valve holds)  │
    │  ON    │  OFF   │ +12V across valve (OPEN)│
    │  OFF   │  ON    │ -12V across valve (CLOSE│
    │  ON    │  ON    │ SHORT — never do this!  │
    └────────┴────────┴─────────────────────────┘
```

### Detailed Relay Wiring

Each SPDT relay has 3 contacts: **COM** (common), **NC** (normally closed), **NO** (normally open).

**Relay 1:**
- COM → 12V+
- NC → Valve wire A
- NO → GND

**Relay 2:**
- COM → 12V+  
- NC → Valve wire B
- NO → GND

**Valve wiring:**
- Valve wire A connects to: Relay1 NC *and* Relay2 NO
- Valve wire B connects to: Relay2 NC *and* Relay1 NO

Wait — let me simplify. The cleanest wiring:

**Relay 1:**
- COM → Valve wire A

- NC → GND
- NO → 12V+

**Relay 2:**
- COM → Valve wire B
- NC → 12V+
- NO → GND

| Relay1 | Relay2 | Wire A | Wire B | Motor Direction |
|--------|--------|--------|--------|-----------------|
| OFF (NC) | OFF (NC) | GND | 12V+ | CLOSING |
| ON (NO) | ON (NO) | 12V+ | GND | OPENING |
| OFF | ON | GND | GND | STOPPED |
| ON | OFF | 12V+ | 12V+ | STOPPED |

> **To avoid running the motor continuously**, pulse the relays for the valve's rated travel time (typically 5–10 seconds for a ½″–¾″ valve), then return both relays to a stopped state.

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
- **Manual buttons**: GPIO2 (open) and GPIO10 (close) act as wake-up pins — pressing either button wakes the ESP32 from deep sleep and actuates the valve
- **Interlock via time_based cover**: The cover component ensures only one relay direction is active at a time
- **Sensor shutoff before sleep**: A script turns off sensor power and relays before entering deep sleep

---

## Safety Notes

1. **Never energize both relays simultaneously** — this shorts 12V to GND through the motor. The ESPHome `time_based` cover handles this correctly. The YAML also uses `deep_sleep.prevent` during valve actuation to avoid sleeping mid-operation.

2. **Adjust `open_duration` / `close_duration`** to match your specific valve. Over-driving a valve at its end stop for extended periods can damage the motor or strip gears.

3. **Flyback protection** — if using bare relays without a module, place a 1N4007 diode reverse-biased across each relay coil.

4. **Power supply sizing** — motorized valves can draw 1–1.5A at stall. The LiFePO4 battery can easily supply this.

5. **Common ground** — all GND connections (buck converter, relay module, ESP32, battery) must share a common ground.

6. **Deep sleep and OTA** — while in deep sleep the ESP32 is unreachable. To flash OTA updates, either wait for a wake cycle or press a manual button to wake it. The 30s run duration gives enough time for OTA to begin (ESPHome will prevent sleep during an active OTA).

---

## Wiring Summary

| ESP32-C3 Pin | Connects To |
|--------------|-------------|
| GPIO2 | Manual TOGGLE button (+ 10kΩ pull-down) — also deep sleep wake |
| GPIO3 | MOSFET gate driver (N-FET gate → controls sensor power) |
| GPIO4 | Relay module CH1 IN (open valve) |
| GPIO5 | Relay module CH2 IN (close valve) |
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
| Valve actuation | ~1.5 A (peak) | ~8s (infrequent) | ~40 mWh |

**Average draw**: ~5–8 mA → battery life without solar: 26–42 days on 5Ah LiFePO4.
With a 20–30W solar panel, the system is easily self-sustaining year-round.
