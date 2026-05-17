# Hot Tub Temperature Monitor

Battery-powered ESP32-C3 with a DS18B20 temperature probe. Wakes every 5 minutes, takes 5 averaged readings, sends to Home Assistant, then deep sleeps.

## Parts

- ESP32-C3 DevKitM-1
- DS18B20 waterproof temperature probe
- 4.7kΩ resistor (pull-up for 1-Wire bus)
- 2× 100kΩ resistors (voltage divider for battery monitoring)
- Single-cell LiPo battery (3.7V nominal, 4.2V max)

## Wiring

### DS18B20 Temperature Probe → GPIO4

```
         ESP32-C3
        ┌─────────┐
        │         │
 3.3V ──┤ 3V3     │
        │         │
        │    GPIO4├───┬─── DS18B20 DATA (yellow)
        │         │   │
        │         │  [4.7kΩ]
        │         │   │
 3.3V ──┤ 3V3 ───┘───┘
        │         │
  GND ──┤ GND ────────── DS18B20 GND (black)
        │         │
 3.3V ──┤ 3V3 ────────── DS18B20 VCC (red)
        │         │
        └─────────┘
```

| DS18B20 Wire | Connects To |
|--------------|-------------|
| Red (VCC)    | 3.3V        |
| Black (GND)  | GND         |
| Yellow (DATA)| GPIO4 + 4.7kΩ pull-up to 3.3V |

The 4.7kΩ pull-up resistor goes between the DATA line and 3.3V. This is required for the 1-Wire protocol.

### Battery Voltage Divider → GPIO2

```
  Battery +  ──── [100kΩ] ───┬─── [100kΩ] ──── GND
                              │
                              └─── GPIO2 (ADC)
```

| Connection       | Connects To       |
|------------------|-------------------|
| Battery +        | Top of R1 (100kΩ) |
| R1/R2 midpoint   | GPIO2             |
| Bottom of R2     | GND               |

This divides the battery voltage by 2, bringing the 3.0–4.2V LiPo range into the ESP32 ADC-safe range (0–1.65V at the pin). The `multiply: 2.0` filter in the YAML compensates to report the true battery voltage.

### Power

```
  Battery + ──── ESP32-C3 VIN (or 5V pin via regulator, depending on board)
  Battery - ──── ESP32-C3 GND
```

Check your specific C3 devkit — some accept battery voltage directly on a `BAT` or `VIN` pin, others need it on the 5V pin (which feeds through the onboard regulator). A single-cell LiPo (3.0–4.2V) can typically power the 3.3V regulator directly.

## Full Pinout Summary

| GPIO | Function              |
|------|-----------------------|
| GPIO4| DS18B20 1-Wire data   |
| GPIO2| Battery ADC (divider) |

## Notes

- **Voltage divider values**: If your battery exceeds 4.2V (e.g., 2S LiPo at 8.4V), adjust the resistor ratio and the `multiply` filter accordingly. Formula: `multiply = (R1 + R2) / R2`.
- **Deep sleep current**: The ESP32-C3 draws ~5µA in deep sleep. Main power drain is the ~12–15 second wake window every 5 minutes.
- **DS18B20 address**: If you have multiple sensors on the bus, specify the address in the YAML. Run once without an address to discover it in the logs.
- **Waterproofing**: Use the stainless steel waterproof DS18B20 variant for submersion. Seal the cable entry point with heat shrink or silicone.

## Flashing

```bash
cd esphome
esphome run hottub.yaml
```

See [washer.md](./washer.md) for flash mode instructions (hold BOOT, press RST, release BOOT).
