# ESP Home

## Python

```bash
python3 -m venv venv
source venv/bin/activate
pip3 install esphome
```

```powershell
python -m venv venv
. .\venv\Scripts\activat
```

## Enter Flash Mode

- Hold the BOOT button (this pulls GPIO0 to GND).
- Press and release the EN/RST button (to reset the ESP32).
- Release the BOOT button after resetting.

The ESP32 is now in flash mode and ready for firmware uploads.

## Flash image

```bash
esphome run furnace.yaml
```

## Generate Binary Image

To generate a binary image for manual upload:

```bash
esphome compile washer.yaml
```

The binary file will be saved in the `.esphome/build/<washer>/.pioenvs/<washer>` directory.

## Washer/Dryer Vibration Tuning

The washer accel trigger in `washer.yaml` uses two tuned factors to reduce false positives from dryer-coupled vibration:

- `DRYER_COUPLING_COMPENSATION_FACTOR` = `0.75`
	- Formula: `washer_micro_effective = max(0, washer_micro - dryer_micro * factor)`
	- Meaning: how much dryer micro-vibration is subtracted from washer micro-vibration.

- `WASHER_MICRO_EFFECTIVE_TRIGGER` = `0.30`
	- Condition: washer accel flag can turn on when `washer_large_vibrations >= 0.8` and `washer_micro_effective >= trigger`.
	- Meaning: the minimum compensated washer micro-vibration needed to treat washer movement as real.

### Where these live in code

- File: `esphome/washer.yaml`
- Sensor block: `binary_sensor` -> `Accel Flag 1` (`id: washer_accel_flag_1`)

### Quick tuning method

1. Run dryer only (washer off) and observe false washer triggers.
2. If washer still false-triggers, increase `DRYER_COUPLING_COMPENSATION_FACTOR` by `0.05`.
	 - Typical range: `0.60` to `0.90`.
3. Run washer only and check missed starts.
4. If washer misses real activity, reduce compensation by `0.05` or lower `WASHER_MICRO_EFFECTIVE_TRIGGER` by `0.03`.
5. If washer still false-triggers with dryer only, raise `WASHER_MICRO_EFFECTIVE_TRIGGER` by `0.03`.
	 - Typical range: `0.20` to `0.45`.

### Practical guidance

- Increase compensation first when dryer coupling is the main problem.
- Increase trigger first when general noise is the main problem.
- Change one parameter at a time and test at least one full washer cycle.

