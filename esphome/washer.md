# ESP Home

## Python

```bash
python3 -m venv .venv
source venv/bin/activate
pip3 install esphome
```

```powershell
python -m venv .venv
. .\venv\Scripts\activate
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

See [Detection history](./DETECTION_ACCURACY_HISTORY.md) for more details.

### Detection paths

The washer has three independent paths to trigger ON (any one is sufficient):

1. **Baseline-relative accel path** (in `Accel Flag 1`):
   - Condition: `washer_large_vibrations >= washer_large_vibration_trigger` AND `washer_micro_effective >= washer_micro_effective_trigger`
2. **Jitter path** (in `Accel Flag 1`):
   - Condition: `washer_accel_jitter >= 0.07`
   - Immune to baseline drift; detects sample-to-sample variation.
3. **Gyro path** (in `Gyro Flag 1`):
   - Condition: `washer_combined_gyro >= 1.20` AND `washer_micro_effective >= 0.06`

All flags use `delayed_on: 30s` — the condition must hold continuously for 30 seconds before the flag turns ON.

### Tunable globals (in `washer.yaml`)

| Global | Current value | Role |
|--------|--------------|------|
| `dryer_coupling_compensation_factor` | `0.75` | How much dryer micro-vibration is subtracted from washer micro. Formula: `wm_effective = max(0, wm - dm * factor)` |
| `washer_micro_effective_trigger` | `0.10` | Minimum compensated washer micro-vibration for the baseline-relative accel path. |
| `washer_large_vibration_trigger` | `0.30` | Minimum washer large vibration (10s max window) for the baseline-relative accel path. |

### Where these live in code

- File: `esphome/washer.yaml`
- Globals block: `dryer_coupling_compensation_factor`, `washer_micro_effective_trigger`, `washer_large_vibration_trigger`
- Sensor block: `binary_sensor` → `Accel Flag 1` (`id: washer_accel_flag_1`)
- Sensor block: `binary_sensor` → `Gyro Flag 1` (`id: washer_gyro_flag_1`)

### Additional detection safeguards

- **Baseline freeze**: Washer baseline stops learning when `w - baseline >= 0.06 m/s²`, preventing the baseline from chasing the signal upward and eroding the effective delta before the 30s delayed_on completes.
- **Dryer dominance suppression**: Both accel and gyro flags suppress when `dl >= 0.9 && dm >= 0.5 && dl > wl * 1.2` (dryer clearly dominates).
- **Washer hold logic**: Once started, washer stays ON for at least 20 min, then holds for up to 8 min of inactivity between wash phases. Hard cap of 3 hours maximum cycle duration. Weak-activity refresh requires at least 2 of 3 signals (compensated micro ≥ 0.12, large ≥ 0.35, gyro ≥ 0.80) to prevent idle noise from extending the cycle indefinitely.

### Quick tuning method

1. Run dryer only (washer off) and observe false washer triggers.
2. If washer still false-triggers, increase `dryer_coupling_compensation_factor` by `0.05`.
   - Typical range: `0.60` to `0.90`.
3. Run washer only and check missed starts.
4. If washer misses real activity, reduce `washer_large_vibration_trigger` by `0.05` or lower `washer_micro_effective_trigger` by `0.02`.
   - `washer_large_vibration_trigger` typical range: `0.15` to `0.45`.
   - `washer_micro_effective_trigger` typical range: `0.05` to `0.20`.
5. If washer still false-triggers with dryer only, raise `washer_micro_effective_trigger` by `0.02`.

### Troubleshooting: washer not detected when it turns on

If the washer is running but not being detected:

1. **Run calibration first** — press the "Calibrate Accelerometers" button in HA (under the washer device config section). Both machines must be OFF and still. The ESP collects 30 samples over 30 seconds and computes offsets so the resting combined acceleration = 9.81 m/s² (gravity). Offsets persist across reboots.
2. **Enable debug metrics** — set `debug_metrics_internal: "false"` in the substitutions block to expose all debug sensors to Home Assistant.
3. **Check `Debug Washer Large Trigger Margin`** — if consistently negative while the washer runs, `washer_large_vibration_trigger` is too high. Lower it by `0.05`.
4. **Check `Debug Washer Micro Trigger Margin`** — if consistently negative, `washer_micro_effective_trigger` is too high or compensation is over-subtracting. Lower the trigger or reduce `dryer_coupling_compensation_factor`.
5. **Check `Debug Washer Accel Jitter`** — if this stays below `0.07` during washer operation, the jitter path won't fire either. This suggests the washer vibration is steady-state (elevated but not varying sample-to-sample).
6. **Check baseline behavior** — if `Washer Accel Baseline` is rising toward the running value, the baseline freeze threshold (`0.06`) may be too high for your washer's vibration signature. This would need a code change to lower the freeze delta.
7. **Check the `delayed_on: 30s`** — the washer must sustain the trigger condition for a full 30 seconds. If vibration is intermittent at startup, the timer resets each time the condition drops. Consider lowering `delayed_on` to `20s` if this is the pattern.

### Accelerometer calibration

The MPU6050 sensors have per-unit offsets that drift over time and temperature. Instead of hardcoded offsets in the YAML, the system uses auto-calibration:

- **How it works**: Press "Calibrate Accelerometers" in HA. The ESP averages 30 raw readings per axis, then computes per-axis offsets that scale the resting vector magnitude to exactly 9.81 m/s².
- **When to calibrate**: After flashing, after physically moving the sensor, or if the resting baseline drifts far from 9.81.
- **Where offsets are stored**: In ESP32 flash (`restore_value: yes` globals). They survive reboots. The offset values are also exposed as HA diagnostic sensors (`Washer Offset X/Y/Z`, `Dryer Offset X/Y/Z`).
- **Clear offsets**: Press "Clear Accel Offsets" to reset all offsets to zero (useful before re-calibrating if values look wrong).

### Practical guidance

- Lower `washer_large_vibration_trigger` first when the washer is not being detected (most common fix for missed starts).
- Increase compensation first when dryer coupling is the main false-positive problem.
- Increase triggers when general noise is the main false-positive problem.
- Change one parameter at a time and test at least one full washer cycle.
- The jitter path (`0.07`) is a safety net — if both the baseline and jitter paths miss, the washer vibration profile may have changed (e.g., different load type, leveling shifted).

