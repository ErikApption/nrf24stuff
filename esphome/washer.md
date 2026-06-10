# ESP Home

## Python

```bash
python3 -m venv .venv
source .venv/bin/activate
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
- **Washer hold logic**: Once started, washer stays ON for at least 20 min, then holds for up to 5 min of inactivity between wash phases. Hard cap of 3 hours maximum cycle duration. A 2-min `delayed_off` filter adds final debounce. Weak-activity refresh requires at least 2 of 3 signals (compensated micro ≥ 0.12, large ≥ 0.35, gyro ≥ 0.80) to prevent idle noise from extending the cycle indefinitely. Weak-activity is completely suppressed when the dryer is confirmed ON, since coupled vibration easily fakes these signals.

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

# Washer/Dryer Detection Accuracy History

## Goal
Improve washer and dryer run detection accuracy with two MPU6050 sensors while reducing cross-coupling false positives.

## Key Metrics

- Washer baseline-relative acceleration (raw combined minus washer baseline)
- Dryer baseline-relative acceleration (raw combined minus dryer baseline)
- Washer compensated micro vibration
  - `washer_micro_effective = max(0, washer_micro - dryer_micro * compensation_factor)`
- Washer gyro magnitude (combined)
- Dryer gyro magnitude (combined)

## Major Detection Changes And Outcomes

### 1. Absolute thresholds on raw/combined acceleration
- Result: Did not work.
- Reason: Idle offsets and drift caused persistent false positives.

### 2. Adaptive baselines and delta metrics
- Result: Worked.
- Reason: Detection became relative to each sensor's live baseline instead of fixed absolute values.

### 3. Baseline source changed from filtered max-window to raw combined acceleration
- Result: Worked.
- Reason: Faster and cleaner baseline convergence.

### 4. Independent baseline learning per machine
- Result: Worked.
- Reason: One machine state no longer blocks baseline updates for the other.

### 5. Cross-coupling suppression between washer and dryer
- Result: Mixed to good.
- Reason: Reduced many cross-trigger events but needed threshold tuning to avoid edge cases.

### 6. Washer micro compensation using dryer micro
- Result: Worked.
- Reason: Reduced washer false positives during dryer operation.

### 7. Washer run-cycle hold logic (minimum cycle plus inter-phase hold)
- Result: Worked for cycle continuity.
- Reason: Prevented washer from dropping OFF during short rinse/transition low-motion periods.

### 8. Dryer run-cycle latch based on permissive start
- Result: Did not work reliably.
- Reason: Could create false ON when start criteria were too permissive.

### 9. Dryer start confirmation window and tighter start gating
- Result: Current direction.
- Reason: Requires sustained evidence before latching, reducing false starts.

### 10. Washer hold refresh changed to washer-specific activity
- Result: Worked (fix for washer staying ON too long).
- Reason: Previous weak-activity refresh used permissive raw motion signals, so background/dryer-coupled vibration could continuously extend washer ON time. Refresh now prefers compensated washer activity and blocks refresh while dryer clearly dominates.

### 11. Washer baseline freeze on start (fix for washer not detected when started)
- Result: Worked.
- Reason: The baseline EMA (alpha=0.03, ~33s time constant) was adapting toward elevated values while `washer_on_sensor` was still false. This caused `wm_effective` and `wl` to decay back below threshold before the `delayed_on: 30s` on the accel/gyro flags could complete, preventing detection entirely. Fix freezes baseline learning whenever `w − baseline ≥ 0.06 m/s²`, which is below the detection thresholds but above dryer-coupling noise, so the effective signal stays stable long enough for detection to confirm.

## Key Tunables

- `dryer_coupling_compensation_factor`
- `washer_micro_effective_trigger`
- `washer_large_vibration_trigger`
- Dryer accel triggers (large and micro)
- Dryer gyro trigger
- Dryer start confirmation duration
- Washer gyro trigger and washer effective micro gate (gyro path)
- Washer weak-activity refresh thresholds (`wm_effective`, `wl`, `wg`)
- Washer dryer-dominance gate for hold refresh

### 12. Washer not detected on start (missed ON) — resolved
- Result: Worked.
- Reason: Hardcoded accel offsets were wrong, causing resting magnitude of ~3.7 m/s² instead of ~9.81. Replaced with auto-calibration system that computes offsets from 30 samples at rest.

### 13. Washer never turns OFF (stays ON forever after cycle completes) — resolved
- Result: Worked.
- Reason: The `weak_activity` refresh condition was too permissive — any single signal above a low threshold (wm_effective ≥ 0.08 OR wl ≥ 0.24 OR wg ≥ 0.50) could refresh the 8-minute hold timer. Normal idle sensor noise with the max-window filters was enough to keep triggering it. Fix requires at least 2 of 3 signals above raised thresholds (wm_effective ≥ 0.12, wl ≥ 0.35, wg ≥ 0.80) and adds a 3-hour hard cap on maximum cycle duration.

## What Clearly Did Not Work

- Fixed absolute acceleration thresholds without adaptive baseline.
- Baseline learning from delayed max-window acceleration values.
- Dryer latch with permissive start criteria and no start-confirmation window.
- Washer hold refresh based on permissive raw weak-activity thresholds.
- Washer baseline learning while raw signal is already elevated at startup (races against delayed_on timer).
- Washer weak-activity refresh using OR logic with low thresholds (any single noisy signal refreshes hold indefinitely).
