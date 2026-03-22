# Washer/Dryer Detection Accuracy History

## Goal
Improve washer and dryer run detection accuracy with two MPU6050 sensors while reducing cross-coupling false positives.

## Attempted Changes And Outcomes

### 1. Absolute acceleration thresholds only
- Change: Used fixed thresholds on combined acceleration for washer and dryer.
- Result: Did not work.
- Why: Idle baselines were high and drifting, so fixed low thresholds caused false positives.

### 2. Cross-machine suppression with simple checks
- Change: Added logic to suppress one machine when the other was considered active.
- Result: Mixed.
- Why: Helped some coupling cases, but early versions could create startup dependency issues and inconsistent behavior.

### 3. Adaptive acceleration baselines
- Change: Added runtime baselines and used vibration deltas instead of raw magnitudes.
- Result: Worked.
- Why: Converted detection from absolute values to relative movement, which reduced offset/drift sensitivity.

### 4. Baseline from filtered max-window signals
- Change: Baseline used combined acceleration values that had max/window filtering.
- Result: Did not work well.
- Why: Baseline response was delayed and could initialize poorly.

### 5. Baseline from raw combined acceleration
- Change: Added raw combined acceleration sensors and used those for baseline and delta calculations.
- Result: Worked.
- Why: Faster convergence and more stable delta behavior.

### 6. Independent baseline learning per machine
- Change: Washer baseline updates when washer is off; dryer baseline updates when dryer is off.
- Result: Worked.
- Why: Prevented one machine from blocking baseline updates for the other.

### 7. Hide per-axis accel/gyro from Home Assistant
- Change: Set axis sensors to internal only.
- Result: Worked.
- Why: Reduced HA entity noise while keeping internal math intact.

### 8. Initialization guards and NaN handling
- Change: Added has_state and NaN guards in template sensors and interval logic.
- Result: Worked.
- Why: Prevented unknown/invalid startup states from contaminating logic.

### 9. 20-minute minimum cycle latch
- Change: Added minimum on-time after strong start for both washer and dryer.
- Result: Mixed.
- Why: Reduced false off transitions during real cycles, but could hold ON after a false start.

### 10. Washer inter-phase hold for rinse gaps
- Change: Added weak-activity hold window to keep washer ON through short low-motion phases.
- Result: Worked for washer cycle continuity.
- Why: Avoided off transitions between rinse and spin phases.

### 11. Dryer compensation in washer accel trigger
- Change: Washer micro trigger uses compensated signal:
  - washer_micro_effective = max(0, washer_micro - dryer_micro * factor)
- Result: Worked.
- Why: Reduced washer false positives when dryer-induced vibration was present.

### 12. Named tuning constants for compensation/threshold
- Change: Replaced hardcoded numbers with named variables and persistent globals.
- Result: Worked.
- Why: Easier tuning and repeatability.

### 13. Debug tuning metrics in template sensors
- Change: Added debug sensors for subtraction, effective micro, and trigger margins.
- Result: Worked.
- Why: Made tuning decisions data-driven.

### 14. Runtime HA switch for debug updates
- Change: Attempted adding a switch to enable/disable debug metric updates.
- Result: Not retained in current file.
- Why: This attempt was reverted/undone, so current config still relies on compile-time visibility control.

### 15. Dryer detection using gyro-only path (earlier tuning)
- Change: Added gyro-assisted dryer start with permissive thresholds.
- Result: Did not work reliably.
- Why: Could trigger dryer ON when off due to transient gyro behavior.

### 16. Dryer start confirmation window and tighter gyro gate
- Change: Required sustained strong condition before latching dryer cycle; increased gyro trigger and added micro gate.
- Result: Current direction.
- Why: Intended to reduce false dryer starts while preserving true starts.

## Current Key Tunables
- dryer_coupling_compensation_factor
- washer_micro_effective_trigger
- washer_large_vibration_trigger
- Dryer start confirmation duration
- Dryer accel and gyro start thresholds

## Known Failure Patterns Observed
- Dryer OFF but detected ON due to permissive start/latch conditions.
- Dryer ON but not detected when thresholds were too strict for low-amplitude delta signals.
- Washer false ON from dryer coupling when washer gyro path had insufficient local-motion gating.

## Summary Of What Did Not Work
- Raw absolute thresholds without adaptive baseline.
- Baseline learning based on delayed max-window combined acceleration.
- Permissive dryer gyro-based starts without sufficient confirmation.
- Any latch strategy without robust start validation.
