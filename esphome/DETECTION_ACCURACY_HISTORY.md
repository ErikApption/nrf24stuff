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

## Key Tunables

- `dryer_coupling_compensation_factor`
- `washer_micro_effective_trigger`
- `washer_large_vibration_trigger`
- Dryer accel triggers (large and micro)
- Dryer gyro trigger
- Dryer start confirmation duration
- Washer gyro trigger and washer effective micro gate (gyro path)

## What Clearly Did Not Work

- Fixed absolute acceleration thresholds without adaptive baseline.
- Baseline learning from delayed max-window acceleration values.
- Dryer latch with permissive start criteria and no start-confirmation window.
