# Optm-WePO — Optimization & Weak-Point Overview

> Scope: control, flow, applied physics/math, and software architecture recommendations for
> `Rebuilt2026`. These are **design/control recommendations**, not cosmetic refactors. Each item
> describes the issue, why it matters competitively, and a concrete suggested solution.
> File references are `path:line` and clickable.
>
> Benchmarked against common practice from top shooter/aiming teams (254, 1678, 6328, 3015, 1690)
> for shoot-on-the-move, vision fusion, and turret tracking.

---

## Part A — Substantial Weak Points (high-level)

These are correctness/architecture problems where the *intent* in the code is sound but the
implementation silently does nothing, defeats itself, or contradicts an assumption elsewhere. These
are ranked by competitive impact.

### A1. Shoot-on-the-move is written but **disabled** — time-of-flight map is empty
`Shooter.java:243-255` — `initDistanceTimeOfFlightMap()` only inserts `(0.0, 0.0)`; every other entry
is commented out. The moving-shot lead compensation in `getTurretToTargetVector()`
(`Shooter.java:358-367`) multiplies the chassis velocity by `timeOfFlightMap.get(distance)`. With an
empty map, `InterpolatingDoubleTreeMap` returns `0.0` for every distance, so
`virtualTargetOffset` is always the zero vector and **no motion compensation is ever applied**. The
entire shoot-while-driving feature is dead despite the math being present.
→ See **B1**.

### A2. HIGH/LOW target selection is a **no-op** — wrong calibration for low goal
`Shooter.java:490-500` — all three selectors (`getInterpolatingTimeOfFlightMap`,
`getInterpolatingVelocityMap`, `getInterpolatingAngleMap`) use the ternary
`x.equals(HIGH) ? highMap : highMap` — both branches return the HIGH map. The fully-populated
`lowAngleDistanceMap` / `lowVelocityDistanceMap` / `lowDistanceTimeOfFlightMap`
(`Shooter.java:274-304`) are **never read**. Any LOW-goal/delivery shot uses HIGH-goal calibration.
→ See **B2**.

### A3. The hood angle map for HIGH is empty → hood never aims
`Shooter.java:257-259` — `highAngleDistanceMap` contains only `(0.0, 0.0)`. Since A2 forces every
shot through the HIGH map, the hood setpoint interpolates to `0.0` at all distances. The flywheel
velocity map *is* populated, so the robot spins up correctly but **does not adjust hood angle by
distance** — a single fixed hood angle for all ranges. This is the largest accuracy gap.
→ See **B2 / B6**.

### A4. `TurretOffsetGetter.setTurretRotationalVelSup` has a copy-paste bug
`TurretOffsetGetter.java:30-32` — the setter assigns
`this.turretOffsetSupplier = turretOffsetSupplier;` (self-assignment of the wrong field) instead of
`this.turretRotationalVelSup = turretRotationalVelSup;`. Consequences:
- The turret's rotational velocity is never registered, so `isFast()`
  (`TurretOffsetGetter.java:24-26`) only ever sees robot rotational speed.
- `isFast()` gates whether vision measurements are accepted (`Swerve.java:505`). A fast-spinning
  turret will **not** suppress vision even though the turret-mounted Limelight is motion-blurred —
  exactly when single-tag pose noise is worst.
- The intended `turretMechanism::logVelocity` wiring at `Shooter.java:237` is silently lost.
→ Trivial fix; high impact on localization quality.

### A5. Vision fusion has **no trust model** and the validator is dead code
`Swerve.java:504-506` calls `m_odometry.addVisionMeasurement(pose, ts)` with:
- No `setVisionMeasurementStdDevs(...)` anywhere — every vision frame is trusted with the WPILib
  default std-devs regardless of tag count, tag distance, or robot speed.
- Acceptance gated only on `tagCount >= 1`. Single-tag MegaTag poses are the dominant source of
  field-localization error and yaw drift.
- `VisionMeasurementValidator.java` (continuity / field-bounds / default-pose checks) exists in full
  but is **never called**.
- `PoseEstimator.java` is a second, unused duplicate of `Odometry.java`.
→ See **B3**.

### A6. Turret motion profile and feedforward are **declared but ignored**
`Shooter.java:133-140` constructs the `Turret` with
`new TrapezoidProfile.Constraints(Math.PI*2, Math.PI*100)`, but the `Turret` constructor that takes
constraints (`Turret.java:55-66`) **stores nothing** — `setPosition()` (`Turret.java:81-92`) runs a
plain position PID plus a static `ks·sign(pid)` term. So:
- There is no motion profiling (the constraints object is discarded).
- There is no velocity feedforward to track a *moving* bearing. When the robot drives/rotates, the
  target's turret-relative bearing slews continuously; a pure position PID always lags it.
- `enableContinuousInput` is commented out (`Turret.java:50, 63`); wrap-around is handled externally
  by `ContinuousSoftLimit`, which is workable but means the PID can command the long way around at
  the seams.
→ See **B4** — this is the single biggest "aim while moving" win after A1.

### A7. Hood feedback source is inconsistent with its readiness check
`Shooter.java:417-423` — the hood PID feeds back on `hoodMotor.getMotorPosition()` (the Talon's
internal rotor-derived position, seeded once at boot from the CANcoder at `Shooter.java:166`), while
the "hood adjusted" readiness trigger (`Shooter.java:200`) compares the **absolute encoder**
`hoodAngleSupplier`. If the internal position drifts from the absolute encoder (backlash, slip,
conversion-factor error), the PID can settle while the readiness trigger never fires — or vice
versa. The control loop and its "ready" gate disagree on where the hood is.
→ See **B6**.

### A8. The zone/state machine is almost entirely stubbed out
`Superstructure.java:69-98` — `inAllianceZone = () -> true`, `inIntermediateZone = () -> false`,
`shouldDeliver = () -> false`. As a result ~10 of the 12 state-transition triggers in
`initTriggers()` (`Superstructure.java:105-175`) can never fire (every neutral/intermediate/delivery
branch is unreachable). This is a large, intricate, untested decision tree shipping in a permanently
dead configuration.
→ See **B8** — either re-enable with real geometry or delete the dead branches so the live behavior
is auditable.

### A9. Out-of-range shots silently clamp instead of being rejected
`shooterReady` (`Shooter.java:229-233`) has its distance gate commented out
(`.and(() -> ...getNorm() < 3.5)`). The velocity map tops out at 3.0 m (`Shooter.java:270`).
`InterpolatingDoubleTreeMap` clamps to the nearest endpoint outside its range, so at 6 m the robot
reports `shooterReady == true` while flat-lining flywheel speed at the 3 m value. The driver gets a
green "locked on" with no valid ballistic solution.
→ See **B6**.

---

## Part B — Recommended Optimizations (with solutions)

### Category 1 — Shooting on the Move (physics + control)

#### B1. Make the shoot-on-the-move lead actually run, and iterate the time-of-flight
**Problem:** the lead model (`Shooter.java:341-369`) is first-order and currently inert (A1). Even
once the ToF map is filled, a single ToF lookup is evaluated at the *static* distance, but leading
the shot changes the effective range, which changes ToF — a coupling the current one-shot lookup
ignores.

**Solution:**
1. Populate `highDistanceTimeOfFlightMap`/`lowDistanceTimeOfFlightMap` from the commented empirical
   data (it's right there at `Shooter.java:245-254`) or from a ballistic model (B5).
2. Replace the single evaluation with a **fixed-point iteration** (2–3 passes is plenty at 20 ms):

   ```
   Translation2d aim = staticTurretToTarget;          // robot-frame vector to hub
   for (int i = 0; i < 3; i++) {
       double tof = tofMap.get(aim.getNorm());
       Translation2d lead = robotVelInTurretFrame.times(tof); // include omega lever arm
       aim = staticTurretToTarget.minus(lead);
   }
   ```
   This converges the (range ↔ flight-time ↔ lead) loop instead of guessing once.
3. Cache the result once per loop (see B7) so every consumer sees the *same* solved aim point.

#### B2. Compensate exit velocity (and hood) for the **radial** component of robot motion
**Problem:** the lead model corrects *aim direction* for chassis velocity, but the flywheel velocity
and hood angle are still looked up purely by range (`Shooter.java:391-414`). When the robot drives
toward/away from the hub, the ball inherits the chassis's radial velocity; exit speed needed from
the flywheel changes accordingly. Teams that shoot reliably on the move correct both the *tangential*
(aim) **and** *radial* (speed) components.

**Solution:**
- Decompose robot velocity into radial (toward target) and tangential components in the turret
  frame. Feed `flywheelSetpoint = map.get(range) − k · v_radial` (sign per geometry), and bias the
  hood map similarly. `k` is a small empirically tuned gain; even a linear correction removes most of
  the systematic miss when driving through a shot.
- First, fix the HIGH/LOW selector (A2) and fill `highAngleDistanceMap` (A3) so the *base* lookup is
  correct before layering motion correction on top.

### Category 2 — Localization & Vision Fusion

#### B3. Add a distance/tag-count trust model and re-enable the validator
**Problem:** all vision frames are fused with default trust and only a `tagCount >= 1` gate
(A5).

**Solution (in `Swerve.periodic`, `Swerve.java:498-508`):**
- Require `tagCount >= 2` for full trust; for single-tag frames either reject, or inflate std-devs
  heavily and reject if `ta` (target area) is small / `avgTagDist` is large.
- Scale std-devs with distance, e.g. `xyStdDev = base * avgTagDist² / tagCount`, then
  `m_odometry.setVisionMeasurementStdDevs(...)` *before* each `addVisionMeasurement`. This is the
  single highest-leverage localization change — it lets MegaTag2 anchor heading without single-tag
  jitter yanking the pose.
- Route every measurement through `VisionMeasurementValidator.isValidVisionMeasurement(...)` and
  `isWithinFieldBounds(...)` (already implemented, just unused) to drop teleport/NaN frames.
- For MegaTag2 specifically, push robot yaw into Limelight (`SetRobotOrientation`) and trust only
  translation — standard practice to kill single-tag yaw ambiguity.

#### B4. Give the turret a velocity feedforward so it tracks a moving bearing
**Problem:** turret is a pure position PID; the passed motion-profile constraints are discarded
(A6). Against a target whose bearing moves (robot translating/rotating), a position-only loop has
steady-state lag proportional to bearing rate.

**Solution:**
- In `Turret.setPosition` (`Turret.java:81-92`), add a velocity feedforward equal to the
  **negative robot angular velocity** plus the **target-bearing rate** from the swerve motion. To
  hold a field-fixed heading the turret must rotate at `−ω_robot`; that term alone removes most lag
  during spins. Use `kv` from the existing `Gains` (it's currently 0 for turret —
  `ShooterConstants.java:47` — run a SysId pass; the routine already exists at
  `SwerveModule.java:231-237`).
- Either use the `TrapezoidProfile.Constraints` that are already plumbed in (store and apply them, or
  switch to `ProfiledPIDController` — already imported at `Turret.java:4`), or remove the dead
  constraints argument so the API doesn't imply profiling that isn't happening.
- Reconsider `enableContinuousInput` (`Turret.java:50`): with continuous input the PID picks the
  short way around natively; the external `ContinuousSoftLimit` then only needs to enforce the cable
  wrap limits, not direction selection.

### Category 3 — Shooter Physics & Calibration

#### B5. Replace stacked empirical fudge-factors with a derived model + one calibration term
**Problem:** conversion factors are layered correction constants that hide the real geometry, e.g.:
- `Shooter.java:165`: `POSITION_CONVERSION_FACTOR * (-0.208/1.497) * 1.0231`
- `ShooterConstants.java:50-51`: `-0.2 * 2π * 0.9823` and `0.102434 * 2π / 5 * 0.9875 * 0.9919`

Each trailing `0.98xx` is a patch on a wrong base ratio. These are unmaintainable and will silently
drift when a gear or belt changes.

**Solution:** compute each conversion factor from the *actual* gear/pulley ratio as a named constant,
and keep **at most one** empirical scale factor with a comment explaining the calibration that
produced it. This makes the math auditable and survives mechanical changes.

#### B6. Gravity-aware hood feedforward + consistent feedback + range gating
**Problem:** the hood uses a PID plus a crude `±` static term (`Shooter.java:417-423`) with no
gravity compensation, feeds back on a different sensor than its readiness check (A7), and accepts
out-of-range setpoints (A9). Note the intake arm already does gravity FF correctly via `Mass`
(`Intake.java:75`) — the pattern exists in the codebase.

**Solution:**
- Feed the hood PID from the **absolute CANcoder** (`hoodAngleSupplier`) so the loop and the
  `hoodAdjustedTrigger` agree on position.
- Add an `ArmFeedforward`/gravity term (`kG·cos(θ)`) like the intake's `Mass` model instead of the
  hand-tuned `+0.375 / −0.25`.
- Gate readiness on calibrated range: re-enable a `distance < maxCalibratedRange` term on
  `shooterReady` so the driver isn't shown "locked on" past the table (A9).

#### B7. Compute the target solution **once per loop** and share it
**Problem:** `getTurretToTargetVector()` (`Shooter.java:341-369`) rebuilds the full transform — pose
read, chassis-speeds read, two `rotateBy`, a ToF map lookup — on **every call**. It's invoked from
`turretRelativeAngleToTarget`, `turretRelativeDistanceFromTarget`, `isTurretAligned`,
`setAdjustedTurretAngle`, `adjustFlyWheelVelocityCommand`, `getDistanceFromHubTarget`,
`getHubOnFieldAfterCalc`, and logging — many times per 20 ms tick. Besides wasted CPU, different
callers can read slightly different pose/velocity snapshots within one loop, so aim, speed, and the
"aligned" check are computed against **inconsistent** target states.

**Solution:** solve the aim vector once in `periodic()` (or a cached supplier invalidated per loop),
store `Translation2d cachedTurretToTarget` + its norm/angle, and have all consumers read the cached
value. Guarantees internal consistency and removes redundant trig/map lookups from the hot path.

#### B8. Tune the flywheel-velocity EMA and ball counter together
**Problem:** the flywheel velocity is EMA-filtered with `alpha = 0.05` (`Shooter.java:174-177`) —
heavy smoothing ≈ a slow low-pass with noticeable lag. The `BallCounter` (`BallCounter.java:18-23`)
detects a shot when the *filtered* velocity dips 5% below setpoint. Heavy filtering both **delays**
and **attenuates** the dip, risking missed counts at high cadence or double-counts on a slow
recovery; `flyWheelReadyTrigger` (`Shooter.java:198`) also reacts late off the same filtered signal.

**Solution:** use a lighter `alpha` (or a median/debounced detector) for shot detection so the dip is
sharp, and add a debounce/refractory window in `BallCounter` so one dip = one ball. Consider counting
off a current spike or a dedicated sensor instead of inferring from filtered RPM if cadence is high.

### Category 4 — Software Architecture & Flow

#### B9. Make the superstructure state machine reflect reality
**Problem:** the stubbed zone triggers (A8) leave a large unreachable decision tree in
`Superstructure.initTriggers()`. Dead-but-present branches make it impossible to reason about what
the robot will actually do, and invite "fixing" code that never runs.

**Solution:** either (a) re-enable `inAllianceZone`/`inIntermediateZone`/`shouldDeliver` with the
real field geometry (the intended logic is preserved in the comment blocks at
`Superstructure.java:57-94`), or (b) delete the unreachable states/triggers so the shipped state
machine is the *whole* state machine. Consider encoding the table (intake? × zone × target → state)
as data rather than 12 hand-built boolean trigger chains — easier to verify exhaustively.

#### B10. Treat loop timing and motor I/O as first-class
**Observations / solutions:**
- `Threads.setCurrentThreadPriority(true, 99)` runs every `robotPeriodic` (`Robot.java:54`) *after*
  the periodic work it's meant to protect — set it once at init, and confirm it brackets the
  scheduler/I/O rather than trailing it.
- `TalonFXMotor.refreshAll()` (`Robot.java:58`) batches signal refresh (good). Pair it with explicit
  `StatusSignal` update-frequency settings and latency-compensated reads on the turret/hood encoders
  so aim uses the freshest sample; you already have a `PerformanceMetricsTracker` — log loop overrun
  to know if any of the above is costing you a 20 ms tick.

#### B11. Remove dead/duplicate code that obscures the control path
Low risk, real clarity gain: `PoseEstimator.java` (unused duplicate of `Odometry`),
`import java.sql.SQLOutput;` in `Intake.java:17`, the empty-map initializers, and the commented map
data. Dead vision/zone code (A5/A8) is the dangerous kind because it *looks* active. Prune or wire it
in — don't leave it ambiguous.

---

## Priority Summary

| # | Item | Type | Effort | Impact |
|---|------|------|--------|--------|
| A4 | Fix `setTurretRotationalVelSup` field bug | Bug | Trivial | High (vision gating) |
| A2/A3 | Fix HIGH/LOW map selector + fill hood angle map | Bug | Low | **Highest (accuracy)** |
| A1/B1 | Enable + iterate time-of-flight lead | Physics | Low–Med | High (move-shoot) |
| B3 | Vision std-dev trust model + use validator | Control | Med | High (localization) |
| B4 | Turret velocity feedforward / profiling | Control | Med | High (tracking) |
| B7 | Cache target solution once per loop | Flow/Perf | Low | Med (consistency) |
| B6 | Hood gravity FF + consistent feedback + range gate | Control | Med | Med–High |
| B2 | Radial-velocity exit-speed compensation | Physics | Med | Med (move-shoot) |
| B8 | Tune EMA vs. ball-counter | Control | Low | Med |
| B5 | Derive conversion factors from geometry | Math/maint | Med | Med (maintainability) |
| A8/B9 | Re-enable or delete dead zone state machine | Architecture | Med | Med (clarity) |
| B10/B11 | Loop timing + dead-code cleanup | Perf/clarity | Low | Low–Med |

**If you do only three things:** A2/A3 (the hood literally isn't aiming by distance), A4 (one-line
fix restoring vision gating), and B3 (vision trust model) — together they fix static accuracy and
field localization, which gate everything else.
****