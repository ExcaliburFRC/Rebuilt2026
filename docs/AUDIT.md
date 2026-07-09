# Phase 0 — Audit: ExcaLib & Rebuilt2026 Robot Code

> **Date:** 2026-07-02 · **Branch:** `feature/lib-rebuild` · **Baseline:** `2f015c3` (main)
> **Build status at audit time:** ✅ `./gradlew build` passes
> **Scope:** full inventory of the current library (`frc.excalib`), robot code (`frc.robot`), dependencies, and ranked pain points, prior to the ground-up library rebuild.

---

## 1. Repository shape

| Item | Finding |
|---|---|
| Framework | Java 17, WPILib **2026.2.1** (GradleRIO), command-based |
| Library location | `src/main/java/frc/excalib` — **53 files, ~5,100 lines** |
| Robot code | `src/main/java/frc/robot` — 21 files (~4.5k lines incl. vendored `LimelightHelpers`) |
| Submodule status | ⚠️ `.gitmodules` declares `frc/excalib` as a submodule of `ExcaliburFRC/ExcaLib`, but **all files are tracked directly in this repo** (mode `100644`, no gitlink). The submodule config is vestigial — the library can be rebuilt with normal commits on a branch. |
| Tests | **None** (JUnit 5 is on the classpath; `ShiftUtil` even has test-injection hooks, but no test sources exist) |
| Docs | No `docs/` dir prior to this audit. `MOTOR_LIBRARY_CHANGES.md` documents a previous motor-layer optimization pass. |

### Dependencies (from `build.gradle` + `vendordeps/`)

| Dependency | Version | Notes |
|---|---|---|
| WPILib / GradleRIO | 2026.2.1 | current season |
| **Phoenix 6** | **26.1.0** | TalonFX, CANcoder, Pigeon2 |
| PathPlannerLib | 2026.1.2 | auto — `AutoBuilder` configured in `Swerve` |
| REVLib | (2026) | SparkMax/Flex wrappers exist but **no REV hardware in robot code** |
| Studica | (2026) | NavX wrapper exists; robot uses Pigeon2 |
| AmLib | 2026.0.1 | vendor dep present; usage not found in robot code |
| Monologue | `v1.0.0-beta6` (jitpack) | current telemetry annotation lib — beta |
| JUnit | 5.10.1 | unused |

---

## 2. Current library inventory (`frc.excalib`)

### 2.1 `control.motor` — motor abstraction

| Class | Purpose | Notes |
|---|---|---|
| `Motor` (interface) | vendor-agnostic motor API — raw-double getters/setters, conversion factors, current limits, soft limit, follower | no closed-loop API at the interface level: only voltage/percent |
| `TalonFXMotor` | `extends TalonFX implements Motor`. Static per-bus `StatusSignal` registry + `refreshAll()` batch refresh; pre-allocated control requests; ramp defaults; `configurePID`/`configureMotionMagic`/`setVelocityDirect`/`setPositionDirect`/`setMotionMagicPosition` | onboard closed-loop methods exist but are **Voltage-only, and nothing in the robot uses them** (see P-02) |
| `SparkMaxMotor`, `FlexMotor` | REV equivalents | unused by robot |
| `MotorGroup` | delegates to primary, sets followers | |
| `DirectionState`, `IdleState` | enums | |

### 2.2 `mechanisms` — mechanism layer

| Class | Control strategy | Notes |
|---|---|---|
| `Mechanism` (base) | open-loop voltage/duty + SysId routines + coast command | all mechanism telemetry via Monologue `@Log.NT` |
| `Arm` | **software**: velocity setpoint = `error / 0.02` clamped by `SoftLimit`, then ks/kv/kg feedforward + RIO `PIDController`, voltage out | effectively a disguised P-controller with gain `kv/0.02`; no real motion profile |
| `FlyWheel` | **software**: RIO `PIDController` + `SimpleMotorFeedforward`, voltage out; optional `TrapezoidProfile` on velocity; current-based accel estimation option | |
| `Turret` | **software**: `ContinuousSoftLimit` wrap-around setpoint + optional `TrapezoidProfile` + RIO PID + ks/kv, voltage out | |
| `LinearExtension` | similar software pattern | unused by robot |

### 2.3 `swerve`

| Class | Purpose | Notes |
|---|---|---|
| `Swerve` | drive commands, PathPlanner `AutoBuilder`, pose estimation w/ Limelight fusion in `periodic()`, pid-to-pose, Elastic/Shuffleboard UI | **imports `frc.robot.*`** (Constants, LimelightHelpers, TurretOffsetGetter) — see P-01 |
| `ModulesHolder` | 4-module aggregate, kinematics | |
| `SwerveModule` | module = `FlyWheel` (drive) + `Turret` (steer) — all software PID voltage control | drive FlyWheel constructed with magic accel/jerk `(10, 10)`; CANcoder read via lambda from `Constants` |
| `SwerveAccUtils` | accel-limiting helper | its call site in `Swerve.driveCommand` is commented out |

### 2.4 `slam.mapper` — odometry & vision

| Class | Purpose | Notes |
|---|---|---|
| `Odometry` | `SwerveDrivePoseEstimator` subclass | |
| `AuroraClient`, `PoseEstimator`, `VisionMeasurementValidator` | previous Aurora vision client | **dead path** — limelight is active; `AuroraPoseGetter` referenced in CLAUDE.md no longer exists in the tree |

### 2.5 `control.*` utilities

| Package | Contents |
|---|---|
| `control.gains` | `Gains` (public-field kp/ki/kd/ks/kv/ka/kg bag), `SysidConfig` |
| `control.imu` | `IMU` interface, `Pigeon`, `NavX` |
| `control.limits` | `SoftLimit`, `ContinuousSoftLimit` (wrap-aware setpoint chooser — genuinely good idea) |
| `control.math` | `Vector2D`, `Circle`, `Line`, `MathUtils` (**imports `frc.robot.Constants`** — P-01), `EMAFilter`, `physics.Mass` |
| `control.math.periodics` | `PeriodicScheduler` / `PeriodicTask` — custom 20 ms task runner |

### 2.6 `additional_utilities` & `commands`

`Alliance`, `AllianceUtils` (alliance-relative poses, field dims), `AutoChooserManager` (unused), `CANHealthMonitor` (constructed, update commented out), `Color`, `ControllerStateTracker`, `DoubleClickClient`, `DoubleKeyMap`, `Elastic` (dashboard notifications), `LEDs` (singleton), `LoggablePS5Controller`, `PerformanceMetricsTracker`, `Position`, `RobotDiagnostics` (commented out), `CommandMutex`, `CommandUtils`, `ContinuouslyConditionalCommand`, `MapCommand`.

---

## 3. Robot code inventory (`frc.robot`)

### 3.1 Startup chain & loop

`Robot` (TimedRobot) → `RobotContainer` → `Superstructure` → `Shooter` / `Intake` / `Transport`; `Swerve` built by **static factory in `Constants`**.

`Robot.robotPeriodic()` order: `ShiftUtil.update()` → `robotContainer.periodic()` → `PeriodicScheduler` → **`Threads.setCurrentThreadPriority(true, 99)`** → `CommandScheduler.run()` → `Monologue.updateAll()` → `TalonFXMotor.refreshAll()`.

### 3.2 Subsystems

| Subsystem | Hardware | Control today |
|---|---|---|
| `Shooter` | hood TalonFX + CANcoder, 2× flywheel TalonFX (`MotorGroup`), turret TalonFX + CANcoder | turret: `Turret` mechanism (software profile+PID); flywheel: `FlyWheel` software PID; hood: **hand-rolled PID in the subsystem** with asymmetric hardcoded kS (+0.375 / −0.25); interpolation maps (HIGH/LOW: angle, velocity, time-of-flight); 3-iteration shot-lead fixed point; `BallCounter` from flywheel velocity dips; `shooterReady` = turret ∧ flywheel ∧ hood triggers |
| `Intake` | 2× four-bar TalonFX (`MotorGroup`) + CANcoder (constructed, **never read** — arm zeroed from a hardcoded constant), roller TalonFX | four-bar: `Arm` mechanism; roller: open-loop voltage from state enum; "PUMP" mode oscillates setpoint 1.60↔0 on 0.5 s timeouts |
| `Transport` | drum + transport TalonFX | 2× `FlyWheel` software velocity, gated on `shooterReady` |
| `Swerve` | 8× TalonFX + 4× CANcoder + Pigeon2 on `SwerveCANivore` | all software PID (see 2.3) |

### 3.3 State handling today ("superstructure")

- `RobotState` enum = tuple of (`IntakeStates`, `ShooterStates`, `TransportStates`); 14 states incl. a `_test` state.
- `Superstructure.setStateCommand()` = `ParallelCommandGroup` of three `InstantCommand`s that **mutate a `currentState` field** in each subsystem; the subsystems' default commands poll that field with `.until()`/`ConditionalCommand`.
- Trigger wiring (`initTriggers`) auto-transitions on button+zone+shift conditions, **but the zone triggers are stubbed** (`inAllianceZone = () -> true`, `inIntermediateZone = () -> false`) and ~90 lines of trigger/LED logic are commented out. `ledStateFor()` is dead code.
- No transition table, no guards, no interlocks — any state is commandable at any time.

### 3.4 Vision & odometry

- `Swerve.periodic()` uses `LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-turret")` — **MegaTag1**, not MegaTag2; gated by tag count / ambiguity < 0.3 / `TurretOffsetGetter.isFast()`.
- Turret-mounted limelight: pose is turret-relative, converted by `turretToRobot()` which **hardcodes `0.215`** (duplicating `TURRET_OFFSET_TRANSLATION` with opposite sign convention).
- Fixed vision std-devs (whatever `Odometry` defaults to) — no distance/tag-count scaling.

### 3.5 Auto

- PathPlanner autos in `src/main/deploy/pathplanner/autos/` (4 files incl. `New Auto.auto`); chooser from `AutoBuilder.getAllAutoNames()`.
- 🚨 `registerCommands()` is **commented out in the `RobotContainer` constructor** — any auto referencing `idle`/`shoot`/`intake` named commands will not do those actions (PathPlanner logs missing-command errors and substitutes `Commands.none()`).

### 3.6 Game-specific logic

- `ShiftUtil` — REBUILT hub-shift tracker: period model, FMS game-data parsing, hub-active prediction, listeners, DS sign code. Well-documented, designed for testability. **Keep.**
- `TurretOffsetGetter` — turret-angle offset for vision-pose conversion + "too fast" gate.

---

## 4. Pain points & smells — ranked

### 🟥 Critical

| ID | Finding | Evidence | Consequence |
|---|---|---|---|
| **P-01** | **Library depends on robot code** — `frc.excalib.swerve.Swerve` imports `frc.robot.Constants`, `frc.robot.util.LimelightHelpers`, `frc.robot.util.TurretOffsetGetter`; `frc.excalib.control.math.MathUtils` imports `frc.robot.Constants` | `Swerve.java:27-39`, `MathUtils.java:5` | The "library" is not reusable, not season-independent, and untestable in isolation. Root architectural defect the rebuild must fix. |
| **P-02** | **All closed-loop control is software PID on the RIO at 50 Hz, voltage-out** — `Arm`, `FlyWheel`, `Turret`, hood, and all 8 swerve motors. `TalonFXMotor`'s onboard `VelocityVoltage`/`MotionMagic` methods exist but have **zero call sites**. No FOC / `TorqueCurrent` anywhere despite Phoenix Pro license. | mechanisms package; `Shooter.getControlledOutputForAngle` | Leaves the biggest available control-quality win (1 kHz onboard FOC closed loop, MotionMagic/Expo) unused; wastes RIO loop headroom; ~20 ms control latency. |
| **P-03** | **No simulation support at all** — zero `TalonFXSimState` / `DCMotorSim` / maple-sim usage; no `simulationPeriodic` anywhere | grep across `src/` | `simulateJava` launches a GUI where nothing moves; code can only be validated on the real robot. |
| **P-04** | **Auto named commands never registered** — `registerCommands()` call commented out | `RobotContainer.java:78` | Competition autos silently lose their intake/shoot actions. |
| **P-05** | **Hardware constructed in `Constants` static initializers** — 4 CANcoders + Pigeon + full swerve factory live in the constants class; one CANcoder is `public static` and used directly elsewhere | `Constants.SwerveConstants` | Device creation order/failure is invisible, benchtop `DISABLE_SWERVE` flag can't prevent CAN traffic from static init, constants class is not "just data". |
| **P-06** | **Magic-number calibration baked into code** — e.g. hood conversion `POSITION_CONVERSION_FACTOR * (−0.208/1.497) * 1.0231`, asymmetric hood kS `+0.375/−0.25`, turret offset `0.215` re-hardcoded in `Swerve.turretToRobot`, intake zero `Math.PI − 0.732601 − 0.159`, per-module fudge `2.065 * 0.976` | `Shooter.java:164,433-435`, `Swerve.java:509`, `Intake.java:72` | Untraceable tuning, duplicated truths (turret offset exists in two places with different signs), silent breakage when one copy changes. |

### 🟧 High

| ID | Finding | Evidence | Consequence |
|---|---|---|---|
| **P-07** | **No config apply+verify** — each `set*` does its own blocking `refresh`→mutate→`apply` round-trip; failures only produce a DriverStation warning; no retry, no read-back verification, no single `TalonFXConfiguration` snapshot | `TalonFXMotor` config methods | A brown-out/CAN hiccup during init leaves a motor half-configured and nobody knows. |
| **P-08** | **Vision uses MegaTag1** (`getBotPoseEstimate_wpiBlue`), fixed std-devs, manual ambiguity gating; no `SetRobotOrientation` feed for MT2 | `Swerve.periodic()` | Noisier pose, alliance-flip fragility; the stack the team asked for (MT2) isn't wired. |
| **P-09** | **"State machine" is mutable-enum-field polling** — states set via `InstantCommand` side effects, consumed by default commands with `.until()`; no transition table, no guards, no interlocks, no illegal-state protection; zone triggers stubbed to constants; large commented-out trigger blocks | `Superstructure`, all three subsystems | Race-prone, hard to reason about, impossible to enforce mechanism interlocks; exactly what the rebuild's state-machine engine must replace. |
| **P-10** | **Raw-double unit soup** — API mixes rad, RPS, m/s with `double`; correctness depends on calling `setPositionConversionFactor`/`setVelocityConversionFactor` after construction (CLAUDE.md itself warns "raw encoder ticks are meaningless without it") | `Motor` interface, all mechanisms | Classic source of silent wrong-units bugs; WPILib Units exist and are unused. |
| **P-11** | **Telemetry is fragmented & beta** — Monologue `v1.0.0-beta6` annotations + ad-hoc `SmartDashboard` + `Shuffleboard` tabs + raw NT entries (`Tab1/flywheelVel`) + `Elastic` notifications, with dozens of copy-paste `@Log.NT` getter methods | `Superstructure` (15 boolean getters), `Swerve.initElastic`, `RobotContainer` | No single log of record; high boilerplate; beta dependency. |
| **P-12** | **Dead / vestigial code** — Aurora slam package, `SwerveAccUtils` (call commented), `AutoChooserManager`, `RobotDiagnostics`, `CANHealthMonitor.update()` commented, `ledStateFor()` never called, `NO_INTAKE_SHOOT_HUB_test` state, `yoavHatesThisCommandCommand`, unused REV/NavX wrappers, stale `.gitmodules`, unused import `java.sql.SQLOutput` | throughout | Obscures what's real; inflates maintenance surface. |
| **P-13** | **Swerve translation/rotation `Gains()` are all-zero** — `ANGLE_PID_GAINS`/`TRANSLATION_PID_GAINS` constructed empty, so `turnToAngleCommand` and `pidToPoseCommand` output ~0 | `Constants.java:103-104` | Move-to-pose and turn-to-angle features are inert. |
| **P-14** | **Intake CANcoder constructed but never read** — arm position zeroed from a hardcoded constant instead of the absolute encoder | `Intake.java:56,72` | Four-bar position is wrong after any power-cycle in a non-stow pose. |

### 🟨 Medium

| ID | Finding | Evidence |
|---|---|---|
| P-15 | `Threads.setCurrentThreadPriority(true, 99)` every loop — RT priority 99 competes with NI/CAN driver threads; also re-applied redundantly each iteration | `Robot.java:49` |
| P-16 | Deadband without rescaling — output jumps from 0 to 0.2·MAX at the deadband edge | `RobotContainer.applyDeadband` |
| P-17 | Allocation-heavy hot paths — `getTurretToTargetVector()` returns a new capturing `Supplier` per call; new `Translation2d`/`Rotation2d` chains per loop; suppliers-of-suppliers indirection | `Shooter.java:334+` |
| P-18 | `refreshAll()` runs **after** `CommandScheduler.run()` — commands act on signals from the previous loop (+20 ms sensor latency) | `Robot.robotPeriodic` |
| P-19 | Inconsistent current limits with no rationale (turret 120/80 vs hood 40/30 vs drum 80/80); no `SupplyCurrentLowerLimit`/time (breaker-trip protection), no `TorqueCurrent` peaks | subsystem constructors |
| P-20 | `MAX_VEL = 2` > `MAX_MODULE_VEL = 1.5` (inconsistent, and both far below hardware capability — presumably test values still in place) | `Constants` |
| P-21 | Non-deterministic init coupling: `Shooter` constructor writes into `TurretOffsetGetter.instance` singletons that `Swerve.periodic()` reads; `LEDs.getInstance()` side effects in `Robot` ctor | `Shooter.java:235-237` |
| P-22 | `pumpCommand` magic numbers (`1.60`, `0.5 s`), transport conversion `0.0731`, drum `0.39898/9` — no derivation comments | `Intake`, `Transport` |
| P-23 | No unit tests despite JUnit + `ShiftUtil` test hooks | `src/test` absent |

### 🟩 Low

| ID | Finding |
|---|---|
| P-24 | Naming/style inconsistencies: package `mechanisms.Arm` (capitalized) vs `fly_wheel` (snake_case); `logVelocity()` used as a getter; Hungarian-ish `m_` mixed with bare fields |
| P-25 | `System.out` debugging & commented-out print blocks left in `Swerve`, `RobotContainer` |
| P-26 | Javadoc coverage thin outside `ShiftUtil`/`TalonFXMotor`; several docs stale (e.g. `Turret` "1 kHz" claims on software-PID code paths) |
| P-27 | `Robot()` does work in constructor vs `robotInit()`; `DriverStation.silenceJoystickConnectionWarning` not set for sim |

---

## 5. What's worth keeping (seeds for the rebuild)

| Asset | Why |
|---|---|
| `TalonFXMotor` signal registry: per-bus batched `refreshAll()`, deliberate update frequencies, `optimizeBusUtilization` | Exactly the StatusSignal discipline the new library needs — keep the idea, re-home it |
| `ContinuousSoftLimit` wrap-aware setpoint selection | The turret min-path-within-limits logic is genuinely useful; port it |
| Shooter interpolation maps + 3-iteration shot-lead loop + moving-shot velocity compensation | Proven competition behavior — must survive migration unchanged (P-06 cleanup aside) |
| `ShiftUtil` | Well-designed, documented, testable game-data logic |
| `AllianceUtils` alliance-relative pose types | Good pattern, keep concept |
| `BallCounter` velocity-dip counting, `EMAFilter` | Simple and effective |
| PathPlanner integration shape (`AutoBuilder` config, chooser) | Works; will be rebuilt on the new swerve but same tool |

---

## 6. Summary & go-ahead

The codebase is a working competition robot on top of a library with the **right instincts** (motor abstraction, mechanism layer, batched status signals, wrap-aware turret limits) but **wrong load-bearing choices**: the library depends on the robot code (P-01), every mechanism closes its loop in software at 50 Hz voltage-out while a Pro-licensed FOC controller idles (P-02), there is no simulation (P-03), and "state machine" means enum fields mutated by `InstantCommand`s (P-09). Config, units, telemetry, and calibration are all ad-hoc (P-05/06/07/10/11).

Two 🟥 findings are **live competition bugs independent of the rebuild**: unregistered auto named commands (P-04) and, arguably, the all-zero pid-to-pose gains (P-13) — worth fixing on `main` regardless of this project.

**Recommendation:** proceed to Phase 1 (research). The rebuild should be a clean `frc.excalib` v2 (new package root, old library untouched until migration is proven), Phoenix-6-native with TorqueCurrentFOC defaults, unit-safe config-as-code, a real state-machine engine + superstructure coordinator, DogLog/Epilogue + SignalLogger telemetry, and Phoenix 6 sim + maple-sim parity.
