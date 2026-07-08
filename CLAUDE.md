# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the FRC 2026 (REBUILT game) robot codebase for Team Excalibur #6738, robot named *Galahad*. It is a Java 17 / WPILib project built with Gradle (GradleRIO 2026.2.1), targeting a swerve-drive robot with a turret-based shooter, intake, and transport subsystems. Vendor deps: Phoenix6, REVLib, PathPlanner, Studica, AmLib.

## Commands

```bash
./gradlew build          # Compile and build
./gradlew deploy         # Deploy to robot (robot must be connected)
./gradlew simulateJava   # Run in simulation
```

There are no unit tests in this project (no `src/test` directory).

## Architecture

The codebase is split into two packages:

- **`frc.excalib`** — Reusable team library: motor wrappers, mechanism abstractions, swerve drive, odometry, math utilities, and command helpers. It originated as a git submodule (`.gitmodules` still references ExcaLib) but is now tracked as regular files and edited in place in this repo. `MOTOR_LIBRARY_CHANGES.md` at the repo root documents a recent optimization pass over the motor/mechanism classes. Never use vendor classes (TalonFX, SparkMax) directly in robot code — always go through the wrappers here.
- **`frc.robot`** — The robot-specific implementation: subsystems, superstructure, constants, and controller bindings.

### Startup chain

`Robot` (TimedRobot) → `RobotContainer` → `Superstructure` → subsystems (`Shooter`, `Intake`, `Transport`)

`Robot.robotPeriodic()` manually calls, in order: `ShiftUtil.update()`, `RobotContainer.periodic()`, `PeriodicScheduler`, `CommandScheduler.run()`, `Monologue.updateAll()`, and `TalonFXMotor.refreshAll()` — this last call bulk-refreshes all Phoenix6 CAN status signals in one batch per loop. Loop timing and power draw are recorded by `PerformanceMetricsTracker` (summary printed on auto/teleop exit). `ShiftUtil.reset()` is called in `disabledInit()`.

### ShiftUtil (2026 game mechanic)

`frc.robot.util.ShiftUtil` is a static utility that tracks REBUILT's hub-shift mechanic: during teleop the two alliance hubs alternate active/inactive across four 25-second shifts, decided by an FMS game message ('R'/'B'). It exposes `isOwnHubActive()`, shift/period queries, flip prediction, and listeners. `Superstructure`'s shoot triggers are meant to be driven by it (currently wired to a controller button instead). It must be `update()`ed every loop and `reset()` between matches — `Robot` already does both.

### Superstructure & RobotState pattern

`Superstructure` owns all subsystems and coordinates them through a `RobotState` enum. Each `RobotState` value is a tuple of `(IntakeStates, ShooterStates, TransportStates)`. Calling `superstructure.setStateCommand(state)` dispatches the correct state to each subsystem simultaneously via `ParallelCommandGroup`.

`Superstructure` uses WPILib `Trigger` composition (not an explicit state machine loop) to automatically transition between states. Its constructor takes three input triggers wired in `RobotContainer`: intake button (R2), alliance-shift-active (square), and delivery button (circle). Several field-zone triggers (`inAllianceZone`, `inIntermediateZone`) are currently stubbed to constants (`true`/`false`) with the real pose-based logic commented out — check before relying on them.

### Shooter targeting

`Shooter` uses `InterpolatingDoubleTreeMap` lookup tables (distance → hood angle, distance → flywheel velocity, distance → time-of-flight) to compute setpoints, with separate HIGH and LOW map sets selected by the `ShooterStates.targetHeight` field (`TargetHeight` enum). It applies a 3-iteration fixed-point loop in `getTurretToTargetVector()` to compensate for robot velocity (shot lead) using the time-of-flight map.

`shooterReady` is a compound `Trigger`: turret aligned AND flywheel at setpoint AND hood at setpoint. `Transport` is constructed with this trigger and only feeds when the shooter is ready.

The hood's maximum soft limit shrinks when the robot is in the trench zone (`volatileTrenchHoodTrigger`) to avoid contact with the trench.

### Vision & odometry

`Swerve.periodic()` reads from `LimelightHelpers` (limelight named `"limelight-turret"`) and fuses the result into `Odometry` (a `SwerveDrivePoseEstimator` subclass) via `addVisionMeasurement`. Vision updates are skipped when `TurretOffsetGetter.instance.isFast()` (turret/robot rotating too fast for reliable pose).

`TurretOffsetGetter` tracks the turret's angle offset so the limelight pose (which is turret-relative) can be converted to robot-field coordinates in `Swerve.turretToRobot()`. `Shooter`'s constructor wires its suppliers.

`AuroraClient` (`frc.excalib.slam`) is a leftover from a previous vision pipeline; the limelight path is the active one.

### Motor abstraction

`Motor` (interface) → `TalonFXMotor`, `SparkMaxMotor`, `FlexMotor`. `MotorGroup` delegates all calls to a primary motor and sets followers. Always call `setPositionConversionFactor` / `setVelocityConversionFactor` after constructing a motor — raw encoder ticks are meaningless without it.

`TalonFXMotor` notes (see `MOTOR_LIBRARY_CHANGES.md` for the full list):
- `refreshAll()` is called once per robot loop (in `Robot.robotPeriodic`); do not call `refresh()` on individual motors in periodic code.
- Control requests are pre-allocated and mutated in place — keep it that way (zero GC per loop).
- Onboard closed-loop is available via `configurePID(gains)` + `setVelocityDirect` / `setPositionDirect` / `setMotionMagicPosition`, running at 1 kHz on the motor controller instead of 50 Hz software PID.
- `setFollower(id)` defaults to Aligned; use the overload for opposed followers.

### Mechanism abstractions (`frc.excalib.mechanisms`)

- `Mechanism` — base class wrapping a `Motor`; provides manual/stop/coast commands and SysId support.
- `Arm` — extends Mechanism with gravity feedforward (via `Mass`) and trapezoidal profile position control.
- `FlyWheel` — velocity control with acceleration and jerk limits.
- `Turret` — continuous soft-limit rotation with optional trapezoidal profile position control.
- `LinearExtension` — profiled linear mechanism.

### Swerve

`Swerve` (in `frc.excalib.swerve`) wraps `ModulesHolder` (four `SwerveModule`s) and a `Pigeon` IMU, and is constructed by `Constants.SwerveConstants.configureSwerve()`. PathPlanner `AutoBuilder` is configured inside `Swerve.initAutoBuilder()`. The swerve CAN bus is a separate CANivore (`"SwerveCANivore"`); subsystems use the default `""` CAN bus (`SUBSYSTEMS_CANBUS`).

### Logging

All subsystems and `RobotContainer` implement `monologue.Logged`. Use `@Log.NT` on methods/fields to publish them to NetworkTables. `Monologue.setupMonologue(robotContainer, "Robot", ...)` is the root. Do not manually call SmartDashboard for subsystem data — use `@Log.NT` instead (`ShiftUtil` is the one exception, publishing its own telemetry).

### Autonomous

Auto routines live in `src/main/deploy/pathplanner/autos/`. Named commands are registered in `RobotContainer.registerCommands()` and reference `RobotState` transitions on the superstructure — note this call is currently commented out in the `RobotContainer` constructor, so PathPlanner named commands are not registered; re-enable it if an auto needs them. The auto chooser is a `SendableChooser<String>` populated from `AutoBuilder.getAllAutoNames()`, with a `"/ null Auto"` default that resolves to `Commands.none()`.

## Key files

| File | Purpose |
|------|---------|
| `frc/robot/Constants.java` | All CAN IDs, PID constants, field geometry, swerve config |
| `frc/robot/RobotContainer.java` | Controller bindings, auto chooser, health alerts |
| `frc/robot/superstructure/Superstructure.java` | State machine coordination, trigger wiring |
| `frc/robot/superstructure/RobotState.java` | All combined robot states |
| `frc/robot/subsystems/shooter/ShooterStates.java` | Shooter target/height/shooting enum |
| `frc/robot/util/ShiftUtil.java` | Hub-shift game data tracking and prediction |
| `frc/excalib/swerve/Swerve.java` | Drive, odometry, PathPlanner, vision fusion |

## Important conventions

- **Adding a new `RobotState`**: add the enum entry in `RobotState.java`, add the corresponding `ShooterStates` / `IntakeStates` / `TransportStates` entries if needed, then wire any new trigger logic in `Superstructure.initTriggers()`.
- **Tuning shooter**: edit the `put(distance, value)` entries in `Shooter.initAngleMap()`, `initVelocityMap()`, `initDistanceTimeOfFlightMap()` (HIGH goal) and `initLowMaps()` (LOW goal). Distance is in meters, angle in radians, velocity in RPS.
- **CAN IDs**: swerve modules use IDs 10/20/30/40 (drive) and 12/22/32/42 (rotation) with CANcoders at 11/21/31/41 and the Pigeon at 2, all on `"SwerveCANivore"`. Subsystem motors use the default CAN bus.
- **`DISABLE_SUBSYSTEMS` / `DISABLE_SWERVE`** flags in `Constants.java` suppress the default commands of the shooter/transport and swerve respectively (via `.unless(...)`) for benchtop testing.
- **`PeriodicScheduler`**: a custom 20ms periodic task runner; use `PeriodicScheduler.PERIOD.MILLISECONDS_20.add(task)` for anything that must run every loop but shouldn't be a full WPILib subsystem (e.g. the flywheel EMA filter).
