# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the FRC 2026 robot codebase for Team Excalibur #6738, robot named *Galahad*. It is a Java/WPILib project built with Gradle, targeting a swerve-drive robot with a turret-based shooter, intake, and transport subsystems.

## Commands

```bash
./gradlew build          # Compile and build
./gradlew deploy         # Deploy to robot (robot must be connected)
./gradlew simulateJava   # Run in simulation
```

There are no unit tests in this project.

## Architecture

The codebase is split into two packages:

- **`frc.excalib`** — Reusable library (git submodule). Contains motor wrappers, mechanism abstractions, swerve drive, odometry, math utilities, and command helpers. Never use vendor classes (TalonFX, SparkMax) directly in robot code — always go through the wrappers here.
- **`frc.robot`** — The robot-specific implementation: subsystems, superstructure, constants, and controller bindings.

### Startup chain

`Robot` (TimedRobot) → `RobotContainer` → `Superstructure` → subsystems (`Shooter`, `Intake`, `Transport`)

`Robot.robotPeriodic()` manually calls `AuroraPoseGetter.periodic()`, `GameDataClient.updateGameData()`, `PeriodicScheduler`, `CommandScheduler.run()`, `Monologue.updateAll()`, and `TalonFXMotor.refreshAll()` — this last call bulk-refreshes all Phoenix6 CAN status signals in one batch per loop.

### Superstructure & RobotState pattern

`Superstructure` owns all subsystems and coordinates them through a `RobotState` enum. Each `RobotState` value is a tuple of `(IntakeStates, ShooterStates, TransportStates)`. Calling `superstructure.setStateCommand(state)` dispatches the correct state to each subsystem simultaneously via `ParallelCommandGroup`.

`Superstructure` uses WPILib `Trigger` composition (not an explicit state machine loop) to automatically transition between states based on field zone, intake button, and alliance.

### Shooter targeting

`Shooter` uses `InterpolatingDoubleTreeMap` lookup tables (distance → hood angle, distance → flywheel velocity, distance → time-of-flight) to compute setpoints. It applies a 3-iteration fixed-point loop in `getTurretToTargetVector()` to compensate for robot velocity (shot lead). The shooter selects between HIGH and LOW goal maps based on the `ShooterStates.targetHeight` field.

`shooterReady` is a compound `Trigger`: turret aligned AND flywheel at setpoint AND hood at setpoint.

### Vision & odometry

`Swerve.periodic()` reads from `LimelightHelpers` (limelight named `"limelight-turret"`) and fuses the result into `Odometry` (a `SwerveDrivePoseEstimator` subclass) via `addVisionMeasurement`. Vision updates are skipped when `TurretOffsetGetter.instance.isFast()` (turret/robot moving too fast for reliable pose).

`TurretOffsetGetter` tracks the turret's angle offset so the limelight pose (which is turret-relative) can be converted to robot-field coordinates in `Swerve.turretToRobot()`.

Aurora (`AuroraPoseGetter`, `AuroraClient`) is a separate vision client that was used previously; the limelight path is now the active one.

### Motor abstraction

`Motor` (interface) → `TalonFXMotor`, `SparkMaxMotor`, `FlexMotor`. `MotorGroup` delegates all calls to a primary motor and sets followers. Always call `setPositionConversionFactor` / `setVelocityConversionFactor` after constructing a motor — raw encoder ticks are meaningless without it.

`TalonFXMotor.refreshAll()` is called once per robot loop (in `Robot.robotPeriodic`) to bulk-refresh Phoenix6 signals; do not call `refresh()` on individual motors in periodic code.

### Mechanism abstractions (`frc.excalib.mechanisms`)

- `Mechanism` — base class wrapping a `Motor`; provides manual/stop/coast commands and SysId support.
- `Arm` — extends Mechanism with gravity feedforward (via `Mass`) and trapezoidal profile position control.
- `FlyWheel` — velocity control with acceleration and jerk limits.
- `Turret` — continuous soft-limit rotation with trapezoidal profile position control.

### Swerve

`Swerve` (in `frc.excalib.swerve`) wraps `ModulesHolder` (four `SwerveModule`s) and a `Pigeon` IMU. PathPlanner `AutoBuilder` is configured inside `Swerve.initAutoBuilder()`. The swerve CAN bus is a separate CANivore (`"SwerveCANivore"`); subsystems use the default `""` CAN bus.

### Logging

All subsystems and `RobotContainer` implement `monologue.Logged`. Use `@Log.NT` on methods/fields to publish them to NetworkTables. `Monologue.setupMonologue(robotContainer, "Robot", ...)` is the root. Do not manually call SmartDashboard for subsystem data — use `@Log.NT` instead.

### Autonomous

Auto routines live in `src/main/deploy/pathplanner/autos/`. Named commands are registered in `RobotContainer.registerCommands()` and reference `RobotState` transitions on the superstructure. The auto chooser is a `SendableChooser<String>` populated from `AutoBuilder.getAllAutoNames()`.

## Key files

| File | Purpose |
|------|---------|
| `frc/robot/Constants.java` | All CAN IDs, PID constants, field geometry, swerve config |
| `frc/robot/RobotContainer.java` | Controller bindings, auto chooser, system health alerts |
| `frc/robot/superstructure/Superstructure.java` | State machine coordination, trigger wiring |
| `frc/robot/superstructure/RobotState.java` | All combined robot states |
| `frc/robot/subsystems/shooter/ShooterStates.java` | Shooter target/mode enum |
| `frc/excalib/swerve/Swerve.java` | Drive, odometry, PathPlanner, vision fusion |

## Important conventions

- **Adding a new `RobotState`**: add the enum entry in `RobotState.java`, add the corresponding `ShooterStates` / `IntakeStates` / `TransportStates` entries if needed, then wire any new trigger logic in `Superstructure.initTriggers()`.
- **Tuning shooter**: edit the `put(distance, value)` entries in `Shooter.initAngleMap()`, `initVelocityMap()`, and `initDistanceTimeOfFlightMap()`. Distance is in meters, angle in radians, velocity in RPS.
- **CAN IDs**: swerve modules use IDs 10/20/30/40 (drive) and 12/22/32/42 (rotation) with CANcoders at 11/21/31/41. All on `"SwerveCANivore"`. Subsystem motors use the default CAN bus.
- **`DISABLE_SUBSYSTEMS` / `DISABLE_SWERVE`** flags in `Constants.java` gate subsystem/swerve initialization for benchtop testing.
- **`PeriodicScheduler`**: a custom 20ms periodic task runner; use `PeriodicScheduler.PERIOD.MILLISECONDS_20.add(task)` for anything that must run every loop but shouldn't be a full WPILib subsystem.