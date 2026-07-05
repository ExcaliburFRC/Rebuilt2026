# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the FRC 2026 robot codebase for Team Excalibur #6738, robot named *Galahad*. It is a Java/WPILib project built with Gradle, targeting a swerve-drive robot with a turret-based shooter, intake, and transport subsystems.

## Commands

```bash
./gradlew build          # Compile and build (runs unit tests)
./gradlew deploy         # Deploy to robot (robot must be connected)
./gradlew simulateJava   # Run in simulation
```

On this Windows machine, Gradle must run on the WPILib JDK for tests/sim (other JDKs' bundled MSVC runtime crashes the WPILib natives):
`./gradlew simulateJava "-Dorg.gradle.java.home=C:/Users/Public/wpilib/2026/jdk"` (the `test` task is already pinned in build.gradle).

## Architecture

- **`frc.excalib2`** — **ExcaLib v2, the one and only library.** Phoenix-6-native, FOC-first, declarative. All robot subsystems are built on it. Never imports `frc.robot`.

ExcaLib v1 (`frc.excalib`) has been **removed** — the migration is complete and the robot runs entirely on v2. Only Monologue remains as a legacy logging dependency (a separate vendordep; DogLog is the log of record). If you find a reference to `frc.excalib` (no "2"), it's a mistake.

Design history and rationale: `docs/AUDIT.md` (v1 pain points), `docs/RESEARCH.md` (patterns mined from other teams), `docs/DESIGN.md` (approved v2 design), `docs/V1_TO_V2_MAPPING.md` (where each old class went), `docs/EXCALIB_MANUAL.html` (the full reference), `NOTICE.md` (credits).

### ExcaLib v2 in one paragraph

A subsystem = **mechanism declarations + a state machine**. A mechanism is declared with a `MechanismConfig` (devices, ratios, gains real+sim, `MotionConstraints`, `CurrentBudget`, soft limits, tolerance, sim model) and instantiated as one of three archetypes: `PositionalMechanism` (MotionMagic/Expo goals, gravity FF, continuous-wrap option for turrets), `VelocityMechanism` (MotionMagicVelocity or plain velocity), `RollerMechanism` (volts/amps/duty + current-spike detection). The base `Mechanism` class owns config apply+verify+retry (`DeviceConfigs`), StatusSignal registration (`SignalHub`), DogLog telemetry, disconnect Alerts, async brake/coast, SysId, and physics sim (`MechanismSim`). States are declared on `StateMachine<S>` (default-deny transition table, guards, onEnter/whileIn/onExit); the robot-level `Superstructure<RobotState>` fans each goal out to the subsystem machines. **Recipe: `docs/ADDING_A_SUBSYSTEM.md`.**

### Control conventions

- Everything runs **onboard the TalonFX in mechanism units** (`SensorToMechanismRatio` in config) — there are no RIO-side conversion factors in v2.
- `ControlMode.TORQUE_CURRENT_FOC` is the library default; **all migrated mechanisms currently run `VOLTAGE`** because their gains were ported from v1's volts-based software PID. The FOC switch is a one-line config change per mechanism **after a SysId session** (gains do not transfer between modes).
- Simulation always forces VOLTAGE mode and uses the config's sim gain set.
- Current limits come from `CurrentBudget` records — keep every mechanism's amps in its subsystem constants, reviewable at a glance.

### Robot loop (Robot.robotPeriodic order matters)

`SignalHub.refreshAll()` (fresh signals BEFORE commands) → `FaultReporter.poll()` → `ShiftUtil.update()` → `PeriodicScheduler` (legacy) → `CommandScheduler.run()` → `Monologue.updateAll()` (legacy) → `TalonFXMotor.refreshAll()` (legacy v1 motors). `Telemetry.init(...)` and `SignalHub.optimizeAll()` run once at boot.

### Superstructure & states

`RobotState` enum = tuple of (`IntakeStates`, `ShooterStates`, `TransportStates`). `Superstructure` (extends `frc.excalib2.superstructure.Superstructure<RobotState>`) enters a goal via `request(goal)`; `onEnter` fans out to `shooter/transport/intake.requestState(...)`. Trigger wiring (buttons × zones × alliance shift) lives in `initTriggers()`; the field-zone triggers are **intentionally stubbed** (`inAllianceZone = true`, `inIntermediateZone = false`) — same as pre-migration.

### Shooter targeting (proven competition logic — treat as tuning-sacred)

`InterpolatingDoubleTreeMap` lookup tables (distance → hood angle / flywheel velocity / time-of-flight; HIGH and LOW goal sets), a 3-iteration fixed-point shot-lead loop in `getTurretToTargetVector()`, and radial-velocity compensation of flywheel speed. Tuning: edit the `put(distance, value)` entries in `Shooter.initAngleMap()/initVelocityMap()/initDistanceTimeOfFlightMap()/initLowMaps()` — distance in meters, angle radians, velocity RPS.

### Swerve & vision

`frc.excalib2.swerve.SwerveSubsystem` wraps the CTRE `SwerveDrivetrain` (250 Hz odometry, built-in sim). Module constants are hand-authored in `frc.robot.SwerveConfig` (⚠ inversions/offsets/coupling ratio still need on-robot verification). Vision is **Limelight MegaTag2** (`limelight-turret`): the turret-corrected heading is fed via `SetRobotOrientation`, the returned turret-frame pose is transformed to a robot pose (`SwerveConfig.turretPoseToRobotPose`), gated by `TurretOffsetGetter.isFast()`, fused with distance/tag-count-scaled std-devs.

### Logging

**DogLog is the log of record** (+ Phoenix `SignalLogger` hoot for device-rate data; both open in AdvantageScope). Log with `DogLog.log("Subsystem/Thing", value)`; mechanisms auto-log their basics. Live tuning via `frc.excalib2.telemetry.TunableNumber` (enable in dev only). Monologue `@Log.NT` remains only in legacy v1 classes and `RobotContainer` — don't add new Monologue usage.

### Autonomous

PathPlanner. Autos in `src/main/deploy/pathplanner/autos/`; named commands registered in `RobotContainer.registerCommands()` (must stay called — unregistered names silently no-op); `AutoBuilder` configured in `SwerveConfig.createDrivetrain()`.

## Key files

| File | Purpose |
|------|---------|
| `frc/robot/SwerveConfig.java` | Swerve module constants, PathPlanner gains, vision wiring |
| `frc/robot/Constants.java` | Field geometry, CAN bus, controller ports (legacy swerve section unused — do not reference `Constants.SwerveConstants`, loading it constructs old devices) |
| `frc/robot/superstructure/Superstructure.java` | Goal machine + trigger wiring |
| `frc/robot/subsystems/*/[X]Constants.java` | Per-mechanism configs, gains, budgets (with v1-derivation comments) |
| `frc/excalib2/mechanisms/MechanismConfig.java` | The declarative mechanism builder |
| `frc/excalib2/statemachine/StateMachine.java` | The state-machine engine |
| `docs/ADDING_A_SUBSYSTEM.md` | The add-a-subsystem recipe |

## Important conventions

- **New subsystems**: follow `docs/ADDING_A_SUBSYSTEM.md` — declare a `MechanismConfig` + states; do not hand-write control loops or call vendor classes directly.
- **Adding a `RobotState`**: add the enum entry (plus subsystem-state entries if needed); fan-out and `transitionFromAny` are generated from `RobotState.values()`; wire new triggers in `Superstructure.initTriggers()`.
- **CAN IDs**: swerve drive 10/20/30/40, steer 12/22/32/42, CANcoders 11/21/31/41, Pigeon 2 — all on `"SwerveCANivore"`. Subsystem motors on the RIO bus (`""`).
- **`DISABLE_SUBSYSTEMS` / `DISABLE_SWERVE`** flags in `Constants.java` gate behavior for benchtop testing.
- **Unit tests** live in `src/test/java` (state-machine engine is covered); they run on the WPILib JDK automatically.
- Branch discipline: never commit directly to `main`; library work on `feature/lib-rebuild`, migration on `feature/robot-migration`.
