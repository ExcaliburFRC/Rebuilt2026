# AI Agent Guide for Rebuilt2026

This documentation describes the architecture, patterns, and workflows for the `Rebuilt2026` FRC robot codebase.

## ⚡ Quick Start
- **Build**: `./gradlew build`
- **Deploy**: `./gradlew deploy`
- **Simulate**: `./gradlew simulateJava`
- **Language**: Java 17
- **Dependencies**: WPILib, Monologue, PathPlanner, Phoenix 6, REVLib

## 🏗 Architecture & Code Organization
The project is split into two main packages:
1. `frc.robot`: The specific robot implementation (subsystems, commands, mappings).
2. `frc.excalib`: A reusable library containing core frameworks (Swerve, Motor wrappers, Math utils).

### The Superstructure Pattern
- **Role**: Coordinates state and interactions between multiple subsystems (e.g., aiming Turret while spinning up Shooter).
- **Location**: `src/main/java/frc/robot/superstructure/Superstructure.java`.
- **Pattern**: `RobotContainer` initializes `Superstructure`, which in turn initializes and holds references to subsystems like `Intake`, `Shooter`, `Transport`, `Turret`.

### Subsystem Abstractions (`frc.excalib`)
Avoid using raw vendor classes (like `TalonFX`) directly in subsystems. Use the project's wrappers:
- **Motors**: `frc.excalib.control.motor.controllers.TalonFXMotor` wraps `TalonFX` (Phoenix6).
- **Groups**: `frc.excalib.control.motor.controllers.MotorGroup` for followers.
- **Mechanisms**: `frc.excalib.mechanisms.Mechanism` provides a base for controlled mechanisms (PID, Feedforward).

## 📝 Coding Patterns

### Logging (Monologue)
All subsystems and logic classes should implement `monologue.Logged`.
- Use the `@Log` annotation for fields to be logged.
- `RobotContainer` acts as the root of the log tree.

### Autonomous
- **PathPlanner**: Used for auto routines.
- **AutoBuilder**: Configured in `Swerve.java`.
- **Registration**: Register named commands in `RobotContainer.registerCommands()`:
  ```java
  NamedCommands.registerCommand("shoot", new InstantCommand(...));
  ```
- **Chooser**: `AutoBuilder.buildAuto(name)` is used in `RobotContainer` to prompt the dashboard chooser.

### Command Structure
- **Commands**: Define complex behaviors in `frc.robot.commands`.
- **Inline Commands**: Prefer defining simple composition in `RobotContainer.configureBindings()` using factories (e.g., `.alongWith()`).
- **Factories**: Subsystems expose `Command` factories (e.g., `shooter.shootCommand()`) rather than raw methods.

## 🛠 Key Files
- `src/main/java/frc/robot/RobotContainer.java`: Main entry point for binding controls and autos.
- `src/main/java/frc/robot/Constants.java`: Centralized constants file.
- `src/main/java/frc/robot/superstructure/Superstructure.java`: High-level robot logic and subsystem instantiation.
- `src/main/java/frc/excalib/swerve/Swerve.java`: Custom Swerve implementation with PathPlanner and Odyssey integration.

## ⚠️ Gotchas
- **Vendor Deps**: Wrappers in `frc.excalib` effectively hide the vendor implementation. Check the wrapper before assuming a vendor method exists.
- **Vision**: `AuroraPoseGetter` and `AuroraClient` in `frc.excalib` handle vision updates, integrated into `RobotContainer.periodic()` and `Swerve`.
- **Coordinates**: Uses standard WPILib field coordinates (blue alliance origin).

