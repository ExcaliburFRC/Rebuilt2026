# ExcaLib v1 → v2 mapping

Every class in the old library (`frc.excalib`) and where its functionality lives now.
Nothing was silently dropped: each row is either **ported**, **replaced** (equivalent or
better in v2 / WPILib / CTRE), or **dropped** with the reason stated.

Legend: ✅ ported/replaced in v2 · 🔷 covered by WPILib/CTRE/PathPlanner directly · ⛔ dropped deliberately

## Motors & control

| v1 | Status | v2 answer |
|---|---|---|
| `Motor` (interface) | ✅ | `device.Motor` — mechanism-unit API, verified config, FOC+Voltage request families (v1's interface + `TalonFXMotor` impl collapse into one class; TalonFX-only since REV was dropped) |
| `TalonFXMotor` (incl. `refreshAll`, signal registry) | ✅ | `device.Motor` + `device.SignalHub` (per-bus batched refresh, frequency tiers, `optimizeAll`, latency compensation) |
| `MotorGroup` (followers) | ✅ | `MechanismConfig.follower(id, opposed)` |
| `SparkMaxMotor`, `FlexMotor` | ⛔ | No REV hardware (approved decision OD-8); old classes remain in git history |
| `Gains` | ✅ | `control.Gains` (record; constructors; maps to `Slot0Configs` incl. kG/GravityType) |
| `SysidConfig` | ✅ | Built into `Mechanism.sysIdQuasistatic/Dynamic` and `SwerveSubsystem.sysIdTranslation/Steer` (records via Phoenix `SignalLogger`) |
| `IMU` / `Pigeon` / `NavX` | 🔷 | Pigeon 2 owned by CTRE `SwerveDrivetrain` (`getPigeon2()`); NavX unused/dropped |
| `SoftLimit` | ✅ | Device-side soft limits in `MechanismConfig.softLimits`; dynamic limits (hood-in-trench) clamped at the subsystem (see `Shooter.setHoodAngle`) |
| `ContinuousSoftLimit` (turret wrap) | ✅ | `MechanismConfig.continuousWrap()` — `PositionalMechanism` wraps goals to the nearest legal equivalent |

## Mechanisms

| v1 | Status | v2 answer |
|---|---|---|
| `Mechanism` (base) | ✅ | `mechanisms.Mechanism` — plus verified config, alerts, telemetry, sim, async brake/coast |
| `Arm` | ✅ | `PositionalMechanism` with `Gains.withGravity(kG, Arm_Cosine)` + MotionMagic |
| `FlyWheel` | ✅ | `VelocityMechanism` (plain or MotionMagicVelocity-profiled) |
| `Turret` | ✅ | `PositionalMechanism` with `continuousWrap()` |
| `LinearExtension` | ⛔ | Unused by the robot; when needed, `PositionalMechanism` in rotations + a meters-per-rotation constant (a typed `LinearMechanism` is a noted follow-up) |
| `Mass` (gravity model) | 🔷 | Phoenix `kG` + `GravityType` handles gravity onboard |

## Swerve (v1 `Swerve` → v2 `SwerveSubsystem`, method by method)

| v1 method | v2 equivalent |
|---|---|
| `driveCommand(vel, omega, fieldOriented)` | `fieldCentricDriveCommand(...)` / `robotCentricDriveCommand(...)` |
| `driveRobotRelativeChassisSpeeds(speeds)` | `driveRobotRelative(speeds)` |
| `turnToAngleCommand(vel, angle)` | `headingLockedDriveCommand(x, y, targetHeading, ...)` |
| `pidToPoseCommand(pose)` | `driveToPoseCommand(...)` (profiled x/y/heading, finish tolerance) |
| `driveToPoseCommand(pose)` (pathfinding) | `pathfindToPoseCommand(pose, constraints)` |
| `driveToPoseWithOverrideCommand(...)` | `pathfindWithOverrideCommand(...)` |
| `pathfindThenFollowPathCommand(name)` | `pathfindThenFollowPathCommand(name, constraints)` (missing-file Alert kept) |
| `resetAngleCommand()` | `resetHeadingCommand()` |
| `resetOdometry(Command)` | `resetPose(...)` / `resetPoseCommand(...)` |
| `coastCommand()` | `coastCommand()` (all modules coast while held) |
| `stopCommand()` | `brakeCommand()` (X-stance) / `idleCommand()` |
| `getPose2D` / `getRotation2D` / `getRobotRelativeSpeeds` / `getVelocity` | `getPose()` / `getHeading()` / `getSpeeds()` / `getVelocityMagnitude()` |
| `getAccelerationDistance()` | `getPigeon2().getAccelerationX/Y()` (device signals) |
| `updateOdometry()` / `Odometry` / `PoseEstimator` | 🔷 CTRE 250 Hz odometry thread + built-in pose estimator |
| vision fusion in `periodic()` (MegaTag**1**) | MegaTag**2**: `withLimelight(...)` + `withVisionPoseTransform(...)` + scaled std-devs |
| `sawTagRecently()` | `sawTagRecently(seconds)` trigger |
| `initAutoBuilder()` | `configureAutoBuilder(translationGains, rotationGains)` (+ wheel-force feedforwards) |
| `initElastic()` (Field2d + swerve widget) | `Field2d` published automatically; module states in DogLog/AdvantageScope |
| `driveSysId` / `angleSysId` (per module) | `sysIdTranslation(...)` / `sysIdSteer(...)` (whole-drivetrain CTRE routines) |
| `SwerveModule` / `ModulesHolder` / `SwerveAccUtils` | 🔷 CTRE `SwerveModule` internals (SwerveAccUtils' call site was already commented out in v1) |
| `VisionMeasurementValidator` | ✅ reject gate + std-dev scaling in `LimelightMegaTag2` |
| `AuroraClient` (slam) | ⛔ dead path in v1 (limelight replaced it) |

## Utilities (`additional_utilities`)

| v1 | Status | v2 answer |
|---|---|---|
| `AllianceUtils` (+ `AlliancePose`, `AllianceTranslation`) | ✅ | `util.AllianceFlip` (+ nested `AlliancePose`, `AllianceTranslation`); field dims injected via `AllianceFlip.configure` — robot code switched |
| `Alliance` | ✅ | `AllianceFlip.isBlue()` |
| `LEDs` (+ `Color`) | ✅ | `util.LedStrip` on WPILib `LEDPattern` (solid/blink/rainbow/gradients built in) — robot code switched |
| `PerformanceMetricsTracker` | ✅ | `telemetry.LoopTimer` (loop time/overruns → DogLog) + `DogLog.setPdh` (power) — robot code switched |
| `CANHealthMonitor` | ✅ | `FaultReporter` device-fault scan + RIO CAN utilization/error logging + Alert (v1's monitor was constructed but its update was commented out) |
| `RobotDiagnostics` | ✅ | `FaultReporter` + mechanism disconnect Alerts + DogLog (v1's was commented out) |
| `LoggablePS5Controller` | 🔷 | WPILib `CommandPS5Controller` + the existing disconnect Alert (per-button logging dropped — DS log already captures joystick data via `withCaptureDs`) |
| `ControllerStateTracker` | 🔷 | Same — disconnect Alert + DS capture |
| `Elastic` (notifications) | 🔷 | WPILib `Alert`s (Elastic renders them natively) |
| `AutoChooserManager` | ✅ | `auto.Autos` (named-command registration + safe chooser) |
| `DoubleClickClient`, `DoubleKeyMap`, `Position` | ⛔ | Unused by robot code; WPILib `Trigger` combinators cover double-click patterns |

## Math & commands

| v1 | Status | v2 answer |
|---|---|---|
| `Vector2D` | 🔷 | WPILib `Translation2d` (same operations) |
| `EMAFilter` | 🔷 | WPILib `LinearFilter.singlePoleIIR` (Shooter's readiness filter keeps v1's exact inline EMA for parity) |
| `Circle`, `Line`, `MathUtils` | ⛔ | Unused by robot code; WPILib `MathUtil` + geometry classes cover the use cases |
| `PeriodicScheduler` / `PeriodicTask` | 🔷 | WPILib `TimedRobot.addPeriodic(...)`; still running in `Robot` for legacy v1 code until v1 removal |
| `CommandMutex`, `CommandUtils`, `ContinuouslyConditionalCommand`, `MapCommand` | ⛔ | Unused; WPILib `Commands.either/select/defer` + `Trigger` cover them |

## Robot-side leftovers still on v1 (removed together with v1, after approval)

- `Robot`'s `PeriodicScheduler` + `TalonFXMotor.refreshAll()` calls (harmless; nothing registers anymore except legacy paths)
- Monologue (`RobotContainer implements Logged`, `Monologue.setupMonologue/updateAll`)
