# NOTICE — Credits & Inspirations

ExcaLib v2 (`frc.excalib2`) was written from scratch by FRC Team 6738 Excalibur, but many
of its patterns were learned from other teams' open-source work. No code was copied; the
concepts below were re-implemented in our own style. Thank you all.

| Pattern in ExcaLib v2 | Learned from | License of source |
|---|---|---|
| Config apply + read-back verify + retry (`DeviceConfigs`) | Team 254 `Phoenix6Util` / `TalonFXConfigEquality` (via 3061-lib) | MIT |
| `CANDeviceId` value type | Team 254 | MIT |
| Transition-table state machine with guards (`StateMachine`) | Team 254 superstructure (simplified), MA5951 `StateSubsystem` (concept) | MIT / none (concept only) |
| Superstructure goal = per-mechanism setpoint tuple | Team 1678 `SuperstructureGoal` | MIT |
| Generic roller with disconnect Alert + current-spike detection | Team 6328 `RollerSystem`, Team 254 `CurrentSpikeDetector` | MIT |
| Named per-mechanism current budget table | Team 6328 `energy` package | MIT |
| NT-tunable numbers with change detection (`TunableNumber`) | Team 6328 `LoggedTunableNumber` (re-expressed without AdvantageKit) | MIT |
| Device fault scanning → Alerts (`FaultReporter`) | Team 3061 `FaultReporter` / self-check | MIT |
| Real/sim dual gains, physics-sim attachment, async brake mode | Team 5990 TRIGON (concepts only — no license on source) | none (concept only) |
| Mechanism archetype taxonomy (position/velocity/roller) | MA5951 MAutils, Team 1678 `ServoMotorSubsystem` (concepts) | none / MIT |
| Swerve on CTRE `SwerveDrivetrain` + PathPlanner wiring | CTRE Phoenix6-Examples (`SwerveWithPathPlanner`), 3061-lib, Team 254 | WPILib BSD headers / MIT |
| MotionMagic / MotionMagicExpo / current-limit idioms | CTRE Phoenix6-Examples | WPILib BSD headers |
| Field-zone triggers | MA5951 (concept) | none (concept only) |

Dependencies: WPILib (BSD), CTRE Phoenix 6, PathPlannerLib, DogLog, maple-sim (BSD-3-Clause),
Monologue (legacy, being phased out).
