# Phase 1 — Research: FRC Team Libraries

> **Date:** 2026-07-02 · **Branch:** `feature/lib-rebuild`
> **Goal:** mine open-source FRC libraries for patterns worth adopting in the ExcaLib rebuild — efficient/smooth motion, current & voltage regulation, clean subsystem templates, state machines, telemetry, sim, and auto — and mark what to deliberately avoid.
> **Constraint reminder:** 🚫 no AdvantageKit. Ideas from AK-based repos are translated to our stack (Phoenix 6 native + DogLog/Epilogue + SignalLogger + Phoenix sim/maple-sim). See §11.

---

## 1. 5990 TRIGON — `Programming-TRIGON/TRIGONLib` + `RobotTemplate` *(named team, AK-based)*

**Architecture.** A standalone library (`frc.trigon.lib`) of *named hardware wrappers* over an IO layer: `TalonFXMotor` → `TalonFXIO` → `RealTalonFXIO` / `SimulationTalonFXIO`. Every device type (TalonFX, TalonFXS, CANcoder, Pigeon2, Spark, servo, LaserCAN, simple sensors) gets the same treatment. The robot template adds a `MotorSubsystem` base class and per-domain modules (pose estimation, object detection, simulated field).

**Notable techniques** (files under `TRIGONLib/`):

| Technique | File | Takeaway |
|---|---|---|
| **Dual real/sim configuration** — `applyConfigurations(realConfig, simConfig)`; sim gains can differ from real gains | `hardware/phoenix6/talonfx/TalonFXMotor.java` | Adopt: sim rarely matches real gains; making both first-class avoids hacks |
| **Signal registry split by bus** — static RIO vs CANivore arrays, one `refreshAll` per bus per loop; threaded high-rate signal queues for odometry | `hardware/phoenix6/Phoenix6Inputs.java`, `Phoenix6SignalThread.java` | Same idea our `TalonFXMotor.refreshAll()` already has; keep + formalize signal-per-frequency registration |
| Signal frequencies forced to sane values in sim (high-rate threads slow the sim) | `Phoenix6Inputs.registerSignal` | Adopt detail |
| **Physics-sim attachment** — motor takes a `MotorPhysicsSimulation` (flywheel/arm/elevator WPILib sims) that drives `TalonFXSimState` | `hardware/simulation/*` | Adopt: this is the Phoenix-6-sim-without-AK recipe |
| `MotorSubsystem` base — auto stop-on-disable, async brake-mode executor (brake config calls are blocking!), SysId plumbing, registered-subsystem `forEach` | `RobotTemplate .../subsystems/MotorSubsystem.java` | Adopt shape; the async brake executor is a subtle real-world fix |
| Utility commands: `GearRatioCalculationCommand`, `WheelRadiusCharacterizationCommand`, `NetworkTablesCommand` (live tuning) | `commands/` | Adapt: cheap, high-value calibration tools |

**AK coupling:** `Logger.processInputs`/`LogTable` inside the inputs classes; replay branches. Translation: keep the Real/Sim IO split, replace `LogTable` with direct DogLog/Epilogue logging, drop replay branches entirely.

**License:** ⚠️ **no license file** → we may **not** copy code. Reimplement from the concepts only.

---

## 2. 5951 Makers Assemble — `MA5951/Rebuilt2026` (MAutils) *(named team)*

`MA5951/CustomCANDevice` is an empty placeholder ("Will be more soon…"). Their real shared library is **`com.MAutils`**, embedded in their season repo (they also play REBUILT — same game as us).

**Architecture.** The closest existing embodiment of what this rebuild wants: **declarative subsystem archetypes** + a state layer.

| Technique | File | Takeaway |
|---|---|---|
| **Mechanism archetypes**: `PowerControlledSystem` → `VelocityControlledSystem` / `PositionControlledSystem`, each an abstract `StateSubsystem` wired to an IO (`PositionIOReal`/`Replay`) | `Subsystems/DeafultSubsystems/Systems/*` | Adopt the *taxonomy* (power/velocity/position); their implementation has self-admitted flaws (TODOs: extra IO instances, awkward inheritance chain) — design ours cleaner |
| **Typed per-archetype constants**: `PositionSystemConstants` = min/max/start pose, tolerance, real+sim `GainConfig`, `IS_MOTION_MAGIC`, cruise/accel/jerk, mass, continuity — built via builder | `Subsystems/DeafultSubsystems/Constants/PositionSystemConstants.java` | **Adopt strongly** — this is "add a subsystem = fill in a config object" |
| `StateSubsystem` + `State` (name + `onStateSet` runnable + owning subsystem), `SystemMode` (automatic/manual), per-subsystem self-test (`SelfSystemTest`) | `RobotControl/StateSubsystem.java`, `State.java` | Adapt: good skeleton but **no transition table, no guards** — we need more (see 254/1678) |
| `DeafultSuperStructure` — zone helpers (`isRobotIn(Rectangle2d)`), `hasGamePiece()` abstraction | `RobotControl/DeafultSuperStructure.java` | Adapt zone-helper idea (matches our alliance-zone triggers) |
| `StatusSignalsRunner` (centralized signal refresh), `CanConfigServer` | `CanBus/`, `Components/` | Same family as our refreshAll |
| Swerve controllers as small strategy classes (`FieldCentricDrive`, `AngleAdjustController`, `GamePieceAssistController`, `SkidDetector`, `CollisionDetector`) | `Swerve/Controllers/*`, `Swerve/Utils/*` | Adapt: skid/collision detection are nice AdvantageScope-visible diagnostics |
| `SimulationManager` + `Simulatable` registry; game-piece & vision world sim | `Simulation/*` | Confirms the "central sim registry" pattern (pairs with maple-sim) |

**License:** ⚠️ **NOASSERTION** (no license) → concept-only, no copying.

---

## 3. 6328 Mechanical Advantage — `RobotCode2026Public` *(AK originators)*

**Architecture.** Season code (also REBUILT — directly comparable subsystems: hopper, kicker, flywheel, hood, hub counter, trench bounds). Subsystems are thin; hardware behind IO interfaces; heavy use of generic reusable subsystems.

| Technique | File | Takeaway |
|---|---|---|
| **`RollerSystem`** — one generic class reused for every roller (hopper/kicker/intake): open-loop volts, closed-loop velocity + kS/kV FF, **disconnect detection with `Debouncer` + WPILib `Alert`**, coast-override, brake-on-disable | `subsystems/rollers/RollerSystem.java` | **Adopt** — the template for our `RollerMechanism`, incl. the falling-debounced disconnect alert |
| **`energy` package** — `BatteryEstimator`, `BreakerModel`, `CurrentLimits` (named per-mechanism amp budget in one file), `FinanceDepartment` (subsystems *report* current usage; central allocator adjusts drive limits, incl. `driveProbeRateBrownout` amps/sec ramp) | `energy/*` | **Adapt** — the most concrete brownout-mitigation strategy we found: one named table of per-mechanism current budgets + dynamic drive-current throttling |
| `LoggedTunableNumber` / `LoggedTunableBoolean` — NT-tunable constants with change-detection (`ifChanged`) | (via 3061-lib `frc/lib/team6328/util/`) | **Adopt (translated)** — the standard live-tuning primitive; re-express over plain NT/DogLog |
| `LaunchCalculator`, `HubCounter` | `subsystems/launcher/`, `subsystems/hubcounter/` | Reference for our shooter math & ball counting during migration (same game) |
| `DriveToPose`, drive commands as factories | `commands/` | Reference for move-to-pose |

**AK coupling:** total (Idun/AK generation, replay). Translation: take the *generic-subsystem* and *energy-budget* designs; log with DogLog; skip inputs-autolog entirely.

**License:** MIT-style (Littleton Robotics). OK to adapt with credit.

---

## 4. 3061 Huskie Robotics — `HuskieRobotics/3061-lib` *(AK-based)*

**Architecture.** A curated multi-team lib: `frc/lib/team3061` (their HAL) + vendored `frc/lib/team254` + `frc/lib/team6328` utilities. Strong on *operational robustness*.

| Technique | File | Takeaway |
|---|---|---|
| **`Phoenix6Util.checkErrorAndRetry`** (from 254) — apply config, retry N times, surface a WPILib `Alert` on failure | `frc/lib/team254/Phoenix6Util.java` | **Adopt** — exactly the config apply+verify+retry we're missing (P-07) |
| **`TalonFXConfigEquality` / `CANcoderConfigEquality`** — read back the applied config and compare field-by-field | `frc/lib/team254/TalonFXConfigEquality.java` | **Adopt** — the "verify" half of apply+verify |
| **`FaultReporter` + `SelfChecking*`** — periodic device fault scan (Phoenix motors, CANcoder, Pigeon2, Spark) reporting `SubsystemFault`s to dashboard | `frc/lib/team3015/subsystem/*` | **Adopt** — device fault handling & alerts checklist item, ready-made pattern |
| **`RobotConfig` + `configs/*RobotConfig`** — per-robot (comp/practice/board/sim) config classes selected at startup | `frc/lib/team3061/RobotConfig.java`, `frc/robot/configs/` | **Adopt** — per-robot config-as-code |
| `CurrentSpikeDetector` (254) | `frc/lib/team254/CurrentSpikeDetector.java` | Adopt for roller game-piece detection |
| Torque-current SysId variants for CTRE swerve (`SysIdSwerveTranslation_Torque`) | `frc/lib/team3061/swerve_drivetrain/` | Adapt: SysId under TorqueCurrentFOC needs these custom routines |
| `SysIdRoutineChooser`, `LoggedTracer` (loop-time tracing) | `frc/lib/team3061/util/`, `team6328/util/` | Nice-to-have |
| Swerve on **CTRE `SwerveDrivetrain`** via `SwerveDrivetrainIOCTRE` | `frc/lib/team3061/swerve_drivetrain/SwerveDrivetrainIOCTRE.java` | Supports the "build on CTRE swerve API" decision |

**License:** MIT. OK to adapt with credit (their docs are also good tuning references).

---

## 5. 254 Cheesy Poofs — `Team254/FRC-2025-Public`

**Architecture.** Command-based (they moved off their old loop system), subsystems split *per mechanism* (e.g. `IntakePivotSubsystem` + `IntakeRollerSubsystem`), hardware/sensor IO split (`ClawSensorIO{Hardware,Sim}`), subsystem *factories* per mechanism, `RobotState` as the single estimator/state hub, maple-sim for drive (`MapleSimSwerveDrivetrain`).

| Technique | File | Takeaway |
|---|---|---|
| **Graph state machine**: `SuperstructureStateMachine` = states + `StateTransition(from, to, commandSupplier, transitionTime, collision)` edges; **precomputed all-pairs paths**; per-transition costs loadable from a deploy file; collision-flagged edges | `subsystems/superstructure/SuperstructureStateMachine.java`, `StateTransition.java` | **Adapt (simplified)** — the transition-table-with-guards model is our state-machine engine core; skip A*/path precomputation (their elevator/wrist needed it; our turret bot doesn't) |
| `ModalSuperstructureTriggers` — driver "modes" changing what buttons request | `superstructure/ModalSuperstructureTriggers.java` | Reference for trigger-driven goal selection |
| Separate pivot/roller subsystems rather than one mega-subsystem | `subsystems/intake/*` | Adopt: pairs perfectly with mechanism archetypes |
| `CANDeviceId` (id + bus name as one value type) | `lib/drivers/CANDeviceId.java` | Adopt tiny but useful |
| Latency-compensated `RobotState` estimator; Megatag pose classes (`MegatagPoseEstimate`) | `RobotState.java`, `subsystems/vision/*` | Reference for vision fusion structure |
| Config-equality + retry utils (vendored into 3061-lib, §4) | — | Adopt via §4 |

**License:** MIT. OK to adapt with credit.

---

## 6. 1678 Citrus Circuits — `frc1678/C2023-Public`

**Architecture.** Their signature: the **Request framework** — a superstructure queue of `Request` objects (`act()` + `isFinished()`), composable via `ParallelRequest`/`SequentialRequest`, with `Prerequisite` guards; subsystems built on **`ServoMotorSubsystem`** (their descendant of 254's classic template: typed constants block, position/motion-magic modes, homing).

| Technique | File | Takeaway |
|---|---|---|
| `Request` + `Prerequisite` — sequencing with explicit *wait-until-safe* guards | `lib/requests/*` | Concept feeds our guard model; but WPILib `Command` + `Trigger` already covers 90% — **avoid a parallel command system**, express guards as `BooleanSupplier`s in the transition table instead |
| `ServoMotorSubsystem` — declare-constants-and-go positional subsystem (soft limits, homing, profiles) | `subsystems/ServoMotorSubsystem.java` | Adopt spirit: same lineage as our `PositionalMechanism` archetype |
| `SuperstructureGoal` — one value object holding all mechanism setpoints | `states/SuperstructureGoal.java` | **Adopt** — superstructure state = record of per-mechanism setpoints, fan-out happens in one place |

**License:** MIT. OK to adapt with credit.

---

## 7. CTRE — `CrossTheRoadElec/Phoenix6-Examples` *(mandatory reference)*

Examples inventoried: `MotionMagic`, `MotionMagicExpo`, `PositionClosedLoop`, `VelocityClosedLoop`, `CurrentLimits`, `ControlRequestLimits`, `FusedCANcoder`, `PhoenixSysId`, `Simulation`, `BasicLatencyCompensation`, `WaitForAll`, `SwerveWithPathPlanner`, `SwerveWithChoreo`, `CANdi/CANrange/CANdle/Pigeon2`.

Canonical idioms to bake into the control layer (from `java/MotionMagicExpo/.../Robot.java` and friends):

- **Configure once via one `TalonFXConfiguration`** object (not per-feature apply calls like our current `TalonFXMotor`).
- `Feedback.SensorToMechanismRatio` (and `RotorToSensorRatio` + `FeedbackSensorSource` for FusedCANcoder) so **the device itself works in mechanism units** — this eliminates our RIO-side conversion-factor pattern (P-10) entirely.
- **MotionMagicExpo**: profile shaped by `MotionMagicExpo_kV` / `MotionMagicExpo_kA` (always in Volts, even for the TorqueCurrentFOC request variant); `MotionMagicCruiseVelocity` optionally caps it; time-optimal & smoother than trapezoidal — prefer for large moves; plain MotionMagic when constant-cruise behavior is wanted.
- Control requests are **pre-allocated and mutated** (`m_mmReq.withPosition(...)`), matching what our motor wrapper already does.
- `ControlRequestLimits` example → per-request current/voltage limit overrides.
- `BasicLatencyCompensation` → `BaseStatusSignal.getLatencyCompensatedValue(pos, vel)`.
- `WaitForAll` → `BaseStatusSignal.waitForAll(timeout, signals)` for time-synced CANivore reads.
- `Simulation` → `TalonFXSimState` (supply voltage, raw rotor position/velocity feed from a WPILib physics model) — pairs with TRIGON's §1 physics-sim attachment and maple-sim.
- `SwerveWithPathPlanner` → Tuner X `TunerConstants` + `SwerveDrivetrain` + `AutoBuilder` wiring, the canonical shape for our new swerve.

**License:** repo has no SPDX-detected license; example files carry the **WPILib BSD header**. Treat as reference idioms; re-express in our own code, credit CTRE.

---

## 8. DogLog — `jonahsnider/doglog`

- API: static `DogLog.log("Key", value)` (+ units overloads), `DogLog.setOptions(new DogLogOptions()...)` (NT publish vs datalog-only, DS capture), `DogLog.setPdh(...)` for automatic PDH channel logging. Tunables supported via `DogLog.tunable`.
- Zero-ceremony, no annotation processor, logs to WPILib DataLog + optional NT → AdvantageScope-viewable. Works alongside Phoenix `SignalLogger` (hoot for high-rate device data; DataLog for robot logic).
- License: repo API metadata shows none-detected, but the project is distributed as a standard FRC vendordep for team use — **we consume it as a dependency, not copied code**, so licensing is a non-issue. Verify LICENSE file when adding the vendordep.

## 9. WPILib Epilogue

- `@Logged` on classes/fields/methods + `Epilogue.bind(this)`; compile-time codegen, no reflection; backends: NT or file; importance filtering; `@NotLogged` opt-outs.
- Limitations: static structure only, NPE-sensitive, can add CAN-read cost if pointed at device getters (mitigated by our batched-signal caching).
- Part of WPILib (BSD) — free to use.
- **Assessment vs DogLog:** Epilogue shines for auto-logging whole subsystem objects; DogLog shines for explicit, cheap, imperative logs and tunables. They coexist fine. Recommendation in §10.

## 10. 5516 Iron Maple — `Shenzhen-Robotics-Alliance/maple-sim`

- dyn4j 2D rigid-body physics for the whole field: robot-field collisions, game pieces, intake interaction, opponent robots. Distributed as a vendordep; **BSD-3-Clause**.
- Integrates with CTRE swerve (Team 254 uses `MapleSimSwerveDrivetrain` wrapping the CTRE `SwerveDrivetrain` sim — see §5) and with per-mechanism `TalonFXSimState` feeds.
- Takeaway: use maple-sim for drivetrain + game-piece world; use plain WPILib physics sims (flywheel/arm/elevator via `TalonFXSimState`) for mechanisms.

---

## 11. Translate-away-from-AdvantageKit ledger

| AK-coupled idea (source) | AK part | Our re-expression |
|---|---|---|
| Hardware IO split Real/Sim (TRIGON, 3061, 254) | `@AutoLog` inputs classes, replay determinism | Keep **Real/Sim swap only** — a `MechanismIO` chosen at construction (`RobotBase.isSimulation()`); inputs are plain fields logged via DogLog; **no replay contract** |
| `Logger.processInputs` (all AK teams) | AK log tables | `DogLog.log(path, value)` on cached signal values, once per loop |
| `LoggedTunableNumber` (6328) | AK NT input classes | Same API shape over `DoubleSubscriber`/DogLog tunable, with `ifChanged` hash |
| Threaded odometry signal queues (TRIGON `Phoenix6SignalThread`, MAutils `PhoenixOdometryThread`) | AK timestamp locks | CTRE `SwerveDrivetrain` already runs a 250 Hz odometry thread internally — adopt CTRE swerve and we get this for free; don't rebuild it |
| Replay-driven sim validation (all) | AK replay | Replaced by maple-sim interactive sim + hoot log review in AdvantageScope |

---

## 12. License summary

| Source | License | Usage rule |
|---|---|---|
| 254 FRC-2025-Public | MIT | Adapt with attribution |
| 6328 RobotCode2026Public | MIT-style | Adapt with attribution |
| 3061-lib | MIT | Adapt with attribution (already aggregates 254/6328 with credit — good model for our NOTICE file) |
| 1678 C2023-Public | MIT | Adapt with attribution |
| **TRIGONLib** | **none** | **Concepts only — no code copying** |
| **MAutils (MA5951)** | **none (NOASSERTION)** | **Concepts only — no code copying** |
| CTRE Phoenix6-Examples | WPILib BSD headers (no repo SPDX) | Reference idioms; re-express |
| maple-sim | BSD-3-Clause | Use as vendordep |
| DogLog | vendordep dependency | Use as dependency; verify LICENSE on adoption |
| WPILib / Epilogue | BSD | Use freely |

Action: create a `NOTICE`/`CREDITS` file in the new library crediting 254, 6328, 3061, 1678, TRIGON, MA5951, CTRE, Iron Maple for the patterns adapted.

---

## 13. Consolidated recommendations

| # | Idea | Source | Verdict | Priority | Notes |
|---|---|---|---|---|---|
| R-01 | One-shot `TalonFXConfiguration` + `checkErrorAndRetry` + config read-back equality check | 254 via 3061, CTRE | **Adopt** | 🟥 | Fixes P-05/P-07; foundation of the new motor layer |
| R-02 | `SensorToMechanismRatio`/`RotorToSensorRatio` + FusedCANcoder → device-side mechanism units; kill RIO conversion factors | CTRE | **Adopt** | 🟥 | Eliminates P-10's worst failure mode |
| R-03 | Onboard MotionMagic / **MotionMagicExpo** / MotionMagicVelocity with TorqueCurrentFOC request variants + Voltage fallback, behind a small control-request abstraction | CTRE | **Adopt** | 🟥 | The control-quality core; Expo default for positional moves |
| R-04 | Mechanism archetype taxonomy: positional / velocity(flywheel) / roller (+ swerve) with **typed per-archetype config objects** (gains real+sim, limits, MM params, tolerance) | MAutils, 1678 ServoMotorSubsystem, 6328 RollerSystem | **Adopt** | 🟥 | The "declare, don't write" template layer |
| R-05 | Generic `RollerSystem` incl. disconnect Alert w/ debounce, current-spike game-piece detection | 6328, 254 `CurrentSpikeDetector` | **Adopt** | 🟥 | Direct template for intake/transport |
| R-06 | State-machine engine: enum states + transition table with **guards** (`BooleanSupplier`), onEnter/whileIn/onExit, command-based integration | 254 `StateTransition` (simplified), MAutils `StateSubsystem` | **Adapt** | 🟥 | Skip 254's A*/all-pairs precompute — overkill for our topology |
| R-07 | Superstructure goal = value object of per-mechanism setpoints; single fan-out; interlocks as transition guards | 1678 `SuperstructureGoal`, 254 | **Adopt** | 🟥 | Replaces P-09's enum-field mutation |
| R-08 | Real/Sim hardware swap via IO interface **without replay** | TRIGON, 254, 3061 (translated §11) | **Adopt** | 🟥 | Enables sim parity (P-03) |
| R-09 | CTRE `SwerveDrivetrain` + Tuner X `TunerConstants` as the swerve base (250 Hz odometry, sim support built in) | CTRE, 3061, 254 | **Adopt** | 🟥 | Replaces hand-rolled ModulesHolder stack |
| R-10 | Per-mechanism named current budget table + supply/stator/lower-limit discipline; dynamic drive-limit adjustment for brownout | 6328 `energy` | **Adapt** | 🟧 | Start with the static named table + correct `CurrentLimitsConfigs`; dynamic allocator as stretch |
| R-11 | `FaultReporter`/self-check scan → dashboard Alerts | 3061 | **Adopt** | 🟧 | Replaces dead `CANHealthMonitor`/`RobotDiagnostics` |
| R-12 | `LoggedTunableNumber`-style live-tunable constants (translated) | 6328 | **Adopt** | 🟧 | Replaces raw NT `Tab1` tuning entries |
| R-13 | Per-robot config classes (comp/practice/sim) | 3061 `RobotConfig` | **Adapt** | 🟧 | Lighter version: one interface + per-robot implementations |
| R-14 | maple-sim for drive/field/game pieces + `TalonFXSimState` physics for mechanisms | Iron Maple, 254, TRIGON | **Adopt** | 🟧 | Vendordep; wire through the IO sim implementations |
| R-15 | DogLog as log-of-record + Phoenix `SignalLogger` for device-rate data; Epilogue optionally for whole-object convenience | DogLog, WPILib, CTRE | **Adopt** | 🟧 | Recommendation: **DogLog + SignalLogger**, skip Epilogue initially (one system to learn; annotations can come later) |
| R-16 | `CANDeviceId` value type (id+bus) | 254 | Adopt | 🟨 | Tiny; prevents bus mixups |
| R-17 | Calibration commands: wheel-radius characterization, gear-ratio calculation | TRIGON (concept) | Adapt | 🟨 | Rewrite from concept (no license) |
| R-18 | Torque-current SysId routines | 3061 | Adapt | 🟨 | Needed because standard SysId assumes volts |
| R-19 | Async brake-mode setter (blocking CTRE call off the main loop) | TRIGON (concept) | Adapt | 🟨 | Small but real loop-overrun fix |
| R-20 | Skid/collision detectors for swerve | MAutils (concept) | Avoid for now | 🟩 | Nice diagnostics; not core; revisit post-season |
| R-21 | 1678 Request framework as execution engine | 1678 | **Avoid** | — | Parallel command system fights WPILib command-based; use Commands+Triggers |
| R-22 | 254 A*-planned superstructure paths, transition-cost files | 254 | **Avoid** | — | Complexity for articulated-collision robots; our mechanisms are near-independent |
| R-23 | AdvantageKit replay & @AutoLog anywhere | 6328 et al. | **Avoid** | — | Hard constraint |

---

## 14. Top recommendations → hand-off to design

The 🟥 set forms a coherent architecture, in dependency order:

1. **Motor/config layer** (R-01, R-02, R-03, R-16): one `TalonFXConfiguration` per device built from typed config records, applied with retry + read-back verify, mechanism units on-device, pre-allocated FOC control requests with Voltage fallback.
2. **Mechanism archetypes** (R-04, R-05): `PositionalMechanism`, `VelocityMechanism`, `RollerMechanism` — each = config record + IO (real/sim, R-08) + at-setpoint triggers + SysId hooks + DogLog telemetry (R-15).
3. **State-machine engine** (R-06): enum states, transition table, guards, enter/while/exit actions, `Trigger`-friendly, non-blocking.
4. **Superstructure coordinator** (R-07): robot-goal states fan out to mechanism setpoints; interlocks are guards; illegal combinations unrepresentable.
5. **Swerve** (R-09): CTRE `SwerveDrivetrain` + TunerConstants + MegaTag2 fusion + PathPlanner; maple-sim world (R-14).
6. **Robustness rails** (R-10, R-11, R-12, R-13): current budget table, fault reporter, tunables, per-robot configs.

Proceeding to Phase 2: `DESIGN.md`.
