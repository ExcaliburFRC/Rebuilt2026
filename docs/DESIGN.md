# Phase 2 — Design: ExcaLib v2

> **Date:** 2026-07-02 · **Branch:** `feature/lib-rebuild` · **Status: ⏸ AWAITING APPROVAL — no implementation code until this design is approved.**
> Inputs: [AUDIT.md](AUDIT.md) (pain points P-01..P-27) · [RESEARCH.md](RESEARCH.md) (recommendations R-01..R-23).
> Fixed stack: WPILib 2026 command-based · Phoenix 6 (26.1.0) FOC/Pro · DogLog + SignalLogger · Phoenix 6 sim + maple-sim · PathPlanner · Limelight MegaTag2 · WPILib Units in public API.

---

## 1. Design goals (traceability)

| Goal | Fixes | Informed by |
|---|---|---|
| Library never imports robot code; robot code only *declares* | P-01, P-05 | all reference teams |
| All closed loops run on the motor controller (FOC, MotionMagic/Expo), RIO only picks setpoints | P-02 | CTRE (R-03), 254, 6328 |
| Every subsystem is declared as config + states, not plumbing | P-06, P-09, P-10 | MAutils, 1678, 6328 (R-04..R-07) |
| Config applied once, verified, retried, alerted | P-05, P-07 | 254/3061 (R-01) |
| Sim parity for every mechanism + drivetrain | P-03 | TRIGON, maple-sim (R-08, R-14) |
| One log of record, viewable in AdvantageScope | P-11 | DogLog + SignalLogger (R-15) |
| Faults & disconnects surface as dashboard Alerts | P-12 (dead monitors) | 3061 (R-11), 6328 (R-05) |

---

## 2. Package layout

New root **`frc.excalib2`** (old `frc.excalib` untouched until Phase 4 migration is proven — see §12 and OD-1).

```
frc.excalib2
├── device/                    # Phoenix 6 device layer
│   ├── CANDeviceId            # record(id, busName)                          [R-16]
│   ├── Motor                  # TalonFX wrapper: signal registration, pre-allocated
│   │                          #   control requests, follower handling
│   │                          #   (fused CANcoder handled in MechanismConfig — no wrapper class)
│   ├── DeviceConfigs          # apply + read-back verify + retry + Alert      [R-01]
│   └── SignalHub              # per-bus signal registry: update freqs, batched
│                              #   refreshAll(), waitForAll on CANivore,
│                              #   latency-compensated position
├── control/
│   ├── ControlMode            # TORQUE_CURRENT_FOC (default) | VOLTAGE | DUTY_CYCLE
│   ├── Gains                  # record: kP kI kD kS kV kA kG + GravityType
│   ├── MotionConstraints      # record: cruise, accel, jerk | expo kV/kA      [R-03]
│   └── CurrentBudget          # record: stator, supply, supplyLower@time,
│                              #   torque peaks fwd/rev                        [R-10]
├── mechanisms/
│   ├── MechanismConfig        # builder: devices, ratios, limits, gains(real+sim),
│   │                          #   budget, tolerance, neutral mode, telemetry name
│   ├── io/
│   │   ├── MechanismIO        # interface: inputs record + control methods
│   │   ├── TalonFXMechanismIO # real hardware
│   │   └── SimMechanismIO     # TalonFXSimState + WPILib physics model        [R-08]
│   ├── Mechanism              # base: telemetry, connect-Alerts (debounced),
│   │                          #   coast/brake helpers (async), SysId hooks
│   ├── PositionalMechanism    # MotionMagic(Expo) to setpoint, soft limits,
│   │                          #   gravity FF, atSetpoint Trigger
│   ├── LinearExtension        # meters-based positional; dynamic arm-mounted
│   │                          #   gravity FF (kG·sin θ) or constant-gravity elevator
│   ├── VelocityMechanism      # onboard velocity (MM-Velocity), atSpeed Trigger
│   └── RollerMechanism        # duty/voltage/current modes, current-spike
│                              #   game-piece detection                        [R-05]
├── statemachine/
│   ├── StateMachine<S>        # enum states, transition table, guards,
│   │                          #   onEnter/whileIn/onExit, Trigger integration [R-06]
│   └── Transition<S>          # record: from, to, guard, action
├── superstructure/
│   └── SuperstructureBase<G>  # goal enum → per-mechanism setpoint fan-out,
│                              #   interlocks = transition guards              [R-07]
├── swerve/
│   ├── SwerveSubsystem        # wraps CTRE SwerveDrivetrain + TunerConstants  [R-09]
│   ├── DriveCommands          # teleop drive, heading lock, DriveToPose
│   └── vision/
│       └── LimelightMegaTag2  # SetRobotOrientation feed + MT2 estimate +
│                              #   dynamic std-devs → addVisionMeasurement
├── auto/
│   └── Autos                  # PathPlanner AutoBuilder glue + chooser helper
├── telemetry/
│   ├── Telemetry              # DogLog conventions + SignalLogger lifecycle   [R-15]
│   ├── TunableNumber          # NT-tunable w/ ifChanged (6328 pattern, no AK) [R-12]
│   └── FaultReporter          # periodic device fault scan → Alerts           [R-11]
├── sim/
│   └── PhysicsSim             # registry of physics models stepped in
│                              #   simulationPeriodic; maple-sim arena setup   [R-14]
└── util/
    ├── AllianceFlip           # alliance-relative poses (port of AllianceUtils)
    └── Zones                  # Rectangle2d field-zone triggers (MAutils idea)
```

**Dependency rule (enforced by review):** `frc.excalib2` never imports `frc.robot`. Robot-specific values enter only through config objects and constructor parameters.

---

## 3. Control layer

### 3.1 Control-request abstraction

Each mechanism pre-allocates **both** FOC and Voltage request families and switches by `ControlMode`:

| Intent | `TORQUE_CURRENT_FOC` (default, Pro) | `VOLTAGE` (sim/debug/fallback) |
|---|---|---|
| Profiled position | `MotionMagicExpoTorqueCurrentFOC` | `MotionMagicExpoVoltage` |
| Profiled position (trapezoid) | `MotionMagicTorqueCurrentFOC` | `MotionMagicVoltage` |
| Velocity | `MotionMagicVelocityTorqueCurrentFOC` | `MotionMagicVelocityVoltage` |
| Raw effort | `TorqueCurrentFOC` | `VoltageOut` |
| Neutral | `NeutralOut` / `CoastOut` | same |

- `ControlMode` set in `MechanismConfig`; **sim builds force `VOLTAGE`** (OD-4).
- `DutyCycleOut` retained for bring-up only.
- Requests are mutated, never re-allocated (zero-GC hot path, as today).

### 3.2 Profiles: MotionMagicExpo as positional default

- **Expo** (`MotionMagicExpo_kV/_kA`, optional cruise cap) = time-optimal, motor-shaped, smoother & more efficient — default for `PositionalMechanism`.
- **Trapezoid** MotionMagic available per-config when constant-cruise is genuinely wanted (e.g. synchronized motion).
- **MotionMagicVelocity** for flywheel spin-up (jerk-limited ramp) instead of RIO-side `TrapezoidProfile` on velocity.

### 3.3 Feedback & units

- `SensorToMechanismRatio` / `RotorToSensorRatio` + `FusedCANcoder` where an absolute encoder exists (hood, turret, intake four-bar) → **device works in mechanism units**; the RIO-side conversion-factor pattern is deleted (P-10).
- Public API takes/returns WPILib `Measure` types (`Angle`, `AngularVelocity`, `Distance`, `Current`, …); internals cache doubles from status signals.

### 3.4 Gains & feedforward

- `Gains` record → `Slot0Configs` incl. `kG` + `GravityType` (`Elevator_Static` vs `Arm_Cosine`) and `StaticFeedforwardSign`.
- Separate **real and sim gain sets** in every config (TRIGON pattern).
- SysId hooks per mechanism, including torque-current SysId variants (R-18).

### 3.5 Current & voltage regulation

- `CurrentBudget` record per mechanism → `CurrentLimitsConfigs` (`StatorCurrentLimit`, `SupplyCurrentLimit` + `SupplyCurrentLowerLimit`/`SupplyCurrentLowerTime`) + `TorqueCurrentConfigs` peaks. All enables set explicitly.
- Robot-side single **named budget table** (6328 pattern): one class listing every mechanism's amps — reviewable at a glance.
- `MotorOutputConfigs` peak forward/reverse voltage where needed; open/closed-loop ramps per config.
- **Brownout mitigation strategy (documented in the lib):** ① correct supply lower-limits everywhere; ② drive stator limit set from budget table; ③ (stretch, post-migration) dynamic drive-current throttle keyed on bus voltage dip, 6328-style.

### 3.6 StatusSignal discipline

- `SignalHub` registers each signal with an explicit frequency tier (fast 100 Hz: pos/vel/closed-loop-ref; medium 50 Hz: currents/volts; slow 4 Hz: temp/faults), batch-refreshes per bus once per loop **before** `CommandScheduler.run()` (fixes P-18), calls `optimizeBusUtilization` after all devices register, and exposes `BaseStatusSignal.getLatencyCompensatedValue` helpers. CANivore time-sync via `waitForAll` for the swerve bus (handled inside CTRE `SwerveDrivetrain` for drive).

### 3.7 Config apply + verify

`DeviceConfigs.applyVerified(device, config)`:
1. `apply` with 100 ms timeout, retry ≤ 5 (254 `checkErrorAndRetry` pattern),
2. read back full configuration, field-compare (254 config-equality pattern),
3. mismatch/failure → WPILib `Alert` (kError) + DogLog fault entry, return status so callers can gate readiness.

---

## 4. Mechanism archetypes (template layer)

Class shape (all extend `Mechanism`, none extend `SubsystemBase` — subsystems *own* mechanisms, 254-style):

```
Mechanism (base)
 ├─ owns: MechanismIO (real|sim), config, alerts, telemetry name
 ├─ periodic(): refresh cached inputs, DogLog.log(...), disconnect Alert (debounced)
 ├─ coast/brake (async executor), stop(), SysId routines
 │
 ├─ PositionalMechanism      setGoal(Angle|Distance) · atGoal Trigger · soft limits
 │                           · MotionMagicExpo default · gravity FF · homing hook
 ├─ VelocityMechanism        setVelocity(AngularVelocity) · atSpeed Trigger
 │                           · MotionMagicVelocity ramp
 └─ RollerMechanism          run(volts|amps|duty) · currentSpike Trigger (debounced)
                             · hasGamePiece detection
```

### "Add a subsystem" target shape (~30 lines)

```java
public class Hood extends SubsystemBase {
    private final PositionalMechanism hood = new PositionalMechanism(
        MechanismConfig.positional("Shooter/Hood")
            .motor(new CANDeviceId(HOOD_MOTOR_ID))
            .fusedCANcoder(new CANDeviceId(HOOD_ENCODER_ID), ROTOR_TO_SENSOR, SENSOR_TO_MECHANISM)
            .gains(REAL_GAINS, SIM_GAINS)                      // Gains records incl. kG, Arm_Cosine
            .motionMagicExpo(EXPO_KV, EXPO_KA)
            .softLimits(Radians.of(0.0), Radians.of(0.55))
            .currentBudget(CurrentBudget.of(40, 30))           // stator, supply
            .tolerance(Radians.of(0.01))
            .simModel(SimModels.singleJointedArm(...)));       // sim parity, one line

    public Command goTo(Supplier<Angle> angle) { return run(() -> hood.setGoal(angle.get())); }
    public Trigger atGoal() { return hood.atGoal(); }
}
```

Everything else — device construction, config apply+verify, signal registration, telemetry, alerts, sim wiring — happens in the base classes. **Success test:** the Phase 4 rewrite of Transport should fit on one screen.

---

## 5. State-machine engine

Typed, enum-based, declarative, command-friendly, non-blocking:

```java
StateMachine<IntakeState> sm = StateMachine.of("Intake", IntakeState.CLOSED)
    .onEnter(OPEN,   () -> fourBar.setGoal(OPEN_ANGLE))
    .whileIn(PUMP,   pumpOscillationCommand())                 // command scheduled while in state
    .onExit(PUMP,    roller::stop)
    .transition(CLOSED, OPEN).when(armHomed)                   // guard: BooleanSupplier
    .transition(OPEN, PUMP)
    .transition(any(), CLOSED)                                 // always-legal escape
    .build();
```

| Element | Semantics |
|---|---|
| **States** | any enum; one machine per subsystem (and one for the superstructure) |
| **Transition table** | explicit `from → to` edges; **default-deny** — an unlisted transition is illegal and a `requestState` for it fails (logged + Alert in dev mode) |
| **Guards** | `BooleanSupplier` per edge; evaluated at request time and (for pending requests) each loop — never blocking |
| **Actions** | `onEnter` (Runnable or Command), `whileIn` (Command scheduled on entry, cancelled on exit), `onExit` (Runnable) |
| **Command integration** | `sm.requestStateCommand(S)` returns a Command (finishes when accepted or optionally when `atGoal`); `whileIn` commands carry the owning subsystem's requirements; edges exposed as `Trigger sm.in(S)`, `sm.entered(S)` |
| **Non-blocking** | machine ticks in the owning subsystem's `periodic()`; pending request + guard model, no busy waits |
| **Telemetry** | current state, pending request, last denied transition — auto-logged via DogLog |

This is 254's transition-table idea minus the A*/cost-file machinery (R-22), plus MAutils' per-subsystem state ownership, expressed with WPILib primitives (no parallel Request framework, R-21).

## 6. Superstructure coordinator

```java
enum RobotGoal {                        // value object per goal (1678 SuperstructureGoal idea)
    IDLE      (IntakeState.CLOSED, ShooterSpec.idle(),           TransportState.IDLE),
    AIM_HUB   (IntakeState.CLOSED, ShooterSpec.track(HUB, HIGH), TransportState.IDLE),
    SHOOT_HUB (IntakeState.PUMP,   ShooterSpec.shoot(HUB, HIGH), TransportState.FEED),
    ...
}
```

- `Superstructure` owns a `StateMachine<RobotGoal>`; entering a goal **fans out** each field to the matching subsystem state machine (one place, no `ParallelCommandGroup`-of-`InstantCommand`s).
- **Interlocks are guards**, e.g.: `FEED` transport edge guarded by `shooter.ready()`; `SHOOT_*` goals guarded by hub-active (`ShiftUtil`) where strategy demands; hood max-angle guard under the trench zone (kept from today, as a soft-limit *supplier* on the hood config); illegal combinations simply have no enum entry — **uncommandable by construction**.
- Zone/shift/button `Trigger`s request goals exactly like today's `initTriggers()`, but each binding is one line: `intakeButton.and(inAllianceZone).and(hubActive).onTrue(superstructure.request(SHOOT_HUB))`.
- Auto named commands = `superstructure.request(goal)` — same mechanism, no divergence between teleop and auto behavior.

## 7. Config-as-code

| Layer | Form |
|---|---|
| Library | immutable records + builders (`Gains`, `MotionConstraints`, `CurrentBudget`, `MechanismConfig`) — all `Measure`-typed |
| Robot | one constants class per subsystem holding **only data** (no hardware objects — fixes P-05); a named current-budget table |
| Per-robot | `RobotVariant` (COMP / PRACTICE / SIM) selected at startup (3061 pattern, lightweight) supplying variant-specific values (offsets, gains) |
| Live tuning | `TunableNumber` (NT-backed, `ifChanged` callbacks) — replaces raw `Tab1` NT entries; disabled at competitions via flag |

## 8. Telemetry

- **DogLog is the log of record** (decision — see OD-3): `DogLog.log("Shooter/Hood/Position", ...)` from mechanism bases automatically; subsystems add domain values. Options: NT publish on in dev, datalog-only at comp; `setPdh` for power logging.
- **Phoenix `SignalLogger`** started on boot → hoot files for high-rate device data; both viewable in AdvantageScope.
- **Epilogue: not initially** — one telemetry system to learn during the rebuild; annotations can be layered later without rework.
- Monologue, SmartDashboard scatter, Shuffleboard tabs, and the `Elastic` notifier are removed with the migration (Elastic dashboards still read NT fine).
- `FaultReporter` scans device faults (sticky faults incl. undervoltage, boot-during-enable) each second → `Alert`s.

## 9. Simulation

| Piece | Approach |
|---|---|
| Mechanisms | `SimMechanismIO` = same control requests against `TalonFXSimState`, stepped by a WPILib physics model (`DCMotorSim` / `SingleJointedArmSim` / `ElevatorSim` / `FlywheelSim`) chosen in config (`.simModel(...)`) |
| Drivetrain | CTRE `SwerveDrivetrain` sim + **maple-sim** arena (robot collisions, game pieces, REBUILT field) — 254's `MapleSimSwerveDrivetrain` shape |
| Vision | sim pose feed for `LimelightMegaTag2` (ground-truth + noise) so pose-fusion code paths run in sim |
| Selection | `RobotBase.isSimulation()` at construction — **no replay machinery** |
| Game logic | `ShiftUtil` test hooks already allow fake game data in sim |

## 10. Swerve & vision

- **CTRE `SwerveDrivetrain` + `TunerConstants`** (R-09): 250 Hz odometry thread, time-synced CANivore reads, built-in sim, `SwerveRequest` API (incl. FOC drive). TunerConstants hand-authored from current measurements first, verified on-robot (OD-5).
- **MegaTag2**: feed `SetRobotOrientation` each loop (turret-offset-corrected heading — port of `TurretOffsetGetter`, now a library-friendly injected supplier); consume `getBotPoseEstimate_wpiBlue_MegaTag2`; dynamic std-devs by distance/tag-count; keep the "reject while turret/robot spins fast" gate.
- **Move-to-pose**: `DriveToPose` command with real gains (fixes P-13) + PathPlanner `pathfindToPose` for long moves.
- **PathPlanner** stays (OD-6): `AutoBuilder` on the new subsystem, named commands from superstructure goals (fixes P-04), auto chooser preserved.

## 11. Robot loop (target)

```
robotPeriodic():
  SignalHub.refreshAll()            // fresh signals BEFORE commands (fixes P-18)
  ShiftUtil.update()
  CommandScheduler.run()            // subsystem periodics: mechanisms log + state machines tick
  Telemetry.flush()                 // cheap; DogLog writes as it goes
```
(Thread priority raised once at boot to a modest value, not 99 every loop — P-15.)

## 12. Migration strategy (Phase 4 preview)

| Stage | Content | Gate |
|---|---|---|
| A | Library lands (`frc.excalib2`), zero robot-code changes | build + sim green → **PR 1** |
| B1 | **Transport** → `VelocityMechanism`×2 + state machine (smallest, proves template) | behavior parity in sim |
| B2 | **Intake** → `PositionalMechanism` (four-bar, FusedCANcoder — fixes P-14) + `RollerMechanism` | parity |
| B3 | **Shooter** → turret/hood `PositionalMechanism` + flywheel `VelocityMechanism`; interpolation maps, shot-lead loop, ball counter ported **verbatim** | parity incl. `shooterReady` semantics |
| B4 | **Superstructure** → `RobotGoal` machine 1:1 with today's `RobotState` entries (OD-7) | all triggers/auto parity |
| B5 | **Swerve** → CTRE drivetrain + MT2 (riskiest; last; needs robot time for offsets/gains verification) | drive parity in sim + bench |
| C | Delete old `frc.excalib` + dead robot code — **only after your approval** | **PR 2** complete |

- Old and new libraries **coexist** through B1–B5; every commit builds and runs in sim.
- **Behavior deltas to expect (called out now):** ① control-loop gains do not transfer from RIO-voltage-PID to onboard FOC — each migrated mechanism needs SysId/re-tune (sim gains provided as starting points; tuning maps/setpoints are preserved untouched); ② swerve module gains likewise re-derived for the CTRE stack; ③ loop-order fix (P-18) removes 20 ms of sensor latency — strictly better but measurable.

## 13. Testing & docs plan

- Unit tests (JUnit already present): state-machine engine (transitions, guards, deny-by-default), config builders, `AllianceFlip`, zone triggers, `ShiftUtil` (hooks exist).
- Sim-driven checks: each archetype has a sim smoke test (reaches setpoint within tolerance/time).
- Javadoc on all public API; `docs/ADDING_A_SUBSYSTEM.md` recipe; `NOTICE` file crediting 254/6328/3061/1678/TRIGON/MA5951/CTRE/Iron Maple.

---

## 14. Open decisions — **your call before implementation**

| # | Decision | Options | **Recommendation** |
|---|---|---|---|
| OD-1 | New package root | `frc.excalib2` permanent · rename to `frc.excalib` after old lib removal | **`frc.excalib2` permanent** — zero rename churn, unambiguous during migration |
| OD-2 | 🟥 **New vendordeps: DogLog + maple-sim** (rule 7: vendordep changes need your OK) | add both · add DogLog only (defer maple-sim) | **Add both** — DogLog is load-bearing for telemetry; maple-sim for sim parity |
| OD-3 | Telemetry | DogLog+SignalLogger · Epilogue+SignalLogger · both | **DogLog + SignalLogger**, Epilogue later if wanted |
| OD-4 | Control mode in sim | Voltage requests in sim · TorqueCurrentFOC in sim too | **Voltage in sim** — best-supported sim path; FOC on real hardware |
| OD-5 | TunerConstants origin | hand-author from current constants, verify on robot · full Tuner X session | **Hand-author + verify** — no robot downtime now; Tuner X session only if verification fails |
| OD-6 | Auto tool | PathPlanner (current) · Choreo | **PathPlanner** — continuity with existing autos & team knowledge |
| OD-7 | Superstructure goal set | 1:1 port of today's 14 `RobotState`s · redesigned smaller goal set | **1:1 port** — this is a refactor, not a behavior change; prune afterwards |
| OD-8 | REV/NavX wrappers in v2 | drop (no REV hardware) · port | **Drop** — old lib remains in git history if ever needed |

---

**⏸ Stopping here for approval.** On your go (with OD answers or "use recommendations"), Phase 3 begins: library implementation, atomic commits, build green throughout.
