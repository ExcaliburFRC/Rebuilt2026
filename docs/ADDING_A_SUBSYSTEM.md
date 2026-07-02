# How to add a subsystem on ExcaLib v2

Adding a mechanism is **declaring config + states**, not writing control plumbing. The
recipe, end to end:

## 1. Pick the archetype

| Your mechanism | Archetype |
|---|---|
| Arm / pivot / hood / elevator stage / turret | `PositionalMechanism` |
| Flywheel / shooter wheel / drum | `VelocityMechanism` |
| Intake / feeder / indexer roller | `RollerMechanism` |
| Drivetrain | `SwerveSubsystem` |

## 2. Declare the config

```java
private final PositionalMechanism hood = new PositionalMechanism(
    MechanismConfig.of("Shooter/Hood", new CANDeviceId(30))
        .fusedCANcoder(new CANDeviceId(31), ROTOR_TO_SENSOR, SENSOR_TO_MECH, Rotations.of(MAGNET_OFFSET))
        .gains(REAL_GAINS, SIM_GAINS)                    // Gains.pid(...).withSVA(...).withGravity(kG, Arm_Cosine)
        .motion(MotionConstraints.expo(EXPO_KV, EXPO_KA)) // Expo = default choice for positional moves
        .softLimits(Radians.of(0.0), Radians.of(0.55))
        .currentBudget(CurrentBudget.of(40, 30))          // add .withSupplyLower / .withTorquePeaks as needed
        .tolerance(Radians.of(0.01))
        .simModel(MechanismSim.arm(DCMotor.getKrakenX60Foc(1), GEARING, MOI, LENGTH, MIN, MAX, START)));
```

Everything else — device construction, config **apply + verify + retry**, StatusSignal
registration, disconnect Alerts, DogLog telemetry, physics sim — happens in the base class.

Rules of thumb:
- **Gains don't transfer between control modes.** TorqueCurrentFOC gains are amps-based;
  Voltage gains volts-based. Sim always runs Voltage (keep `simGains` separate).
- **MotionMagicExpo** is the default for positional moves (time-optimal, motor-shaped,
  smoother and more efficient). Use trapezoidal only when you want constant-cruise motion.
- Put the mechanism's amps in the robot's **current budget table**, not inline.

## 3. Declare the states

```java
public enum HoodState { STOWED, TRACKING }

private final StateMachine<HoodState> machine = StateMachine.builder("Hood", HoodState.STOWED)
    .onEnter(STOWED, () -> hood.setGoal(Radians.of(0)))
    .whileIn(TRACKING, run(() -> hood.setGoal(angleForDistance())))   // command w/ this subsystem as requirement
    .transition(STOWED, TRACKING)
    .transitionFromAny(STOWED)
    .build();
```

- The table is **default-deny**: an undeclared transition is illegal and requests for it fail loudly.
- Guards (`BooleanSupplier`) make transitions wait: `.transition(A, B, shooter.atSpeed)`.

## 4. Wire the subsystem

```java
public class Hood extends SubsystemBase {
    // ... fields from steps 2-3 ...
    @Override public void periodic() {
        hood.periodic();       // telemetry + alerts + sim step
        machine.periodic();    // pending-request retry + state telemetry
    }
    public Command track()  { return machine.requestCommand(TRACKING); }
    public Trigger atGoal() { return hood.atGoal; }
}
```

## 5. Add it to the superstructure

Add a field to the robot-goal enum (goal = tuple of per-mechanism states, so illegal
combinations don't exist as values), fan it out in the superstructure's `onEnter`, and add
interlocks as transition guards.

## 6. Robot loop requirements (already wired in the template)

`robotPeriodic()` must call, in order: `SignalHub.refreshAll()` → `ShiftUtil.update()` →
`CommandScheduler.run()` → `FaultReporter.poll()`. Call `SignalHub.optimizeAll()` once at
the end of robot construction, and `Telemetry.init(...)` in the robot constructor.

## 7. Tune

- Run SysId: `mechanism.sysIdQuasistatic/Dynamic(subsystem, direction)` — data lands in the
  hoot log (SignalLogger); analyze in AdvantageScope / Tuner X.
- Live-tune with `TunableNumber` (enable with `TunableNumber.enableTuning()` in dev only).
- Verify in sim first: every mechanism with a `.simModel(...)` runs physics in `simulateJava`.
