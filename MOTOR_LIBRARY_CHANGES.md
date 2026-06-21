# Motor Library Optimization — Change Report

> Build status: ✅ **PASSES** (`./gradlew build`)
> Branch: `claude/motor-library-optimization-yrnfbp`
> Files changed: 9

---

## Files Changed

| File | Category |
|------|----------|
| `Motor.java` | Interface |
| `TalonFXMotor.java` | Core driver |
| `SparkMaxMotor.java` | REV driver |
| `FlexMotor.java` | REV driver |
| `MotorGroup.java` | Composite |
| `FlyWheel.java` | Mechanism |
| `Arm.java` | Mechanism |
| `Turret.java` | Mechanism |
| `LinearExtension.java` | Mechanism |

---

## Fix-by-Fix Breakdown

---

### 1 — `TalonFXMotor`: Constructors unified, `canbus` field always set

**Before**
```java
// Two separate constructors, both duplicating all initialization
// Default constructor left canbus field as "" without ever calling canbus.getName()
public TalonFXMotor(int deviceId, CANBus canbus) { super(deviceId, canbus); ... this.canbus = canbus.getName(); }
public TalonFXMotor(int deviceId)               { super(deviceId);         ... /* canbus stays "" */ }
```

**After**
```java
// Default constructor delegates — single source of truth
public TalonFXMotor(int deviceId)               { this(deviceId, new CANBus("")); }
public TalonFXMotor(int deviceId, CANBus canbus) { super(deviceId, canbus); m_canbusName = canbus.getName(); ... }
```

---

### 2 — `TalonFXMotor`: Control requests pre-allocated (zero GC per loop)

**Before**
```java
// new object every 20 ms → GC pressure, potential loop jitter
public void setPercentage(double percentage) {
    super.setControl(new DutyCycleOut(percentage));         // allocates
}
public void setFollower(int mainMotorID) {
    super.setControl(new Follower(mainMotorID, MotorAlignmentValue.Opposed)); // allocates
}
```

**After**
```java
// pre-allocated as member variables, mutated in-place
private final DutyCycleOut       m_dutyCycle   = new DutyCycleOut(0);
private final VoltageOut         m_voltageOut  = new VoltageOut(0);
private final VelocityVoltage    m_velVoltage  = new VelocityVoltage(0);
private final PositionVoltage    m_posVoltage  = new PositionVoltage(0);
private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0);
private final Follower           m_follower    = new Follower(0, MotorAlignmentValue.Aligned);

public void setPercentage(double p) { super.setControl(m_dutyCycle.withOutput(p)); }
public void setMotorVoltage(double v) { super.setControl(m_voltageOut.withOutput(v)); }
```

---

### 3 — `TalonFXMotor`: Signal update rates explicitly configured

**Before**
```java
// No rate configured — position/velocity default to 50 Hz, current/voltage to 4 Hz, temp to 1 Hz
// refreshAll() calls every 20 ms but most signals are stale 90-99% of calls
poseSignal   = super.getPosition();
velSignal    = super.getVelocity();
// (no setUpdateFrequency anywhere)
```

**After**
```java
// Explicit rates: position+velocity at 100 Hz, current+voltage at 50 Hz, temperature at 10 Hz
private static final double RATE_FAST   = 100.0;
private static final double RATE_MEDIUM =  50.0;
private static final double RATE_SLOW   =  10.0;

BaseStatusSignal.setUpdateFrequencyForAll(RATE_FAST,   posSignal, velSignal);
BaseStatusSignal.setUpdateFrequencyForAll(RATE_MEDIUM, supplyCurrentSignal, statorCurrentSignal, voltSignal);
BaseStatusSignal.setUpdateFrequencyForAll(RATE_SLOW,   tempSignal);
```

---

### 4 — `TalonFXMotor`: Signal array cached (no per-loop allocation)

**Before**
```java
// toArray(new BaseStatusSignal[0]) allocates a zero-length probe array every call
public static void refreshAll() {
    for (ArrayList<BaseStatusSignal> signals : canMap.values()) {
        BaseStatusSignal.refreshAll(signals.toArray(new BaseStatusSignal[0])); // allocates
    }
}
```

**After**
```java
// Array rebuilt only when a new motor is added (startup); zero allocation at runtime
private static final HashMap<String, BaseStatusSignal[]> canArrayCache = new HashMap<>();

// on motor construction:
canArrayCache.put(m_canbusName, signals.toArray(new BaseStatusSignal[0]));

public static void refreshAll() {
    for (BaseStatusSignal[] signals : canArrayCache.values()) {
        BaseStatusSignal.refreshAll(signals);   // no allocation
    }
}
```

---

### 5 — `TalonFXMotor`: Temperature signal consistent across both constructors

**Before**
```java
// CANBus constructor: temperatureSignal registered ✓
// Default constructor: temperatureSignal commented out — always returned stale data
//        signals.add(temperatureSignal);   // ← commented out in TalonFXMotor(int)
```

**After**
```java
// Single constructor path — temperature registered for all motors
signals.add(tempSignal);   // always present
```

---

### 6 — `TalonFXMotor`: `setInverted` reads before writing (safe config apply)

**Before**
```java
// Creates a blank MotorOutputConfigs — silently resets peak output, deadband, etc.
public void setInverted(DirectionState mode) {
    var talonFXConfigurator = new MotorOutputConfigs();          // blank — overwrites everything
    talonFXConfigurator.withInverted(...);
    super.getConfigurator().apply(talonFXConfigurator);
}
```

**After**
```java
// Read-modify-write: only the invert field changes
public void setInverted(DirectionState mode) {
    var config = new MotorOutputConfigs();
    super.getConfigurator().refresh(config);                     // read current settings
    config.Inverted = (mode == FORWARD) ? CounterClockwise_Positive : Clockwise_Positive;
    applyChecked(super.getConfigurator().apply(config), "setInverted");
}
```

---

### 7 — `TalonFXMotor`: `setFollower` default changed from Opposed to Aligned

**Before**
```java
// Hardcoded Opposed: every follower spins backwards — wrong for aligned motors
super.setControl(new Follower(mainMotorID, MotorAlignmentValue.Opposed));
```

**After**
```java
// Default Aligned; overloaded version lets you choose explicitly
public void setFollower(int mainMotorID)                  { m_follower.withLeaderID(mainMotorID).withMotorAlignment(Aligned); }
public void setFollower(int mainMotorID, boolean oppose)  { m_follower...withMotorAlignment(oppose ? Opposed : Aligned); }
```

---

### 8 — `TalonFXMotor`: `setSoftLimit` implemented (was TODO stub)

**Before**
```java
public void setSoftLimit(DirectionState directionState, float limit) {
    var talonFXConfigurator = super.getConfigurator(); //TODO: implement
}
```

**After**
```java
public void setSoftLimit(DirectionState directionState, float limit) {
    var config = new SoftwareLimitSwitchConfigs();
    super.getConfigurator().refresh(config);
    if (directionState == FORWARD) {
        config.ForwardSoftLimitThreshold = limit / m_positionConversionFactor;
        config.ForwardSoftLimitEnable    = true;
    } else {
        config.ReverseSoftLimitThreshold = limit / m_positionConversionFactor;
        config.ReverseSoftLimitEnable    = true;
    }
    applyChecked(super.getConfigurator().apply(config), "setSoftLimit");
}
```

---

### 9 — `TalonFXMotor`: Config errors are now reported

**Before**
```java
// apply() return value ignored — failures silently dropped
talonFXConfigurator.apply(limitConfigs);
```

**After**
```java
// All apply() calls go through applyChecked() which logs to DriverStation on failure
private void applyChecked(StatusCode code, String context) {
    if (!code.isOK())
        DriverStation.reportWarning("TalonFX[" + getDeviceID() + "] " + context + " failed: " + code, false);
}
```

---

### 10 — `TalonFXMotor`: Stator current added to interface + all implementations

**Before**
```java
// Only supply current exposed — stator current (motor torque signal) invisible
double getCurrent();   // supply current only
```

**After**
```java
// Motor interface
double getCurrent();            // supply current (input to controller)
double getMotorStatorCurrent(); // stator current (motor torque, jam detection)

// TalonFXMotor: dedicated signal refreshed at 50 Hz
private final StatusSignal<Current> statorCurrentSignal = super.getStatorCurrent();
// SparkMax / Flex: return getOutputCurrent() (equivalent for REV hardware)
```

---

### 11 — `TalonFXMotor`: `refresh()` no longer allocates

**Before**
```java
public void refresh() {
    ArrayList<BaseStatusSignal> signals = new ArrayList<>();  // allocates list
    signals.add(poseSignal); ...
    BaseStatusSignal.refreshAll(signals.toArray(new BaseStatusSignal[0]));  // allocates array
}
```

**After**
```java
public void refresh() {
    BaseStatusSignal.refreshAll(
        posSignal, velSignal, supplyCurrentSignal, statorCurrentSignal, voltSignal, tempSignal);
}
```

---

### 12 — `TalonFXMotor`: `optimizeAll()` added

**Before**
```java
// Unused CAN signals broadcast continuously — wastes bandwidth
// (nothing called ParentDevice.optimizeBusUtilizationForAll)
```

**After**
```java
// Call once after all motors are constructed (e.g., end of RobotContainer init)
public static void optimizeAll() {
    ParentDevice.optimizeBusUtilizationForAll(motors.toArray(new TalonFXMotor[0]));
}
```

---

### 13 — `TalonFXMotor`: Onboard closed-loop methods added

**Before**
```java
// No onboard PID — all control ran in software at 50 Hz
// (setMotorVoltage was the only control path)
```

**After**
```java
// Onboard closed-loop runs at 1 kHz on the Kraken — 20× faster than software PID
public void configurePID(Gains gains)                                          { /* writes Slot0Configs */ }
public void configureMotionMagic(double cruiseVel, double accel, double jerk)  { /* writes MotionMagicConfigs */ }
public void setVelocityDirect(double velocity, double feedforwardVolts)        { /* VelocityVoltage request */ }
public void setPositionDirect(double position, double feedforwardVolts)        { /* PositionVoltage request */ }
public void setMotionMagicPosition(double position, double feedforwardVolts)   { /* MotionMagicVoltage request */ }
```

> **To migrate a mechanism**: call `motor.configurePID(gains)` at init,
> then replace `setMotorVoltage(pid + ff)` with `motor.setVelocityDirect(setpoint, ff)`.

---

### 14 — `SparkMaxMotor`: Config class corrected

**Before**
```java
// SparkMax using SparkFlexConfig — wrong hardware class, SparkMax-specific settings silently ignored
private final SparkFlexConfig config = new SparkFlexConfig();
```

**After**
```java
private final SparkMaxConfig config = new SparkMaxConfig();
```

---

### 15 — `FlyWheel`: `TrapezoidProfile` pre-allocated

**Before**
```java
// Allocates two objects every 20 ms inside the RunCommand lambda
() -> {
    TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxAcceleration, maxJerk));
    ...
}
```

**After**
```java
// Allocated once in constructor, reused every loop
m_constraints = new TrapezoidProfile.Constraints(maxAcceleration, maxJerk);
m_profile     = new TrapezoidProfile(m_constraints);
// lambda just calls m_profile.calculate(...)
```

---

### 16 — `FlyWheel`: `getAcceleration()` divide-by-zero guard + proper init

**Before**
```java
// lastTime = 0 on construction → (currentTime - 0) = huge divisor on first call → wrong acceleration
private double lastTime, lastVelocity;   // both start at 0

private double getAcceleration() {
    double currentTime = Timer.getFPGATimestamp();
    return (currentVelocity - lastVelocity) / (currentTime - lastTime);  // no guard
}
```

**After**
```java
// Initialized to real values; dt guard prevents divide-by-zero
m_lastTime     = Timer.getFPGATimestamp();
m_lastVelocity = motor.getMotorVelocity();

private double getAcceleration() {
    double dt = Timer.getFPGATimestamp() - m_lastTime;
    if (dt < 1e-6) return 0.0;   // guard
    return (m_motor.getMotorVelocity() - m_lastVelocity) / dt;
}
```

---

### 17 — `FlyWheel`: Hardcoded `0.02` replaced

**Before**
```java
profile.calculate(0.02, ...)
```

**After**
```java
m_profile.calculate(TimedRobot.kDefaultPeriod, ...)
```

---

### 18 — `Arm`: `m_kv` now applied in control law

**Before**
```java
// m_kv declared, extracted from gains, but never used in output calculation
public final double m_kv, m_ks, m_kg;
...
double phyOutput = m_ks * Math.signum(velocitySetpoint) + m_kg * m_mass.getCenterOfMass().getX();
//                 ^ kv missing — velocity feedforward term dead
```

**After**
```java
double phyOutput = m_ks * Math.signum(velocitySetpoint)
                 + m_kv * velocitySetpoint          // ← now applied
                 + m_kg * m_mass.getCenterOfMass().getX();
```

---

### 19 — `Arm`: Double soft-limit / wrong dimension fixed

**Before**
```java
// Applied velocityLimit twice: once to velocity (correct) and once to voltage (wrong dimensions)
velocitySetpoint = velocityLimit.limit(velocitySetpoint);
...
super.setVoltage(velocityLimit.limit(output));  // ← limits volts using velocity bounds — wrong
```

**After**
```java
// Velocity clamped once; output clamped to ±12 V (battery max)
double velocitySetpoint = velocityLimit.limit(error / TimedRobot.kDefaultPeriod);
...
double output = MathUtil.clamp(phyOutput + pid, -MAX_VOLTAGE, MAX_VOLTAGE);
super.setVoltage(output);
```

---

### 20 — `Turret`: FF direction fixed (`signum(error)` not `signum(pid)`)

**Before**
```java
// signum(pid) flips direction when PID crosses zero, causing a voltage jolt at setpoint
double ff = m_angleFFcontroller.getKs() * Math.signum(pid);
```

**After**
```java
// signum(error): direction of travel, smoothly zero at setpoint
double ks = m_anglePIDcontroller.atSetpoint() ? 0 : m_angleFFcontroller.getKs() * Math.signum(error);
```

---

### 21 — `Turret`: kv feedforward now applied

**Before**
```java
// SimpleMotorFeedforward constructed with ks, kv, ka but only getKs() ever called
// kv * velocity was silently zero
double ff = m_angleFFcontroller.getKs() * Math.signum(pid);
```

**After**
```java
// kv applied using profile velocity (profiled) or actual motor velocity (unprofiled)
double kv = m_angleFFcontroller.getKv() * kvVelocity;
super.setVoltage(pid + ks + kv);
```

---

### 22 — `Turret`: Redundant `limit()` after `getSetpoint()` removed

**Before**
```java
// getSetpoint() already returns a valid wrapped angle; limit() may corrupt it
double smartSetpoint       = m_rotationLimit.getSetpoint(...);
this.smartSetpoint         = m_rotationLimit.limit(smartSetpoint);  // ← redundant, potentially wrong
```

**After**
```java
// Single assignment; getSetpoint() is the authoritative clamping
this.smartSetpoint = m_rotationLimit.getSetpoint(getPosition().getRadians(), wantedPosition.getRadians());
```

---

### 23 — `Turret`: Dead second constructor now implements profiling

**Before**
```java
// Accepted TrapezoidProfile.Constraints but never stored or used them
public Turret(Motor motor, ..., TrapezoidProfile.Constraints constraints) {
    // ... identical to no-constraints constructor — constraints silently ignored
}
```

**After**
```java
// Single unified constructor; if constraints != null, profiling is active
private final TrapezoidProfile m_profile;
private TrapezoidProfile.State m_profileState = new TrapezoidProfile.State(0, 0);

// In setPosition():
if (m_profile != null) {
    m_profileState = m_profile.calculate(TimedRobot.kDefaultPeriod, m_profileState, new State(smartSetpoint, 0));
    // PID tracks profile position; kv tracks profile velocity
}
```

---

### 24 — `LinearExtension`: `TrapezoidProfile` pre-allocated + period constant

**Before**
```java
// New TrapezoidProfile allocated every 20 ms inside the RunCommand
() -> {
    TrapezoidProfile profile = new TrapezoidProfile(m_constraints);  // allocates
    profile.calculate(0.02, ...);                                     // hardcoded
}
```

**After**
```java
// Pre-allocated in constructor
m_profile = new TrapezoidProfile(constraints);

// In command:
m_profile.calculate(TimedRobot.kDefaultPeriod, ...);
```

---

## Summary Impact Table

| # | Fix | Impact |
|---|-----|--------|
| 1 | Unified constructors | Code hygiene |
| 2 | Pre-allocated control requests | No GC per loop |
| 3 | Signal update rates set | Correct data freshness |
| 4 | Cached signal arrays | No GC per loop |
| 5 | Temperature signal consistent | Accurate temp telemetry |
| 6 | `setInverted` read-modify-write | No silent config wipe |
| 7 | `setFollower` defaults to Aligned | Correct motor direction |
| 8 | `setSoftLimit` implemented | Hardware safety |
| 9 | Config error reporting | Debuggability |
| 10 | Stator current added | Torque/jam detection |
| 11 | `refresh()` no allocation | Minor GC |
| 12 | `optimizeAll()` added | Reduced CAN bus load |
| 13 | Onboard PID methods | 1 kHz control path available |
| 14 | `SparkMaxConfig` fix | Correct REV config |
| 15 | `TrapezoidProfile` pre-allocated (FlyWheel) | No GC per loop |
| 16 | `getAcceleration()` guarded + initialized | No NaN/bad first sample |
| 17 | `kDefaultPeriod` constant | Correct if loop rate changes |
| 18 | `m_kv` applied in Arm | Correct velocity FF |
| 19 | Arm voltage clamp fixed | No velocity-as-voltage bug |
| 20 | Turret FF direction from error sign | No jolt at setpoint |
| 21 | Turret kv FF applied | Under-compensation fixed |
| 22 | Redundant `limit()` removed | Correct setpoint wrapping |
| 23 | Turret second constructor profiles | Constraints actually used |
| 24 | `TrapezoidProfile` pre-allocated (LinearExtension) | No GC per loop |
