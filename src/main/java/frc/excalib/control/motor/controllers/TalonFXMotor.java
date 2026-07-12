package frc.excalib.control.motor.controllers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.motor_specs.CurrentBudget;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.control.motor.PhoenixSignalHub;
import frc.excalib.telemetry.FaultReporter;

import java.util.ArrayList;
import java.util.HashMap;

import static com.ctre.phoenix6.signals.InvertedValue.*;
import static frc.excalib.control.motor.motor_specs.DirectionState.FORWARD;

public class TalonFXMotor extends TalonFX implements Motor {

    // ── Conversion factors ───────────────────────────────────────────────────
    private double m_positionConversionFactor = 1.0;
    private double m_velocityConversionFactor = 1.0;
    private IdleState m_idleState;

    // ── Status signals ───────────────────────────────────────────────────────
    private final StatusSignal<Angle>           posSignal;
    private final StatusSignal<AngularVelocity> velSignal;
    private final StatusSignal<Current>         supplyCurrentSignal;
    private final StatusSignal<Current>         statorCurrentSignal;
    private final StatusSignal<Voltage>         voltSignal;
    private final StatusSignal<Temperature>     tempSignal;

    // ── Pre-allocated control requests (reused every loop, zero GC) ──────────
    private final DutyCycleOut       m_dutyCycle    = new DutyCycleOut(0);
    private final VoltageOut         m_voltageOut   = new VoltageOut(0);
    private final VelocityVoltage    m_velVoltage   = new VelocityVoltage(0);
    private final PositionVoltage    m_posVoltage   = new PositionVoltage(0);
    private final MotionMagicVoltage m_motionMagic  = new MotionMagicVoltage(0);
    private final Follower           m_follower     = new Follower(0, MotorAlignmentValue.Aligned);

    // ── Static signal registry (organized by CAN bus) ────────────────────────
    private static final ArrayList<TalonFXMotor>                      motors        = new ArrayList<>();
    private static final HashMap<String, ArrayList<BaseStatusSignal>> canMap        = new HashMap<>();
    private static final HashMap<String, BaseStatusSignal[]>          canArrayCache = new HashMap<>();

    // Signal update rates (Hz)
    private static final double RATE_FAST   = 100.0;   // position, velocity
    private static final double RATE_MEDIUM =  50.0;   // supply current, stator current, voltage
    private static final double RATE_SLOW   =  10.0;   // temperature

    private final String m_canbusName;

    // ── Constructors ─────────────────────────────────────────────────────────

    /** Default RIO CAN bus. */
    public TalonFXMotor(int deviceId) {
        this(deviceId, new CANBus(""));
    }

    public TalonFXMotor(int deviceId, CANBus canbus) {
        super(deviceId, canbus);
        m_canbusName = canbus.getName();

        setIdleState(IdleState.BRAKE);
        applyRampRates(0.1, 0.05);

        posSignal           = super.getPosition();
        velSignal           = super.getVelocity();
        supplyCurrentSignal = super.getSupplyCurrent();
        statorCurrentSignal = super.getStatorCurrent();
        voltSignal          = super.getMotorVoltage();
        tempSignal          = super.getDeviceTemp();

        // Configure signal update rates
        BaseStatusSignal.setUpdateFrequencyForAll(RATE_FAST,   posSignal, velSignal);
        BaseStatusSignal.setUpdateFrequencyForAll(RATE_MEDIUM, supplyCurrentSignal, statorCurrentSignal, voltSignal);
        BaseStatusSignal.setUpdateFrequencyForAll(RATE_SLOW,   tempSignal);

        // Register signals and rebuild cached array for this bus
        canMap.computeIfAbsent(m_canbusName, k -> new ArrayList<>());
        ArrayList<BaseStatusSignal> signals = canMap.get(m_canbusName);
        signals.add(posSignal);
        signals.add(velSignal);
        signals.add(supplyCurrentSignal);
        signals.add(statorCurrentSignal);
        signals.add(voltSignal);
        signals.add(tempSignal);
        canArrayCache.put(m_canbusName, signals.toArray(new BaseStatusSignal[0]));

        motors.add(this);

        // Auto-register for 1 Hz fault scanning (hardware / temp / boot / undervoltage + CAN health).
        FaultReporter.register("TalonFX-" + deviceId + "@" + m_canbusName, this);

        // Register for the CAN bus-utilization optimize pass (used signals are configured above).
        PhoenixSignalHub.register(this);
    }

    // ── Static lifecycle methods ──────────────────────────────────────────────

    /** Call once per robot loop to batch-refresh all Phoenix6 signals. */
    public static void refreshAll() {
        for (BaseStatusSignal[] signals : canArrayCache.values()) {
            BaseStatusSignal.refreshAll(signals);
        }
    }

    /**
     * Call once after all TalonFX motors are constructed.
     * Disables unused CAN signals, significantly reducing bus load.
     */
    public static void optimizeAll() {
        ParentDevice.optimizeBusUtilizationForAll(motors.toArray(new TalonFXMotor[0]));
    }

    // ── Control ──────────────────────────────────────────────────────────────

    @Override
    public void setPercentage(double percentage) {
        super.setControl(m_dutyCycle.withOutput(percentage));
    }

    @Override
    public void setMotorVoltage(double voltage) {
        super.setControl(m_voltageOut.withOutput(voltage));
    }

    /** Follow another TalonFX (same direction). */
    @Override
    public void setFollower(int mainMotorID) {
        super.setControl(m_follower.withLeaderID(mainMotorID).withMotorAlignment(MotorAlignmentValue.Aligned));
    }

    /** Follow another TalonFX with explicit direction control. */
    public void setFollower(int mainMotorID, boolean oppose) {
        super.setControl(m_follower.withLeaderID(mainMotorID)
                .withMotorAlignment(oppose ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
    }

    /**
     * Onboard velocity closed-loop at 1 kHz — far more precise than software PID.
     * Requires gains configured with {@link #configurePID(Gains)}.
     *
     * @param velocity       desired velocity in mechanism units/s
     * @param feedforwardVolts  additional feedforward voltage
     */
    public void setVelocityDirect(double velocity, double feedforwardVolts) {
        super.setControl(m_velVoltage
                .withVelocity(velocity / m_velocityConversionFactor)
                .withFeedForward(feedforwardVolts));
    }

    /**
     * Onboard position closed-loop at 1 kHz.
     * Requires gains configured with {@link #configurePID(Gains)}.
     *
     * @param position       desired position in mechanism units
     * @param feedforwardVolts  additional feedforward voltage
     */
    public void setPositionDirect(double position, double feedforwardVolts) {
        super.setControl(m_posVoltage
                .withPosition(position / m_positionConversionFactor)
                .withFeedForward(feedforwardVolts));
    }

    /**
     * Motion Magic profiled position control at 1 kHz — replaces software TrapezoidProfile.
     * Requires gains and profile limits via {@link #configurePID(Gains)} and {@link #configureMotionMagic}.
     */
    public void setMotionMagicPosition(double position, double feedforwardVolts) {
        super.setControl(m_motionMagic
                .withPosition(position / m_positionConversionFactor)
                .withFeedForward(feedforwardVolts));
    }

    /**
     * Override the default ramp rates applied in the constructor (0.1 s open-loop, 0.05 s closed-loop).
     * @param openLoopSeconds   seconds from 0 to full output in open-loop modes
     * @param closedLoopSeconds seconds from 0 to full output in closed-loop modes
     */
    public void setRampRates(double openLoopSeconds, double closedLoopSeconds) {
        applyRampRates(openLoopSeconds, closedLoopSeconds);
    }

    private void applyRampRates(double openLoopSeconds, double closedLoopSeconds) {
        var openRamp = new OpenLoopRampsConfigs()
                .withVoltageOpenLoopRampPeriod(openLoopSeconds)
                .withDutyCycleOpenLoopRampPeriod(openLoopSeconds);
        var closedRamp = new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(closedLoopSeconds)
                .withDutyCycleClosedLoopRampPeriod(closedLoopSeconds);
        super.getConfigurator().apply(openRamp);
        super.getConfigurator().apply(closedRamp);
    }

    // ── Configuration ────────────────────────────────────────────────────────

    /** Load PID + feedforward gains into onboard slot 0. */
    public void configurePID(Gains gains) {
        var slot = new Slot0Configs();
        slot.kP = gains.kp;
        slot.kI = gains.ki;
        slot.kD = gains.kd;
        slot.kS = gains.ks;
        slot.kV = gains.kv;
        slot.kA = gains.ka;
        slot.kG = gains.kg;
        applyChecked(super.getConfigurator().apply(slot), "configurePID");
    }

    /**
     * Configure Motion Magic cruise limits (all in mechanism units / mechanism units per second).
     * Call this before using {@link #setMotionMagicPosition}.
     */
    public void configureMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
        var mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = cruiseVelocity / m_velocityConversionFactor;
        mm.MotionMagicAcceleration   = acceleration   / m_velocityConversionFactor;
        mm.MotionMagicJerk           = jerk           / m_velocityConversionFactor;
        applyChecked(super.getConfigurator().apply(mm), "configureMotionMagic");
    }

    /**
     * Applies a full {@link CurrentBudget} (stator + supply limits, optional time-based supply
     * reduction, and TorqueCurrentFOC output peaks) in one call. Prefer this over
     * {@link #setCurrentLimit} when the mechanism's allocation is kept in a named budget table.
     */
    public void applyCurrentBudget(CurrentBudget budget) {
        applyChecked(super.getConfigurator().apply(budget.toCurrentLimitsConfigs()), "applyCurrentBudget(limits)");
        applyChecked(super.getConfigurator().apply(budget.toTorqueCurrentConfigs()), "applyCurrentBudget(torque)");
    }

    /**
     * Fuses a remote CANcoder into this TalonFX's feedback (Phoenix 6 {@code FusedCANcoder}):
     * the absolute CANcoder sets the mechanism position while the integrated rotor supplies
     * high-rate velocity, so {@link #getMotorPosition()} returns the absolute mechanism angle
     * at the rotor's update rate. Used for swerve steering and any absolute-referenced pivot.
     *
     * <p>After this call, mechanism position is reported directly by the device — do <b>not</b>
     * also seed it with {@link #setMotorPosition}. The CANcoder's magnet offset must be burned
     * into the encoder (this does not set it). ⚠ Verify inversion and ratios on the robot.
     *
     * @param cancoderId            CAN id of the CANcoder (same bus as this motor)
     * @param rotorToSensorRatio    gear ratio from motor rotor to the CANcoder (steer gearing)
     * @param sensorToMechanismRatio gear ratio from the CANcoder to the mechanism (usually 1.0)
     */
    public void configureFusedCANcoder(int cancoderId, double rotorToSensorRatio, double sensorToMechanismRatio) {
        var feedback = new FeedbackConfigs();
        if (!refreshChecked(super.getConfigurator().refresh(feedback), "configureFusedCANcoder")) return;
        feedback.FeedbackSensorSource      = FeedbackSensorSourceValue.FusedCANcoder;
        feedback.FeedbackRemoteSensorID    = cancoderId;
        feedback.RotorToSensorRatio        = rotorToSensorRatio;
        feedback.SensorToMechanismRatio    = sensorToMechanismRatio;
        applyChecked(super.getConfigurator().apply(feedback), "configureFusedCANcoder");
    }

    @Override
    public void setIdleState(IdleState idleMode) {
        m_idleState = idleMode;
        super.setNeutralMode(idleMode == IdleState.BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setInverted(DirectionState mode) {
        var config = new MotorOutputConfigs();
        if (!refreshChecked(super.getConfigurator().refresh(config), "setInverted")) return;
        config.Inverted = (mode == FORWARD) ? CounterClockwise_Positive : Clockwise_Positive;
        applyChecked(super.getConfigurator().apply(config), "setInverted");
    }

    @Override
    public void setCurrentLimit(int statorLimit, int supplyLimit) {
        var config = new CurrentLimitsConfigs();
        if (!refreshChecked(super.getConfigurator().refresh(config), "setCurrentLimit")) return;
        config.StatorCurrentLimit       = statorLimit;
        config.StatorCurrentLimitEnable = true;
        config.SupplyCurrentLimit       = supplyLimit;
        config.SupplyCurrentLimitEnable = true;
        applyChecked(super.getConfigurator().apply(config), "setCurrentLimit");
    }

    @Override
    public void setSoftLimit(DirectionState directionState, float limit) {
        var config = new SoftwareLimitSwitchConfigs();
        if (!refreshChecked(super.getConfigurator().refresh(config), "setSoftLimit")) return;
        if (directionState == FORWARD) {
            config.ForwardSoftLimitThreshold = limit / m_positionConversionFactor;
            config.ForwardSoftLimitEnable    = true;
        } else {
            config.ReverseSoftLimitThreshold = limit / m_positionConversionFactor;
            config.ReverseSoftLimitEnable    = true;
        }
        applyChecked(super.getConfigurator().apply(config), "setSoftLimit");
    }

    @Override
    public void setPositionConversionFactor(double factor) { m_positionConversionFactor = factor; }

    @Override
    public void setVelocityConversionFactor(double factor) { m_velocityConversionFactor = factor; }

    @Override
    public void setMotorPosition(double position) {
        super.setPosition(position / m_positionConversionFactor);
    }

    // ── Sensors ──────────────────────────────────────────────────────────────

    @Override public double getMotorPosition()  { return m_positionConversionFactor * posSignal.getValueAsDouble(); }
    @Override public double getMotorVelocity()  { return m_velocityConversionFactor * velSignal.getValueAsDouble(); }
    @Override public double getCurrent()             { return supplyCurrentSignal.getValueAsDouble(); }
    @Override public double getMotorStatorCurrent()  { return statorCurrentSignal.getValueAsDouble(); }
    @Override public double getVoltage()        { return voltSignal.getValueAsDouble(); }
    @Override public double getTemperature()    { return tempSignal.getValueAsDouble(); }
    @Override public IdleState getIdleState()   { return m_idleState; }

    /** Refresh only this motor's signals. Prefer {@link #refreshAll()} in the robot loop. */
    public void refresh() {
        BaseStatusSignal.refreshAll(
                posSignal, velSignal, supplyCurrentSignal, statorCurrentSignal, voltSignal, tempSignal);
    }

    // ── Internal ─────────────────────────────────────────────────────────────

    private void applyChecked(StatusCode code, String context) {
        if (!code.isOK())
            DriverStation.reportWarning("TalonFX[" + getDeviceID() + "] " + context + " failed: " + code, false);
    }

    /** Returns true if refresh succeeded; logs a warning and returns false on failure. */
    private boolean refreshChecked(StatusCode code, String context) {
        if (!code.isOK()) {
            DriverStation.reportWarning("TalonFX[" + getDeviceID() + "] " + context + " refresh failed: " + code, false);
            return false;
        }
        return true;
    }
}
