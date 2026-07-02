package frc.excalib2.mechanisms;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import frc.excalib2.control.ControlMode;
import frc.excalib2.control.CurrentBudget;
import frc.excalib2.control.Gains;
import frc.excalib2.control.MotionConstraints;
import frc.excalib2.device.CANDeviceId;
import frc.excalib2.sim.MechanismSim;

import static edu.wpi.first.units.Units.Rotations;

/**
 * Declarative configuration for one mechanism — the "fill in the template" object.
 *
 * <p>A concrete subsystem should consist almost entirely of one of these per mechanism:
 * devices, ratios, gains (real + sim), motion constraints, current budget, limits,
 * tolerance, and a sim model. All the plumbing (config apply+verify, signal registration,
 * telemetry, alerts, simulation) lives in the mechanism base classes.
 *
 * <p>Angles are WPILib {@link Angle} measures; internally everything is mechanism rotations.
 */
public final class MechanismConfig {
    // Identity & devices
    final String name;
    final CANDeviceId motorId;
    CANDeviceId followerId = null;
    boolean followerOpposed = false;

    // Behavior
    ControlMode controlMode = ControlMode.TORQUE_CURRENT_FOC;
    InvertedValue inverted = InvertedValue.CounterClockwise_Positive;
    NeutralModeValue neutralMode = NeutralModeValue.Brake;
    Gains realGains = Gains.empty();
    Gains simGains = Gains.empty();
    MotionConstraints motion = MotionConstraints.trapezoidal(0, 0, 0);
    CurrentBudget currentBudget = CurrentBudget.of(40, 30);

    // Feedback
    double sensorToMechanismRatio = 1.0;
    CANDeviceId cancoderId = null;
    double rotorToSensorRatio = 1.0;
    double cancoderMagnetOffsetRotations = 0.0;
    SensorDirectionValue cancoderDirection = SensorDirectionValue.CounterClockwise_Positive;

    // Limits & tolerances (mechanism rotations)
    Double forwardSoftLimitRotations = null;
    Double reverseSoftLimitRotations = null;
    boolean continuousWrap = false;
    double toleranceRotations = 0.02;
    double peakForwardVolts = 12.0;
    double peakReverseVolts = -12.0;
    double openLoopRampSeconds = 0.0;
    double closedLoopRampSeconds = 0.0;

    // Simulation
    MechanismSim.ModelFactory simModel = null;

    private MechanismConfig(String name, CANDeviceId motorId) {
        this.name = name;
        this.motorId = motorId;
    }

    /** Starts a config for the mechanism driven by {@code motorId}. {@code name} is the telemetry path. */
    public static MechanismConfig of(String name, CANDeviceId motorId) {
        return new MechanismConfig(name, motorId);
    }

    // ── Builder methods ──────────────────────────────────────────────────────

    /** Adds a follower motor (same configuration; direction via {@code opposed}). */
    public MechanismConfig follower(CANDeviceId followerId, boolean opposed) {
        this.followerId = followerId;
        this.followerOpposed = opposed;
        return this;
    }

    /** Overrides the control-request family (default: TorqueCurrentFOC; sim always runs VOLTAGE). */
    public MechanismConfig controlMode(ControlMode mode) {
        this.controlMode = mode;
        return this;
    }

    public MechanismConfig inverted(boolean clockwisePositive) {
        this.inverted = clockwisePositive
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        return this;
    }

    public MechanismConfig neutralMode(NeutralModeValue mode) {
        this.neutralMode = mode;
        return this;
    }

    /** Real-robot and simulation gain sets (they rarely match — keep both). */
    public MechanismConfig gains(Gains real, Gains sim) {
        this.realGains = real;
        this.simGains = sim;
        return this;
    }

    public MechanismConfig motion(MotionConstraints constraints) {
        this.motion = constraints;
        return this;
    }

    public MechanismConfig currentBudget(CurrentBudget budget) {
        this.currentBudget = budget;
        return this;
    }

    /** Rotor rotations per mechanism rotation, when no absolute encoder is fused. */
    public MechanismConfig rotorToMechanismRatio(double ratio) {
        this.sensorToMechanismRatio = ratio;
        return this;
    }

    /**
     * Fuses a CANcoder as the feedback sensor (FusedCANcoder).
     *
     * @param cancoderId              the CANcoder
     * @param rotorToSensorRatio      rotor rotations per CANcoder rotation
     * @param sensorToMechanismRatio  CANcoder rotations per mechanism rotation (1.0 when 1:1)
     * @param magnetOffset            CANcoder magnet offset
     */
    public MechanismConfig fusedCANcoder(CANDeviceId cancoderId, double rotorToSensorRatio,
                                         double sensorToMechanismRatio, Angle magnetOffset) {
        this.cancoderId = cancoderId;
        this.rotorToSensorRatio = rotorToSensorRatio;
        this.sensorToMechanismRatio = sensorToMechanismRatio;
        this.cancoderMagnetOffsetRotations = magnetOffset.in(Rotations);
        return this;
    }

    public MechanismConfig cancoderDirection(SensorDirectionValue direction) {
        this.cancoderDirection = direction;
        return this;
    }

    /** Device-side soft limits (mechanism angles). */
    public MechanismConfig softLimits(Angle reverse, Angle forward) {
        this.reverseSoftLimitRotations = reverse.in(Rotations);
        this.forwardSoftLimitRotations = forward.in(Rotations);
        return this;
    }

    /**
     * Marks the mechanism as continuously rotating (turret): position goals are wrapped to
     * the nearest equivalent angle inside the soft-limit range before being commanded.
     * Requires {@link #softLimits}.
     */
    public MechanismConfig continuousWrap() {
        this.continuousWrap = true;
        return this;
    }

    /** At-goal / at-speed tolerance. */
    public MechanismConfig tolerance(Angle tolerance) {
        this.toleranceRotations = tolerance.in(Rotations);
        return this;
    }

    public MechanismConfig peakVoltage(double forward, double reverse) {
        this.peakForwardVolts = forward;
        this.peakReverseVolts = reverse;
        return this;
    }

    public MechanismConfig ramps(double openLoopSeconds, double closedLoopSeconds) {
        this.openLoopRampSeconds = openLoopSeconds;
        this.closedLoopRampSeconds = closedLoopSeconds;
        return this;
    }

    /** Physics model used in simulation. Every mechanism should set one (sim parity). */
    public MechanismConfig simModel(MechanismSim.ModelFactory factory) {
        this.simModel = factory;
        return this;
    }

    // ── Derived ──────────────────────────────────────────────────────────────

    /** Effective control mode: VOLTAGE in simulation regardless of the configured mode. */
    ControlMode effectiveControlMode() {
        return RobotBase.isSimulation() ? ControlMode.VOLTAGE : controlMode;
    }

    /** Total rotor rotations per mechanism rotation (used by the sim feed). */
    double rotorPerMechanismRatio() {
        return cancoderId != null ? rotorToSensorRatio * sensorToMechanismRatio : sensorToMechanismRatio;
    }

    /**
     * Builds the full TalonFX configuration. In simulation: voltage-mode gains, plain rotor
     * feedback (no fused CANcoder), sim gain set.
     */
    TalonFXConfiguration toTalonFXConfiguration() {
        boolean sim = RobotBase.isSimulation();
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = inverted;
        config.MotorOutput.NeutralMode = neutralMode;

        config.Slot0 = (sim ? simGains : realGains).toSlot0Configs();
        config.MotionMagic = motion.toMotionMagicConfigs();
        config.CurrentLimits = currentBudget.toCurrentLimitsConfigs();
        config.TorqueCurrent = currentBudget.toTorqueCurrentConfigs();

        config.Voltage.PeakForwardVoltage = peakForwardVolts;
        config.Voltage.PeakReverseVoltage = peakReverseVolts;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRampSeconds;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRampSeconds;
        config.OpenLoopRamps.TorqueOpenLoopRampPeriod = openLoopRampSeconds;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRampSeconds;

        if (cancoderId != null && !sim) {
            config.Feedback.FeedbackRemoteSensorID = cancoderId.id();
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            config.Feedback.RotorToSensorRatio = rotorToSensorRatio;
            config.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;
        } else {
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            config.Feedback.SensorToMechanismRatio = rotorPerMechanismRatio();
        }

        if (forwardSoftLimitRotations != null) {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardSoftLimitRotations;
        }
        if (reverseSoftLimitRotations != null) {
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseSoftLimitRotations;
        }
        return config;
    }

    /** Builds the fused CANcoder's configuration (real robot only). */
    CANcoderConfiguration toCANcoderConfiguration() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = cancoderMagnetOffsetRotations;
        config.MagnetSensor.SensorDirection = cancoderDirection;
        return config;
    }
}
