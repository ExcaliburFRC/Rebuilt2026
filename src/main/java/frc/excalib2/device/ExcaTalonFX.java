package frc.excalib2.device;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.excalib2.control.ControlMode;

/**
 * ExcaLib v2's TalonFX wrapper (composition, not inheritance).
 *
 * <p>Owns the device's status signals (registered with {@link SignalHub} at deliberate
 * frequencies), pre-allocated control requests for both the TorqueCurrentFOC and Voltage
 * families, and the {@link ControlMode} switch between them.
 *
 * <p>All positions/velocities are in <b>mechanism units</b> (rotations, rotations/s):
 * {@code SensorToMechanismRatio}/{@code RotorToSensorRatio} are set in the device
 * configuration, so no RIO-side conversion factors exist.
 */
public class ExcaTalonFX {
    private final TalonFX talon;
    private final String name;
    private final ControlMode controlMode;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Temperature> temperature;

    // Pre-allocated control requests (mutated, never re-allocated — zero-GC hot path)
    private final MotionMagicExpoTorqueCurrentFOC mmExpoTorque = new MotionMagicExpoTorqueCurrentFOC(0);
    private final MotionMagicTorqueCurrentFOC mmTorque = new MotionMagicTorqueCurrentFOC(0);
    private final MotionMagicVelocityTorqueCurrentFOC mmVelTorque = new MotionMagicVelocityTorqueCurrentFOC(0);
    private final TorqueCurrentFOC torqueOut = new TorqueCurrentFOC(0);
    private final MotionMagicExpoVoltage mmExpoVolts = new MotionMagicExpoVoltage(0);
    private final MotionMagicVoltage mmVolts = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage mmVelVolts = new MotionMagicVelocityVoltage(0);
    private final VoltageOut voltsOut = new VoltageOut(0);
    private final DutyCycleOut dutyOut = new DutyCycleOut(0);
    private final NeutralOut neutral = new NeutralOut();
    private final Follower follower = new Follower(0, MotorAlignmentValue.Aligned);

    /**
     * Creates, configures (apply + verify + retry), and signal-registers a TalonFX.
     *
     * @param name          telemetry/alert name, e.g. {@code "Shooter/Hood"}
     * @param deviceId      CAN id + bus
     * @param configuration full device configuration (built by {@code MechanismConfig})
     * @param controlMode   which request family closed-loop calls use
     */
    public ExcaTalonFX(String name, CANDeviceId deviceId, TalonFXConfiguration configuration, ControlMode controlMode) {
        this.name = name;
        this.controlMode = controlMode;
        this.talon = new TalonFX(deviceId.id(), deviceId.toCANBus());

        DeviceConfigs.applyVerified(name, talon, configuration);

        position = talon.getPosition();
        velocity = talon.getVelocity();
        appliedVolts = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        statorCurrent = talon.getStatorCurrent();
        temperature = talon.getDeviceTemp();

        SignalHub.register(deviceId.busName(), SignalHub.Rate.FAST, position, velocity);
        SignalHub.register(deviceId.busName(), SignalHub.Rate.MEDIUM, appliedVolts, supplyCurrent, statorCurrent);
        SignalHub.register(deviceId.busName(), SignalHub.Rate.SLOW, temperature);
        SignalHub.registerDevice(talon);
        frc.excalib2.telemetry.FaultReporter.register(name, talon);
    }

    // ── Closed loop (mechanism units) ────────────────────────────────────────

    /** MotionMagic Expo profiled move to a position goal (mechanism rotations). */
    public void setExpoPositionGoal(double mechanismRotations) {
        switch (controlMode) {
            case TORQUE_CURRENT_FOC -> talon.setControl(mmExpoTorque.withPosition(mechanismRotations));
            default -> talon.setControl(mmExpoVolts.withPosition(mechanismRotations));
        }
    }

    /** Trapezoidal MotionMagic profiled move to a position goal (mechanism rotations). */
    public void setTrapezoidPositionGoal(double mechanismRotations) {
        switch (controlMode) {
            case TORQUE_CURRENT_FOC -> talon.setControl(mmTorque.withPosition(mechanismRotations));
            default -> talon.setControl(mmVolts.withPosition(mechanismRotations));
        }
    }

    /** MotionMagicVelocity profiled (accel/jerk-limited) velocity goal (mechanism rot/s). */
    public void setVelocityGoal(double mechanismRotationsPerSecond) {
        switch (controlMode) {
            case TORQUE_CURRENT_FOC -> talon.setControl(mmVelTorque.withVelocity(mechanismRotationsPerSecond));
            default -> talon.setControl(mmVelVolts.withVelocity(mechanismRotationsPerSecond));
        }
    }

    // ── Open loop ────────────────────────────────────────────────────────────

    /** Direct torque-current output (FOC), amps. Falls back to voltage scaling in VOLTAGE mode. */
    public void setAmps(double amps) {
        if (controlMode == ControlMode.TORQUE_CURRENT_FOC) {
            talon.setControl(torqueOut.withOutput(amps));
        } else {
            // No torque control without FOC — approximate nothing; callers in VOLTAGE mode
            // should use setVolts. Stop instead of guessing.
            talon.setControl(neutral);
        }
    }

    public void setVolts(double volts) {
        talon.setControl(voltsOut.withOutput(volts));
    }

    public void setDutyCycle(double dutyCycle) {
        talon.setControl(dutyOut.withOutput(dutyCycle));
    }

    /** Applies neutral output (brake/coast per configured neutral mode). */
    public void stop() {
        talon.setControl(neutral);
    }

    /** Follows another ExcaTalonFX. */
    public void follow(ExcaTalonFX leader, boolean oppose) {
        talon.setControl(follower
                .withLeaderID(leader.talon.getDeviceID())
                .withMotorAlignment(oppose ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
    }

    // ── Sensors (cached; refreshed by SignalHub.refreshAll once per loop) ────

    public double positionRotations() {
        return position.getValueAsDouble();
    }

    /** Latency-compensated position (extrapolated by velocity over signal latency). */
    public double positionRotationsCompensated() {
        return SignalHub.latencyCompensated(position, velocity);
    }

    public double velocityRotationsPerSecond() {
        return velocity.getValueAsDouble();
    }

    public double volts() {
        return appliedVolts.getValueAsDouble();
    }

    public double supplyAmps() {
        return supplyCurrent.getValueAsDouble();
    }

    public double statorAmps() {
        return statorCurrent.getValueAsDouble();
    }

    public double temperatureCelsius() {
        return temperature.getValueAsDouble();
    }

    /** True while the device is answering CAN frames (per-loop refresh status). */
    public boolean connected() {
        return position.getStatus().isOK();
    }

    // ── Housekeeping ─────────────────────────────────────────────────────────

    /** Overwrites the mechanism position (homing). Blocking config call — use off hot paths. */
    public void resetPosition(double mechanismRotations) {
        talon.setPosition(mechanismRotations);
    }

    /**
     * Sets brake/coast. Blocking CTRE call — {@code Mechanism} wraps this in an async
     * executor; avoid calling directly from the main loop.
     */
    public void setNeutralMode(NeutralModeValue mode) {
        talon.setNeutralMode(mode);
    }

    public String name() {
        return name;
    }

    public ControlMode controlMode() {
        return controlMode;
    }

    /** Escape hatch for sim state and advanced use. */
    public TalonFX unwrap() {
        return talon;
    }
}
