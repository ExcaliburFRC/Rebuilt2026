package frc.robot.subsystems.transport;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib2.control.ControlMode;
import frc.excalib2.device.CANDeviceId;
import frc.excalib2.mechanisms.MechanismConfig;
import frc.excalib2.mechanisms.VelocityMechanism;
import frc.excalib2.statemachine.StateMachine;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.transport.TransportConstants.*;
import static frc.robot.subsystems.transport.TransportStates.IDLE;
import static frc.robot.subsystems.transport.TransportStates.TRANSPORT;

/**
 * Transport on ExcaLib v2: two {@link VelocityMechanism}s (drum + transport belt) driven by
 * a two-state machine. Velocities only flow while the shooter reports ready — same gating
 * as the pre-migration subsystem.
 */
public class Transport extends SubsystemBase {

    private final VelocityMechanism drum = new VelocityMechanism(
            MechanismConfig.of("Transport/Drum", new CANDeviceId(DRUM_MOTOR_ID))
                    .controlMode(ControlMode.VOLTAGE) // ported v1 volts gains; FOC after SysId
                    .inverted(false)
                    .gains(DRUM_GAINS, SIM_GAINS)
                    .currentBudget(CURRENT_BUDGET)
                    .tolerance(edu.wpi.first.units.Units.Rotations.of(DRUM_TOLERANCE_RPS))
                    .simModel(DRUM_SIM_MODEL));

    private final VelocityMechanism transport = new VelocityMechanism(
            MechanismConfig.of("Transport/Belt", new CANDeviceId(TRANSPORT_MOTOR_ID))
                    .controlMode(ControlMode.VOLTAGE)
                    .inverted(true)
                    .gains(TRANSPORT_GAINS, SIM_GAINS)
                    .currentBudget(CURRENT_BUDGET)
                    .tolerance(edu.wpi.first.units.Units.Rotations.of(TRANSPORT_TOLERANCE_RPS))
                    .simModel(TRANSPORT_SIM_MODEL));

    private final StateMachine<TransportStates> machine;
    private final BooleanSupplier shooterReady;

    public Transport(BooleanSupplier isShooterReadyForTransport) {
        this.shooterReady = isShooterReadyForTransport;

        machine = StateMachine.builder("Transport", IDLE)
                .whileIn(IDLE, run(this::applyCurrentState))
                .whileIn(TRANSPORT, run(this::applyCurrentState))
                .transitionFromAny(IDLE)
                .transitionFromAny(TRANSPORT)
                .build();
    }

    /** Velocity flows only while the shooter is ready — otherwise both mechanisms hold 0. */
    private void applyCurrentState() {
        double legacyVelocity = shooterReady.getAsBoolean()
                ? machine.getCurrentState().linearVelocity
                : 0;
        drum.setVelocityRotationsPerSecond(legacyVelocity / DRUM_LEGACY_UNITS_PER_ROTATION);
        transport.setVelocityRotationsPerSecond(legacyVelocity / TRANSPORT_LEGACY_UNITS_PER_ROTATION);
    }

    public Command setStateCommand(TransportStates state) {
        return machine.requestCommand(state);
    }

    /** Both mechanisms at their commanded speed (semantics preserved from v1). */
    public Trigger atPositionTrigger() {
        return drum.atSpeed.and(transport.atSpeed);
    }

    /** Fixed-speed manual feed (5 legacy units, from v1's manualTransport). */
    public Command manualTransport() {
        return run(() -> {
            drum.setVelocityRotationsPerSecond(5 / DRUM_LEGACY_UNITS_PER_ROTATION);
            transport.setVelocityRotationsPerSecond(5 / TRANSPORT_LEGACY_UNITS_PER_ROTATION);
        });
    }

    public Command manualCommand(DoubleSupplier drumOutput, DoubleSupplier transportOutput) {
        return run(() -> {
            drum.setVolts(drumOutput.getAsDouble() * 12);
            transport.setVolts(transportOutput.getAsDouble() * 12);
        });
    }

    @Override
    public void periodic() {
        drum.periodic();
        transport.periodic();
        machine.periodic();
    }
}
