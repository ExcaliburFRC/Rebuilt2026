package frc.robot.subsystems.transport;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import frc.robot.lib.StateMachineSubsystem;
import monologue.Annotations.Log;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.DISABLE_SUBSYSTEMS;
import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.transport.TransportConstants.*;

/**
 * Drum + feeder that moves fuel up to the shooter. Runs at its state velocity only while the shooter
 * reports it is ready to receive (gated by the {@code isShooterReadyForTransport} supplier).
 */
public class Transport extends StateMachineSubsystem<TransportStates> {
    private final TalonFXMotor drumMotor, transportMotor;
    private final FlyWheel drumMechanism, transportMechanism;
    private final Trigger atPositionTrigger;
    private final BooleanSupplier isShooterReadyForTransport;

    public Transport(BooleanSupplier isShooterReadyForTransport) {
        super(TransportStates.IDLE);
        this.isShooterReadyForTransport = isShooterReadyForTransport;

        drumMotor = new TalonFXMotor(DRUM_MOTOR_ID, SUBSYSTEMS_CANBUS);
        transportMotor = new TalonFXMotor(TRANSPORT_MOTOR_ID, SUBSYSTEMS_CANBUS);

        drumMotor.setCurrentLimit(80, 80);
        transportMotor.setCurrentLimit(80, 80);
        drumMotor.setIdleState(IdleState.BRAKE);
        transportMotor.setIdleState(IdleState.BRAKE);
        drumMotor.setInverted(DirectionState.REVERSE);
        transportMotor.setInverted(DirectionState.REVERSE);
        drumMotor.setVelocityConversionFactor(DRUM_VELOCITY_CONVERSION_FACTOR);
        transportMotor.setVelocityConversionFactor(TRANSPORT_VELOCITY_CONVERSION_FACTOR);

        drumMechanism = new FlyWheel(drumMotor, MAX_ACCELERATION, MAX_JERK, DRUM_GAINS);
        transportMechanism = new FlyWheel(transportMotor, TRANSPORT_MAX_ACCELERATION, TRANSPORT_MAX_JERK, TRANSPORT_PID_GAINS);

        atPositionTrigger = new Trigger(
                () -> Math.abs(transportMechanism.getVelocity() - state().linearVelocity()) < TRANSPORT_TOLERANCE &&
                        Math.abs(drumMechanism.getVelocity() - state().linearVelocity()) < DRUM_TOLERANCE
        );

        setDefaultCommand(defaultCommand().unless(() -> DISABLE_SUBSYSTEMS));
    }

    /** @return the velocity the rollers should target this loop: the state velocity, gated on shooter readiness. */
    private double gatedVelocity() {
        return isShooterReadyForTransport.getAsBoolean() ? state().linearVelocity() : 0;
    }

    public Command defaultCommand() {
        Command command = new ParallelCommandGroup(
                drumMechanism.setDynamicVelocityCommand(this::gatedVelocity),
                transportMechanism.setDynamicVelocityCommand(this::gatedVelocity)
        );
        command.addRequirements(this);
        return command;
    }

    public Command manualCommand(DoubleSupplier outputDrum, DoubleSupplier outputTransport) {
        Command command = drumMechanism.manualCommand(outputDrum).alongWith(transportMechanism.manualCommand(outputTransport));
        command.addRequirements(this);
        return command;
    }

    public Command manualTransport() {
        Command command = new ParallelCommandGroup(
                drumMechanism.setDynamicVelocityCommand(() -> MANUAL_TRANSPORT_VELOCITY),
                transportMechanism.setDynamicVelocityCommand(() -> MANUAL_TRANSPORT_VELOCITY)
        );
        command.addRequirements(this);
        return command;
    }

    public Trigger atPositionTrigger() {
        return atPositionTrigger;
    }

    @Log.NT
    public String getCurrentTransportState() {
        return currentStateName();
    }
}
