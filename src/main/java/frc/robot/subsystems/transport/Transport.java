package frc.robot.subsystems.transport;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import frc.robot.Constants;
import monologue.Annotations;
import monologue.Logged;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;
import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.transport.TransportConstants.*;
import static monologue.Annotations.*;

public class Transport extends SubsystemBase implements Logged {
    private final TalonFXMotor drumMotor, transportMotor;
    public FlyWheel drumMechanism, transportMechanism;
    private TransportStates currentState = TransportStates.IDLE;
    private Trigger atPositionTrigger;
    private final BooleanSupplier isShooterReadyForTransport;

    public Transport(BooleanSupplier isShooterReadyForTransport) {
        drumMotor = new TalonFXMotor(DRUM_MOTOR_ID, SUBSYSTEMS_CANBUS);
        drumMechanism = new FlyWheel(drumMotor, MAX_ACCELERATION, MAX_JERK, DRUM_GAINS);

        transportMotor = new TalonFXMotor(TRANSPORT_MOTOR_ID, SUBSYSTEMS_CANBUS);

        drumMotor.setCurrentLimit(80, 80);
        transportMotor.setCurrentLimit(80, 80);

        transportMotor.setIdleState(IdleState.BRAKE);
        drumMotor.setIdleState(IdleState.BRAKE);
        drumMotor.setInverted(DirectionState.FORWARD);
        transportMotor.setInverted(DirectionState.REVERSE);

        this.isShooterReadyForTransport = isShooterReadyForTransport;

        transportMechanism = new FlyWheel(transportMotor, 10, 10, TRANSPORT_PID_GAINS);
        drumMotor.setVelocityConversionFactor(0.39898 / 9);
        transportMotor.setVelocityConversionFactor(0.0731);

        atPositionTrigger = new Trigger(
                () -> Math.abs(transportMechanism.getVelocity() - currentState.linearVelocity) < TRANSPORT_TOLERANCE &&
                        Math.abs(drumMechanism.getVelocity() - currentState.linearVelocity) < DRUM_TOLERANCE

        );

        setDefaultCommand(defaultCommand().unless(() -> DISABLE_SUBSYSTEMS));
    }


    public Command manualCommand(DoubleSupplier outputDrum, DoubleSupplier outputTransport) {
        Command command = drumMechanism.manualCommand(outputDrum).alongWith(transportMechanism.manualCommand(outputTransport));
        command.addRequirements(this);
        return command;
    }

    public Command defaultCommand() {
        Command defaultCommand = new ParallelCommandGroup(
                drumMechanism.setDynamicVelocityCommand(
                        () ->
                                isShooterReadyForTransport.getAsBoolean() ?
                                        this.currentState.linearVelocity :
                                        0
                ),
                transportMechanism.setDynamicVelocityCommand(
                        () -> isShooterReadyForTransport.getAsBoolean() ?
                                this.currentState.linearVelocity :
                                0
                )
        );
        defaultCommand.addRequirements(this);
        return defaultCommand;
    }

    public Command setStateCommand(TransportStates stateToSet) {
        return new InstantCommand(() -> currentState = stateToSet);
    }

    public Command manualTransport() {
        Command defaultCommand = new ParallelCommandGroup(
                drumMechanism.setDynamicVelocityCommand(() -> 5),
                transportMechanism.setDynamicVelocityCommand(() -> 5)
        );
        defaultCommand.addRequirements(this);
        return defaultCommand;
    }

    public Trigger atPositionTrigger() {
        return atPositionTrigger;
    }

    @Log.NT
    public String getCurrentTransportState() {
        return currentState.name();
    }
}
