package frc.robot.subsystems.transport;

import edu.wpi.first.wpilibj2.command.*;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import monologue.Annotations;
import monologue.Annotations.Log;
import monologue.Annotations.Log.NT;
import monologue.Logged;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.transport.TransportConstants.*;

public class Transport extends SubsystemBase implements Logged {
    private final TalonFXMotor drumMotor, transportMotor;
    public FlyWheel drumMechanism, transportMechanism;
    private TransportStates currentState = TransportStates.IDLE;

    public Transport(BooleanSupplier shouldTransport) {
        drumMotor = new TalonFXMotor(DRUM_MOTOR_ID, SUBSYSTEMS_CANBUS);
        drumMechanism = new FlyWheel(drumMotor, MAX_ACCELERATION, MAX_JERK, DRUM_GAINS);

        transportMotor = new TalonFXMotor(TRANSPORT_MOTOR_ID, SUBSYSTEMS_CANBUS);

        drumMotor.setCurrentLimit(80, 80);
        transportMotor.setCurrentLimit(80, 80);

        transportMotor.setIdleState(IdleState.BRAKE);
        drumMotor.setIdleState(IdleState.BRAKE);
        drumMotor.setInverted(DirectionState.REVERSE);
        transportMotor.setInverted(DirectionState.REVERSE);

        transportMechanism = new FlyWheel(transportMotor, 10, 10, TRANSPORT_PID_GAINS);
        drumMotor.setVelocityConversionFactor(0.39898 / 9);
        transportMotor.setVelocityConversionFactor(0.0731);

        setDefaultCommand(defaultCommand());
    }


    public Command manualCommand(DoubleSupplier outputDrum, DoubleSupplier outputTransport) {
        Command command = drumMechanism.manualCommand(outputDrum).alongWith(transportMechanism.manualCommand(outputTransport));
        command.addRequirements(this);
        return command;
    }

    public Command defaultCommand() {
        return new ParallelCommandGroup(
                drumMechanism.setDynamicVelocityCommand(
                        () -> this.currentState.linearVelocity),
                transportMechanism.setDynamicVelocityCommand(
                        () -> this.currentState.linearVelocity)
        );
    }

    public Command setStateCommand(TransportStates stateToSet) {
        return new InstantCommand(() -> currentState = stateToSet, this);
    }
}
