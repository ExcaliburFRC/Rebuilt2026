package frc.robot.subsystems.transport;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
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
    //    private final PIDController transportVelocityController;
    private final BooleanSupplier shouldTransport;
    private double transportVelocitySetpoint = 30;

    public Transport(BooleanSupplier shouldTransport) {
        drumMotor = new TalonFXMotor(DRUM_MOTOR_ID, SUBSYSTEMS_CANBUS);
        drumMechanism = new FlyWheel(drumMotor, MAX_ACCELERATION, MAX_JERK, DRUM_GAINS);

        transportMotor = new TalonFXMotor(TRANSPORT_MOTOR_ID, SUBSYSTEMS_CANBUS);

        this.shouldTransport = shouldTransport;
        drumMotor.setCurrentLimit(20, 20);
        transportMotor.setCurrentLimit(20, 20);


        drumMotor.setInverted(DirectionState.REVERSE);
        transportMotor.setInverted(DirectionState.REVERSE);

        transportMechanism = new FlyWheel(transportMotor, 10, 10, TRANSPORT_PID_GAINS);
        drumMotor.setVelocityConversionFactor(0.39898);
        transportMotor.setVelocityConversionFactor(0.0731);

//        transportVelocityController = new PIDController(TRANSPORT_PID_GAINS.kp, TRANSPORT_PID_GAINS.ki, TRANSPORT_PID_GAINS.kd);

        setDefaultCommand(transportFuelCommand());
    }


    public Command manualCommand(DoubleSupplier outputDrum, DoubleSupplier outputTransport) {
        Command command = drumMechanism.manualCommand(outputDrum).alongWith(transportMechanism.manualCommand(outputTransport));
        command.addRequirements(this);
        return command;
    }

    public Command transportFuelCommand() {
        return new RunCommand(
                () -> {
                    if (true) {
                        transportMechanism.setDynamicVelocity(transportVelocitySetpoint);
                        drumMechanism.setDynamicVelocity(transportVelocitySetpoint);
                    }
                },
                this
        );
    }

    @NT
    public double getSetpoint() {
        return transportVelocitySetpoint;
    }
}
