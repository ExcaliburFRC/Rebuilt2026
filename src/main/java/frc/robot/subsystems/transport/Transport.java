package frc.robot.subsystems.transport;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import monologue.Logged;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.transport.TransportConstants.*;

public class Transport extends SubsystemBase implements Logged {
    private final TalonFXMotor drumMotor;
    public FlyWheel drumMechanism;

    public Transport() {
        drumMotor = new TalonFXMotor(DRUM_MOTOR_ID, SUBSYSTEMS_CANBUS);
        drumMechanism = new FlyWheel(drumMotor, MAX_ACCELERATION, MAX_JERK, GAINS);

        setDefaultCommand(transportFuelCommand());
    }

    public Command manualCommand(DoubleSupplier output) {
        return drumMechanism.manualCommand(output, this);
    }

    public Command transportFuelCommand() {
        return drumMechanism.setDynamicVelocityCommand(() -> 0, this);
    }

    /**
     * Runs the drum at TRANSPORT_VELOCITY while the gate supplier returns true,
     * otherwise holds at 0.  Used so the drum only feeds when the shooter is ready.
     */
    public Command gatedTransportCommand(BooleanSupplier gate) {
        return new RunCommand(
                () -> drumMechanism.setDynamicVelocity(gate.getAsBoolean() ? TRANSPORT_VELOCITY : 0),
                this);
    }

    /**
     * Brief reverse pulse followed by a short forward burst to clear jams in the drum.
     */
    public Command unjamCommand() {
        return new SequentialCommandGroup(
                drumMechanism.manualCommand(() -> -TRANSPORT_VOLTAGE, this).withTimeout(0.35),
                drumMechanism.manualCommand(() -> TRANSPORT_VOLTAGE, this).withTimeout(0.25)
        );
    }
}
