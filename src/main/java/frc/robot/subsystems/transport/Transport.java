package frc.robot.subsystems.transport;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.transport.TransportConstants.*;
import static frc.robot.superstructure.Superstructure.CANivoreBus;

public class Transport extends SubsystemBase {
    private final TalonFXMotor drumMotor;
    public Mechanism drumMechanism;

    public Transport() {
        drumMotor = new TalonFXMotor(DRUM_MOTOR_ID, CANivoreBus);
        drumMechanism = new Mechanism(drumMotor);
    }
    public Command manualCommand(DoubleSupplier output) {
        return drumMechanism.manualCommand(output, this);
    }
}
