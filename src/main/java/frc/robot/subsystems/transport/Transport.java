package frc.robot.subsystems.transport;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.transport.Constants.*;

public class Transport extends SubsystemBase {
    private TalonFXMotor drumMotor;
    public Mechanism drumMechanism;

    public Transport() {
        drumMotor = new TalonFXMotor(DRUM_MOTOR_ID);
        drumMechanism = new Mechanism(drumMotor);
    }
    public Command manualCommand(DoubleSupplier output) {
        return drumMechanism.manualCommand(output, this);
    }
}
