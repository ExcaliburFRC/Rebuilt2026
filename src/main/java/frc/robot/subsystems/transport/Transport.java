package frc.robot.subsystems.transport;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;
import frc.excalib.mechanisms.turret.Turret;

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
