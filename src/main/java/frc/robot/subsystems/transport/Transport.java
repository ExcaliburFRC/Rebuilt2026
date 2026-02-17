package frc.robot.subsystems.transport;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.transport.transportConstans.*;

public class Transport extends SubsystemBase {
    private final TalonFXMotor drumMotor;
    public Mechanism drumMechanism;

    public Transport() {
        drumMotor = new TalonFXMotor(DRUM_MOTOR_ID, new CANBus("Subsystems"));
        drumMechanism = new FlyWheel(drumMotor, MAX_ACCELERATION, MAX_JERK, gains);
    }

    public Command manualCommand(DoubleSupplier output) {
        return drumMechanism.manualCommand(output, this);
    }
}
