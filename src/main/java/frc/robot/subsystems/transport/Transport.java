package frc.robot.subsystems.transport;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.turret.Turret;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.transport.Constants.*;

public class Transport extends SubsystemBase {
    private TalonFXMotor drumMotor;
    private CANcoder drumEncoder;
    public Turret drumMechanism;
    DoubleSupplier angleSupplier;
    public Trigger atPositionTrigger;


    public Transport() {
        drumMotor = new TalonFXMotor(DRUM_MOTOR_ID);
        drumEncoder = new CANcoder(ENCODER_ID);
        drumMechanism = new Turret(drumMotor,
                new ContinuousSoftLimit(() -> Double.NEGATIVE_INFINITY, () -> Double.POSITIVE_INFINITY),
                GAINS, PIDTOLERANCE, angleSupplier);
    }

    public Command manualCommand(DoubleSupplier output) {
        return drumMechanism.manualCommand(output, this);
    }

    public Command shootCommand(){
        return manualCommand(()-> SHOOTING_VOLTAGE);
    }

    public Command shootCommand(DoubleSupplier output){
        return manualCommand(output);
    }

    public Command defaultCommand() {
        return manualCommand(()->DEFAULT_VOLTAGE);
    }
}
