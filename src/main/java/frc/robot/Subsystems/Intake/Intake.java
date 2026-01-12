package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.controls.compound.Diff_VoltageOut_Velocity;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Arm.Arm;

import java.util.function.DoubleSupplier;

import static frc.robot.Subsystems.Intake.IntakeConstants.*;

public class Intake extends SubsystemBase {

    public final TalonFXMotor fourBarMotor;
    public final TalonFXMotor rollerMotor;
    public final Arm fourBarMechanism;
    public final SoftLimit armVLimit;
    public final Gains armGains;
    public final CANcoder angleEncoder;
    public final DoubleSupplier angleSupplier;
    public final Mass armMass;


    public Intake(){
        angleEncoder = new CANcoder(ANGLE_ENCODER_ID);
        fourBarMotor = new TalonFXMotor(FOUR_BAR_MOTOR_ID);
        rollerMotor = new TalonFXMotor(ROLLER_MOTOR_ID);
        armVLimit = new SoftLimit(() -> ARM_MIN_V_LIMIT,
                                       () -> ARM_MAX_V_LIMIT);
        armGains = new Gains();
        angleSupplier = () -> (angleEncoder.getPosition().getValueAsDouble() * Math.PI * 2);
        armMass = new Mass(() -> Math.cos(angleSupplier.getAsDouble() + ARM_MASS_TO_AXIS),
                           () -> Math.sin(angleSupplier.getAsDouble() + ARM_MASS_TO_AXIS),
                                 ARM_MASS);
        fourBarMechanism = new Arm(fourBarMotor, angleSupplier, armVLimit, armGains, armMass);
    }
}
