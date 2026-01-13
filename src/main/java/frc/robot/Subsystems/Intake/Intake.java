package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.controls.compound.Diff_VoltageOut_Velocity;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class Intake extends SubsystemBase {

    public final TalonFXMotor fourBarMotor;
    public final TalonFXMotor rollerMotor;
    public final Arm fourBarMechanism;
    public final SoftLimit armVLimit;
    public final Gains armGains;
    public final CANcoder angleEncoder;
    public final DoubleSupplier angleSupplier;
    public final Mass armMass;
    public final SoftLimit intakeMaxAngle;
    public final Trigger isOpenTrigger;
    public final Trigger isCloseTrigger;


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
        intakeMaxAngle = new SoftLimit(()-> INTAKE_MIN_ANGLE,()-> INTAKE_MAX_ANGLE);
        isOpenTrigger = new Trigger(()-> Math.abs( FLOOR_INTAKE_ANGLE - angleEncoder.getPosition().getValueAsDouble()
                * 2 * Math.PI) < FLOOR_INTAKE_ANGLE_TOLERANCE);
        isCloseTrigger = new Trigger(()-> Math.abs( CLOSE_INTAKE_ANGLE - angleEncoder.getPosition().getValueAsDouble()
                * 2 * Math.PI) < CLOSE_INTAKE_ANGLE_TOLERANCE);
        fourBarMechanism = new Arm(fourBarMotor, angleSupplier, armVLimit, armGains, armMass);
    }
    public Command openIntakeCommand(){
        Command openIntakeCommand = fourBarMechanism.anglePositionControlCommand(
                ()-> FLOOR_INTAKE_ANGLE,
                at -> at = false,
                MAX_OFFSET,
                this);
        return openIntakeCommand;
    }
    public Command closeIntakeCommand(){
        Command closeIntakeCommand = fourBarMechanism.anglePositionControlCommand(
                ()-> CLOSE_INTAKE_ANGLE,
                at -> at = false,
                MAX_OFFSET,
                this);
        return closeIntakeCommand;
    }

}
