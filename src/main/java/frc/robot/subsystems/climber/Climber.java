package frc.robot.subsystems.climber;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.SparkMaxMotor;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.mechanisms.Mechanism;
import frc.excalib.mechanisms.linear_extension.LinearExtension;
import frc.robot.subsystems.ClimberConstant;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.ClimberConstant.*;

public class Climber extends SubsystemBase implements Logged{
    private final LinearExtension m_left, m_right;
    public Climber() {
        Motor rightMotor = new SparkMaxMotor(RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushed);
        Motor leftMotor = new SparkMaxMotor(LEFT_MOTOR_ID,SparkLowLevel.MotorType.kBrushed);
        TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_VELOCITY_LIMIT,MAX_ACCELERATION_LIMIT);
        DoubleSupplier rightMotorAngleSupplier = () -> 0;
        DoubleSupplier leftMotorAngleSupplier = () -> 0;
        m_right = new LinearExtension(
                rightMotor,
                rightMotor::getMotorPosition,
                rightMotorAngleSupplier,
                new Gains(0,0,0,0,0,0,0),
                CONSTRAINTS,
                RIGHT_MOTOR_TOLERANCE
                );
        m_left = new LinearExtension(
                leftMotor,
                leftMotor::getMotorPosition,
                leftMotorAngleSupplier,
                new Gains(0,0,0,0,0,0,0),
                CONSTRAINTS,
                LEFT_MOTOR_TOLERANCE
                );

    }
    public Command manualCommand(DoubleSupplier leftOutput, DoubleSupplier rightOutput) {
        Command manualCommand = new ParallelCommandGroup(
                m_left.manualCommand(leftOutput),
                m_right.manualCommand(rightOutput)
        );
        manualCommand.addRequirements(this);
        return manualCommand;
    }

    public Command setLengthsCommand(DoubleSupplier rightLengthSetPoint, DoubleSupplier leftLengthSetPoint) {
        Command setLengthsCommand = new ParallelCommandGroup(
                m_right.extendCommand(rightLengthSetPoint),
                m_left.extendCommand(leftLengthSetPoint)
        );
        setLengthsCommand.addRequirements(this);
        return setLengthsCommand;
    }
}
