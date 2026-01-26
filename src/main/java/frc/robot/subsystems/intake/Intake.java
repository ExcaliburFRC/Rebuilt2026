package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.ARM_VELOCITY_LIMIT;

public class Intake extends SubsystemBase {

    public final TalonFXMotor fourBarMotor;
    public final TalonFXMotor rollerMotor;

    public final Mechanism rollerMotorMechanism;
    public final Arm fourBarMechanism;

    public final CANcoder angleEncoder;

    public final SoftLimit intakeAngleLimit;

    public final DoubleSupplier angleSupplier;
    public final Trigger atPositionTrigger;
    public TargetAngle targetPosition;

    public Intake() {
        targetPosition = TargetAngle.CLOSE;

        angleEncoder = new CANcoder(ANGLE_ENCODER_ID);
        fourBarMotor = new TalonFXMotor(FOUR_BAR_MOTOR_ID);
        rollerMotor = new TalonFXMotor(ROLLER_MOTOR_ID);

        rollerMotorMechanism = new Mechanism(rollerMotor);

        angleSupplier = () -> (angleEncoder.getPosition().getValueAsDouble() * ROTATION_TO_RAD);

        intakeAngleLimit = new SoftLimit(() -> INTAKE_MIN_ANGLE, () -> INTAKE_MAX_ANGLE);

        atPositionTrigger = new Trigger(
                () -> (
                        Math.abs(targetPosition.radPosition - angleSupplier.getAsDouble())
                                < INTAKE_ANGLE_TOLERANCE
                )
        );

        fourBarMechanism = new Arm(
                fourBarMotor,
                angleSupplier,
                ARM_VELOCITY_LIMIT,
                ARM_POSITION_GAINS,
                new Mass(
                        () -> Math.cos(angleSupplier.getAsDouble()),
                        () -> Math.sin(angleSupplier.getAsDouble()),
                        ARM_MASS
                )
        );

        setDefaultCommand(
                fourBarMechanism.anglePositionControlCommand(
                        () -> intakeAngleLimit.limit(targetPosition.radPosition),
                        at -> at = false,
                        MAX_OFFSET,
                        this
                )
        );
    }

    public Command setAnglePosition(TargetAngle targetPosition) {
        return new InstantCommand(
                () -> this.targetPosition = targetPosition
        );
    }

    public Command rollerManualCommand(double voltage) {
        return rollerMotorMechanism.manualCommand(() -> voltage);
    }

    public Command openIntakeCommand() {
        return new InstantCommand(() -> setAnglePosition(TargetAngle.OPEN));
    }

    public Command closeIntakeCommand() {
        return new InstantCommand(() -> setAnglePosition(TargetAngle.CLOSE));
    }

    public Command intakeCommand() {
        return new StartEndCommand(
                this::openIntakeCommand,
                this::closeIntakeCommand,
                this
        );
    }

    public enum TargetAngle {
        CLOSE(0), // todo
        OPEN(0); // todo

        private final double radPosition;

        TargetAngle(double radPosition) {
            this.radPosition = radPosition;
        }
    }
}