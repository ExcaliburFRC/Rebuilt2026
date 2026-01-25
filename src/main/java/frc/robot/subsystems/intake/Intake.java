package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class Intake extends SubsystemBase {

    public final TalonFXMotor fourBarMotor;
    public final TalonFXMotor rollerMotor;
    public final Mechanism rollerMotorMechanism;
    public final Arm fourBarMechanism;
    public final SoftLimit armVLimit;
    public final Gains armGains;
    public final CANcoder angleEncoder;
    public final DoubleSupplier angleSupplier;
    public final Mass armMass;
    public final SoftLimit intakeAngleLimit;
    public final Trigger atPositionTrigger;
    public double targetPosition;

    public Intake() {
        angleEncoder = new CANcoder(ANGLE_ENCODER_ID);
        fourBarMotor = new TalonFXMotor(FOUR_BAR_MOTOR_ID);
        rollerMotor = new TalonFXMotor(ROLLER_MOTOR_ID);
        rollerMotorMechanism = new Mechanism(rollerMotor);
        armVLimit = new SoftLimit(() -> ARM_MIN_VELOCITY_LIMIT,
                () -> ARM_MAX_V_LIMIT);
        armGains = INTAKE_GAINS;
        angleSupplier = () -> (angleEncoder.getPosition().getValueAsDouble() * ROTATION_TO_RAD);
        armMass = new Mass(() -> Math.cos(angleSupplier.getAsDouble() + ARM_MASS_TO_AXIS),
                () -> Math.sin(angleSupplier.getAsDouble() + ARM_MASS_TO_AXIS),
                ARM_MASS);
        intakeAngleLimit = new SoftLimit(() -> INTAKE_MIN_ANGLE, () -> INTAKE_MAX_ANGLE);
        targetPosition = CLOSE_INTAKE_ANGLE;
        atPositionTrigger = new Trigger(
                () -> (Math.abs(targetPosition - angleSupplier.getAsDouble()) < INTAKE_ANGLE_TOLERANCE)
        );
        fourBarMechanism = new Arm(fourBarMotor, angleSupplier, armVLimit, armGains, armMass);

        setDefaultCommand(
                fourBarMechanism.anglePositionControlCommand(
                        () -> intakeAngleLimit.limit(targetPosition),
                        at -> at = false,
                        MAX_OFFSET,
                        this)

        );
    }

    public Command setAnglePosition(double targetPosition) {
        return new InstantCommand(
                () -> this.targetPosition = targetPosition
        );
    }

    public Command rollerManualCommand(double voltage) {
        return rollerMotorMechanism.manualCommand(() -> voltage, this);
    }

    public Command openFloorIntakeCommand() {
        return new ConditionalCommand(
                new ParallelCommandGroup(
                        rollerManualCommand(INTAKE_ROLLER_VOLTAGE),
                        setAnglePosition(FLOOR_INTAKE_ANGLE)
                ),
                new InstantCommand(
                        () -> {
                        }
                ),
                () -> (targetPosition != FLOOR_INTAKE_ANGLE)
        );
    }

    public Command closeIntakeCommand() {
        return new ConditionalCommand(
                new ParallelCommandGroup(
                        rollerManualCommand(STOW_ROLLER_VOLTAGE),
                        setAnglePosition(CLOSE_INTAKE_ANGLE)
                ),
                new InstantCommand(
                        () -> {
                        }
                ),
                () -> (targetPosition != CLOSE_INTAKE_ANGLE)
        );
    }
}