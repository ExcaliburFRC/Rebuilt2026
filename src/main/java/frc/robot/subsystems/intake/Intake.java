package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.mechanisms.Mechanism;
import monologue.Annotations.Log;
import monologue.Logged;

import java.sql.SQLOutput;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.ARM_VELOCITY_LIMIT;
import static frc.robot.subsystems.intake.IntakeStates.CLOSE;
import static frc.robot.subsystems.intake.IntakeStates.PUMP;

public class Intake extends SubsystemBase implements Logged {

    public final TalonFXMotor fourBarMotorLeft, fourBarMotorRight;
    public final TalonFXMotor rollerMotor;
    public final MotorGroup armMotorGroup;

    public final Mechanism rollerMechanism;
    public final Arm fourBarMechanism;

    public final CANcoder angleEncoder;

    public final SoftLimit intakeAngleLimit;
    public boolean isIntakeOpen = false;
    public final DoubleSupplier angleSupplier;
    public final Trigger atPositionTrigger;

    public IntakeStates currentState;

    public Intake() {
        currentState = CLOSE;

        fourBarMotorLeft = new TalonFXMotor(LEFT_FOUR_BAR_MOTOR_ID, SUBSYSTEMS_CANBUS);
        fourBarMotorLeft.setInverted(DirectionState.FORWARD);

        fourBarMotorRight = new TalonFXMotor(RIGHT_FOUR_BAR_MOTOR_ID, SUBSYSTEMS_CANBUS);
        fourBarMotorRight.setInverted(DirectionState.REVERSE);

        angleEncoder = new CANcoder(ANGLE_ENCODER_ID, SUBSYSTEMS_CANBUS);

        rollerMotor = new TalonFXMotor(ROLLER_MOTOR_ID, SUBSYSTEMS_CANBUS);
        rollerMotor.setCurrentLimit(80, 80);

        rollerMechanism = new Mechanism(rollerMotor);

        intakeAngleLimit = new SoftLimit(() -> INTAKE_MIN_ANGLE, () -> INTAKE_MAX_ANGLE);


        this.armMotorGroup = new MotorGroup(fourBarMotorLeft, fourBarMotorRight);

        this.armMotorGroup.setPositionConversionFactor(2 * Math.PI / 14.14);
        this.armMotorGroup.setVelocityConversionFactor(2 * Math.PI / 14.14);

        this.armMotorGroup.setMotorPosition(Math.PI - 0.732601 - 0.159);

        angleSupplier = armMotorGroup::getMotorPosition;


        atPositionTrigger = new Trigger(() -> (Math.abs(currentState.angle - angleSupplier.getAsDouble()) < INTAKE_ANGLE_TOLERANCE));

        fourBarMechanism = new Arm(
                this.armMotorGroup,
                angleSupplier,
                ARM_VELOCITY_LIMIT,
                ARM_POSITION_GAINS,
                new Mass(
                        () -> Math.cos(angleSupplier.getAsDouble()),
                        () -> Math.sin(angleSupplier.getAsDouble()),
                        ARM_MASS
                )
        );

        setDefaultCommand(defaultCommand());
    }

    public Command setStateCommand(IntakeStates stateToSet) {
        return new InstantCommand(() -> this.currentState = stateToSet);
    }

    public Command setPositionCommand(DoubleSupplier angle) {
        return fourBarMechanism.anglePositionControlCommand(
                () -> intakeAngleLimit.limit(angle.getAsDouble()),
                (at) -> at = false,
                0
        );
    }

    public Command fowardIntake() {
        return fourBarMechanism.manualCommand(() -> 0.5, this);
    }

    public Command reverseIntake() {
        return fourBarMechanism.manualCommand(() -> -0.5, this);
    }

    @Log.NT
    public boolean getIsIntakeOpen() {
        return isIntakeOpen;
    }

    public Command defaultCommand() {
        Command defaultCommand =
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                rollerMechanism.manualCommand(() -> this.currentState.voltage),
                                setPositionCommand(() -> currentState.angle)
                        ).until(() -> currentState.equals(PUMP)),
                        rollerMechanism.manualCommand(() -> this.currentState.voltage)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                setPositionCommand(() -> 2).withTimeout(0.5),
                                                setPositionCommand(() -> 0).withTimeout(0.5))
                                                .repeatedly())
                                .until(() -> !currentState.equals(PUMP)),
                        () -> !currentState.equals(PUMP)
                );
        defaultCommand.addRequirements(this);
        return defaultCommand;
    }

    @Log.NT
    public boolean atPositionTrigger() {
        return atPositionTrigger.getAsBoolean();
    }

    @Log.NT
    public double getIntakeAngleSupplier() {
        return angleSupplier.getAsDouble();
    }

    @Log.NT
    public String getCurrentIntakeState() {
        return currentState.name();
    }

    @Log.NT
    public double getCurrentVoltage() {
        return this.currentState.voltage;
    }
}