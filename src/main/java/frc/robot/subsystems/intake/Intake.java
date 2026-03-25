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

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.ARM_VELOCITY_LIMIT;
import static frc.robot.subsystems.intake.IntakeStates.CLOSE;

public class Intake extends SubsystemBase implements Logged {

    public final TalonFXMotor fourBarMotorLeft, fourBarMotorRight;
    public final TalonFXMotor rollerMotor;

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
        fourBarMotorLeft.setInverted(DirectionState.REVERSE);

        fourBarMotorRight = new TalonFXMotor(RIGHT_FOUR_BAR_MOTOR_ID, SUBSYSTEMS_CANBUS);
        fourBarMotorRight.setInverted(DirectionState.REVERSE);

        angleEncoder = new CANcoder(ANGLE_ENCODER_ID, SUBSYSTEMS_CANBUS);

        rollerMotor = new TalonFXMotor(ROLLER_MOTOR_ID, SUBSYSTEMS_CANBUS);
        rollerMotor.setCurrentLimit(80, 80);

        fourBarMotorLeft.setPositionConversionFactor((double) 1 / 0.29);
        fourBarMotorRight.setPositionConversionFactor((double) 1 / 0.29);

        rollerMechanism = new Mechanism(rollerMotor);

        angleSupplier = () -> (angleEncoder.getAbsolutePosition().getValueAsDouble() * (1 / 0.29));

        intakeAngleLimit = new SoftLimit(() -> INTAKE_MIN_ANGLE, () -> INTAKE_MAX_ANGLE);

        atPositionTrigger = new Trigger(() -> (Math.abs(currentState.angle - angleSupplier.getAsDouble()) < INTAKE_ANGLE_TOLERANCE));

        fourBarMechanism = new Arm(
                new MotorGroup(fourBarMotorRight, fourBarMotorLeft),
                angleSupplier,
                ARM_VELOCITY_LIMIT,
                ARM_POSITION_GAINS,
                new Mass(
                        () -> Math.cos(angleSupplier.getAsDouble()),
                        () -> Math.sin(angleSupplier.getAsDouble()),
                        ARM_MASS
                )
        );

//        setDefaultCommand(defaultCommand().unless(()-> DISABLE_SUBSYSTEMS));
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

    @Log.NT
    public boolean getIsIntakeOpen() {
        return isIntakeOpen;
    }

    public Command rollerManualCommand(DoubleSupplier voltage) {
        return new RunCommand(() -> rollerMechanism.manualCommand(voltage));
    }

    public Command defaultCommand() {
        Command defaultCommand = new ParallelCommandGroup(
                rollerMechanism.manualCommand(() -> this.currentState.voltage),
                setPositionCommand(() -> currentState.angle)
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