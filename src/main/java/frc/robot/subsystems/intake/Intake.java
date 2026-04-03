package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.mechanisms.Mechanism;
import monologue.Annotations.Log;
import monologue.Logged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.intake.Intake.IntakeState.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.ARM_VELOCITY_LIMIT;

public class Intake extends SubsystemBase implements Logged {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public final Mechanism rollerMotorMechanism;
    public final Arm fourBarMechanism;

    public final SoftLimit intakeAngleLimit;
    public boolean isIntakeOpen = false;
    public final DoubleSupplier angleSupplier;
    public final Trigger atPositionTrigger;
    public IntakeState currentState;
    private double targetAngle = 0;
    private double rollerVoltage = 0;

    public Intake(IntakeIO io) {
        this.io = io;
        currentState = IDLE;

        // angleSupplier reads from logged inputs so replay works correctly
        angleSupplier = () -> inputs.angleEncoderAbsolutePositionRotations * (1.0 / 0.29);

        Motor fourBarMotor = io.getFourBarMotor();
        Motor rollerMotor = io.getRollerMotor();

        rollerMotorMechanism = new Mechanism(rollerMotor);

        intakeAngleLimit = new SoftLimit(() -> INTAKE_MIN_ANGLE, () -> INTAKE_MAX_ANGLE);

        atPositionTrigger = new Trigger(() -> (Math.abs(currentState.radPosition - angleSupplier.getAsDouble()) < INTAKE_ANGLE_TOLERANCE));

        Gains ARM_POSITION_GAINS = new Gains(3, 0, 0, 0.45, 0, 0, 0.82);
        fourBarMechanism = new Arm(fourBarMotor, angleSupplier, ARM_VELOCITY_LIMIT, ARM_POSITION_GAINS, new Mass(() -> Math.cos(angleSupplier.getAsDouble()), () -> Math.sin(angleSupplier.getAsDouble()), ARM_MASS));

        setDefaultCommand(defaultCommand());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // Drive mechanisms based on state
        if (currentState == CLOSE) {
            targetAngle = INTAKE_MIN_ANGLE;
            rollerVoltage = 0;
        } else if (currentState == OPEN) {
            targetAngle = INTAKE_MAX_ANGLE;
            // rollerVoltage is managed separately
        }

        // Apply PID control for the arm
        fourBarMechanism.setVoltage(
            fourBarMechanism.getPIDForAngle(() -> intakeAngleLimit.limit(targetAngle))
        );
        rollerMotorMechanism.setVoltage(rollerVoltage);
    }

    public void setClosed() {
        this.currentState = CLOSE;
    }

    public void setIntake() {
        this.currentState = OPEN;
        this.rollerVoltage = 7.0;
    }

    public void setRollerVoltage(double voltage) {
        this.rollerVoltage = voltage;
    }

    public Command setAnglePosition(IntakeState targetPosition) {
        return new InstantCommand(() -> this.currentState = targetPosition);
    }

    public Command setPositionCommand(double angle) {
        return fourBarMechanism.goToAngleCommand(
                intakeAngleLimit.limit(angle),
                at -> {},
                0
        );
    }

    public Command intakeCommand() {
        Command c = setPositionCommand(1).alongWith(rollerManualCommand(7));
        c.addRequirements(this);
        return c;
    }

    public Command closeCommand() {
        Command c = setPositionCommand(0).alongWith(rollerManualCommand(0));
        c.addRequirements(this);
        return c;
    }

    @AutoLogOutput(key = "Intake/IsIntakeOpen")
    @Log.NT
    public boolean getIsIntakeOpen() {
        return isIntakeOpen;
    }

    public Command rollerManualCommand(double voltage) {
        return rollerMotorMechanism.manualCommand(() -> voltage);
    }

    public Command defaultCommand() {
        Command c = Commands.either(
                Commands.idle(this),
                fourBarMechanism.anglePositionControlCommand(
                        () -> intakeAngleLimit.limit(currentState.radPosition),
                        at -> {},
                        MAX_OFFSET
                ),
                () -> currentState.equals(IDLE)
        );

        c.addRequirements(this);
        return c;
    }

    public enum IntakeState {
        CLOSE(INTAKE_MIN_ANGLE), // todo
        IDLE(0), // zero because it doesnt move at all
        OPEN(INTAKE_MAX_ANGLE); // todo

        private final double radPosition;

        IntakeState(double radPosition) {
            this.radPosition = radPosition;
        }
    }

    @AutoLogOutput(key = "Intake/AngleRad")
    @Log.NT
    public double getIntakeAngleSupplier() {
        return angleSupplier.getAsDouble();
    }
}
