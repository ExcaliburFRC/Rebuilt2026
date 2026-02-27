package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.mechanisms.Mechanism;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.intake.Intake.IntakeState.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.ARM_VELOCITY_LIMIT;

public class Intake extends SubsystemBase implements Logged {

    public final TalonFXMotor fourBarMotor;
    public final TalonFXMotor rollerMotor;


    public final Mechanism rollerMotorMechanism;
    public final Arm fourBarMechanism;

    public final CANcoder angleEncoder;

    public final SoftLimit intakeAngleLimit;
    public boolean isIntakeOpen = false;
    public final DoubleSupplier angleSupplier;
    public final Trigger atPositionTrigger;
    public IntakeState currentState;

    public Intake() {
        currentState = IDLE;

        angleEncoder = new CANcoder(ANGLE_ENCODER_ID, SUBSYSTEMS_CANBUS);
        fourBarMotor = new TalonFXMotor(FOUR_BAR_MOTOR_ID, SUBSYSTEMS_CANBUS);
         fourBarMotor.setInverted(DirectionState.REVERSE);
        rollerMotor = new TalonFXMotor(ROLLER_MOTOR_ID, SUBSYSTEMS_CANBUS);

        rollerMotorMechanism = new Mechanism(rollerMotor);

        angleSupplier = () -> (angleEncoder.getAbsolutePosition().getValueAsDouble() * (1 / 0.29));

        intakeAngleLimit = new SoftLimit(() -> INTAKE_MIN_ANGLE, () -> INTAKE_MAX_ANGLE);

        atPositionTrigger = new Trigger(() -> (Math.abs(currentState.radPosition - angleSupplier.getAsDouble()) < INTAKE_ANGLE_TOLERANCE));

        Gains ARM_POSITION_GAINS = new Gains(3, 0, 0, 0.45, 0, 0,0.82);
        fourBarMechanism = new Arm(fourBarMotor, angleSupplier, ARM_VELOCITY_LIMIT, ARM_POSITION_GAINS, new Mass(() -> Math.cos(angleSupplier.getAsDouble()), () -> Math.sin(angleSupplier.getAsDouble()), ARM_MASS));

//        setDefaultCommand(defaultCommand());
    }

    public Command setAnglePosition(IntakeState targetPosition) {
        return new InstantCommand(() -> this.currentState = targetPosition);
    }

    public Command setPositionCommand(double angle) {
        return fourBarMechanism.goToAngleCommand(
                intakeAngleLimit.limit(angle),
                (at) -> at = false,
                0,
                this
        );
    }

    @Log.NT
    public boolean getIsIntakeOpen() {
        return isIntakeOpen;
    }

    public Command rollerManualCommand(double voltage) {
        return rollerMotorMechanism.manualCommand(() -> voltage);
    }

    public Command defaultCommand() {
        Command c = new ConditionalCommand(
                Commands.none(),
                fourBarMechanism.anglePositionControlCommand(
                        () -> intakeAngleLimit.limit(currentState.radPosition),
                        at -> at = false,
                        MAX_OFFSET
                ),
                () -> currentState.equals(IDLE)
        );

        c.addRequirements(this);
        return c;
    }

    public enum IntakeState {
        CLOSE(INTAKE_MIN_ANGLE), // todo
        IDLE(0), // zero becuase it doesnt move at all
        OPEN(INTAKE_MAX_ANGLE); // todo

        private final double radPosition;

        IntakeState(double radPosition) {
            this.radPosition = radPosition;
        }
    }

    @Log.NT
    public double getIntakeAngleSupplier() {
        return angleSupplier.getAsDouble();
    }
}