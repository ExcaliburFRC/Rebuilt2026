package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.mechanisms.Mechanism;
import frc.robot.lib.StateMachineSubsystem;
import monologue.Annotations.Log;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.intake.IntakeConstants.*;

/**
 * Four-bar arm + roller intake. Either holds its state's arm goal while running the roller, or — in
 * the PUMP state — repeatedly strokes the arm to dislodge stuck fuel.
 */
public class Intake extends StateMachineSubsystem<IntakeStates> {
    private final TalonFXMotor fourBarMotorLeft, fourBarMotorRight, rollerMotor;
    private final MotorGroup armMotorGroup;
    private final CANcoder angleEncoder;

    private final Mechanism rollerMechanism;
    private final Arm fourBarMechanism;
    private final SoftLimit intakeAngleLimit;
    private final DoubleSupplier angleSupplier;
    private final Trigger atPositionTrigger;

    private boolean isIntakeOpen = false;

    public Intake() {
        super(IntakeStates.CLOSE);

        fourBarMotorLeft = new TalonFXMotor(LEFT_FOUR_BAR_MOTOR_ID, SUBSYSTEMS_CANBUS);
        fourBarMotorLeft.setInverted(DirectionState.FORWARD);
        fourBarMotorLeft.setCurrentLimit(80, 60);

        fourBarMotorRight = new TalonFXMotor(RIGHT_FOUR_BAR_MOTOR_ID, SUBSYSTEMS_CANBUS);
        fourBarMotorRight.setInverted(DirectionState.REVERSE);
        fourBarMotorRight.setCurrentLimit(80, 60);

        angleEncoder = new CANcoder(ANGLE_ENCODER_ID, SUBSYSTEMS_CANBUS);

        rollerMotor = new TalonFXMotor(ROLLER_MOTOR_ID, SUBSYSTEMS_CANBUS);
        rollerMotor.setCurrentLimit(80, 30);
        rollerMechanism = new Mechanism(rollerMotor);

        intakeAngleLimit = new SoftLimit(() -> INTAKE_MIN_ANGLE, () -> INTAKE_MAX_ANGLE);

        armMotorGroup = new MotorGroup(fourBarMotorLeft, fourBarMotorRight);
        armMotorGroup.setIdleState(IdleState.COAST);
        armMotorGroup.setPositionConversionFactor(ARM_POSITION_CONVERSION_FACTOR);
        armMotorGroup.setVelocityConversionFactor(ARM_POSITION_CONVERSION_FACTOR);
        armMotorGroup.setMotorPosition(ARM_STARTING_ANGLE);

        angleSupplier = armMotorGroup::getMotorPosition;
        atPositionTrigger = new Trigger(
                () -> Math.abs(state().goalAngle() - angleSupplier.getAsDouble()) < INTAKE_ANGLE_TOLERANCE);

        fourBarMechanism = new Arm(
                armMotorGroup,
                angleSupplier,
                ARM_VELOCITY_LIMIT,
                ARM_POSITION_GAINS,
                new Mass(() -> Math.cos(angleSupplier.getAsDouble()), () -> Math.sin(angleSupplier.getAsDouble()), ARM_MASS),
                ARM_MAX_OUTPUT_VOLTAGE);

        setDefaultCommand(defaultCommand());
    }

    public Command defaultCommand() {
        Command defaultCommand = new ConditionalCommand(
                runRollerAndPump(),
                holdStateGoal(),
                () -> state().isPumping());
        defaultCommand.addRequirements(this);
        return defaultCommand;
    }

    /** Hold the state's arm goal while spinning the roller at the state voltage (OPEN / CLOSE). */
    private Command holdStateGoal() {
        return rollerMechanism.manualCommand(() -> state().rollerVoltage())
                .alongWith(setPositionCommand(() -> state().goalAngle()))
                .until(() -> state().isPumping());
    }

    /** Spin the roller while stroking the arm in/out to clear jams (PUMP). */
    private Command runRollerAndPump() {
        return rollerMechanism.manualCommand(() -> state().rollerVoltage())
                .alongWith(pumpCommand())
                .until(() -> !state().isPumping());
    }

    private Command pumpCommand() {
        return new SequentialCommandGroup(
                setPositionCommand(() -> PUMP_EXTENDED_ANGLE).withTimeout(PUMP_STEP_TIMEOUT_SEC),
                setPositionCommand(() -> PUMP_RETRACTED_ANGLE).withTimeout(PUMP_STEP_TIMEOUT_SEC)
        ).repeatedly();
    }

    public Command setPositionCommand(DoubleSupplier angle) {
        return fourBarMechanism.anglePositionControlCommand(
                () -> intakeAngleLimit.limit(angle.getAsDouble()), at -> {}, MAX_OFFSET);
    }

    public Command forwardIntake() {
        return fourBarMechanism.manualCommand(() -> MANUAL_INTAKE_VOLTAGE, this);
    }

    public Command reverseIntake() {
        return fourBarMechanism.manualCommand(() -> -MANUAL_INTAKE_VOLTAGE, this);
    }

    @Log.NT
    public boolean getIsIntakeOpen() {
        return isIntakeOpen;
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
        return currentStateName();
    }
}
