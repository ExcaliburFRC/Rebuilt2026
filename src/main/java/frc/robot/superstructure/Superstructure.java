package frc.robot.superstructure;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.swerve.Swerve;
import frc.excalib2.statemachine.StateMachine;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Transport;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.FieldConstants.CLOSE_TRENCH_TO_BUMP_X;
import static frc.robot.Constants.FieldConstants.FAR_TRENCH_TO_BUMP_X;
import static frc.robot.superstructure.RobotState.*;

/**
 * Robot-goal coordinator on the ExcaLib v2 superstructure base.
 *
 * <p>Each {@link RobotState} goal is a tuple of per-mechanism states; entering a goal fans
 * it out to the intake / shooter / transport state machines in one place. Zone, shift, and
 * button {@link Trigger}s request goals — identical conditions to the pre-migration wiring
 * (including the temporarily stubbed zone triggers).
 */
public class Superstructure extends frc.excalib2.superstructure.Superstructure<RobotState> {

    public final Shooter shooter;
    private final Transport transport;
    public final Intake intake;

    private final Trigger robotAtState, ourAllianceShiftActivate;
    private final Trigger inIntermediateZone, inAllianceZone, inNeutralZone;
    private final Trigger closerToCloseDeliveryTrigger, underTrenchTrigger, overBumpTrigger;
    private final Trigger intakeRequested, shouldDeliver;

    public Superstructure(Swerve swerve, Trigger intakeButton, Trigger ourAllianceShiftActivate, Trigger deliveryButton) {
        this(new Shooter(swerve::getPose2D, swerve::getRobotRelativeSpeeds),
                intakeButton, ourAllianceShiftActivate, deliveryButton);
    }

    private Superstructure(Shooter shooter, Trigger intakeButton, Trigger ourAllianceShiftActivate, Trigger deliveryButton) {
        this(shooter, new Transport(shooter.isShooterReady()), new Intake(),
                intakeButton, ourAllianceShiftActivate, deliveryButton);
    }

    private Superstructure(Shooter shooter, Transport transport, Intake intake,
                           Trigger intakeButton, Trigger ourAllianceShiftActivate, Trigger deliveryButton) {
        super(buildMachine(shooter, transport, intake));
        this.shooter = shooter;
        this.transport = transport;
        this.intake = intake;

        robotAtState = transport.atPositionTrigger().and(shooter.isShooterReady());
        this.ourAllianceShiftActivate = ourAllianceShiftActivate;

        // Zone triggers — stubbed values preserved from the pre-migration code (the real
        // field-zone conditions are commented out there too; re-enable both together).
        inAllianceZone = new Trigger(() -> true);
        inIntermediateZone = new Trigger(() -> false);
        inNeutralZone = (inAllianceZone.or(inIntermediateZone)).negate();

        closerToCloseDeliveryTrigger = new Trigger(
                () -> shooter.getTurretOnField().getTranslation().getY() < AllianceUtils.FIELD_WIDTH_METERS / 2);

        underTrenchTrigger = new Trigger(() -> {
            double y = shooter.getTurretOnField().getTranslation().getY();
            return y < CLOSE_TRENCH_TO_BUMP_X || y > FAR_TRENCH_TO_BUMP_X;
        }).and(inIntermediateZone);

        overBumpTrigger = inIntermediateZone.and(underTrenchTrigger.negate());
        intakeRequested = intakeButton;
        shouldDeliver = deliveryButton;

        initTriggers();
    }

    /** Goal machine: every goal fans out to the three subsystem machines on entry. */
    private static StateMachine<RobotState> buildMachine(Shooter shooter, Transport transport, Intake intake) {
        StateMachine.Builder<RobotState> builder = StateMachine.builder("Superstructure", NO_INTAKE_AIM_HUB);
        for (RobotState goal : RobotState.values()) {
            builder.onEnter(goal, () -> {
                shooter.requestState(goal.shooterState);
                transport.requestState(goal.transportState);
                intake.requestState(goal.intakeState);
            });
            builder.transitionFromAny(goal); // v1 semantics: every goal reachable from anywhere
        }
        return builder.build();
    }

    private void initTriggers() {
        intakeRequested.negate()
                .and(ourAllianceShiftActivate)
                .and(inAllianceZone)
                .onTrue(request(NO_INTAKE_SHOOT_HUB));

        intakeRequested
                .and(ourAllianceShiftActivate)
                .and(inAllianceZone)
                .onTrue(request(INTAKE_SHOOT_HUB));

        intakeRequested.negate()
                .and(shouldDeliver)
                .and(closerToCloseDeliveryTrigger)
                .onTrue(request(NO_INTAKE_SHOOT_CLOSE_DELIVERY));

        intakeRequested.negate()
                .and(shouldDeliver)
                .and(closerToCloseDeliveryTrigger.negate())
                .onTrue(request(NO_INTAKE_SHOOT_FAR_DELIVERY));

        intakeRequested
                .and(shouldDeliver)
                .and(closerToCloseDeliveryTrigger)
                .onTrue(request(INTAKE_SHOOT_CLOSE_DELIVERY));

        intakeRequested
                .and(shouldDeliver)
                .and(closerToCloseDeliveryTrigger.negate())
                .onTrue(request(INTAKE_SHOOT_FAR_DELIVERY));
    }

    /** v1-compatible API (used by auto named commands and bindings). */
    public Command setStateCommand(RobotState state) {
        return request(state);
    }

    public Command setStateCommandAndWait(RobotState state) {
        return request(state).andThen(new WaitUntilCommand(robotAtState));
    }

    public Command coastCommand() {
        return shooter.coastTurretCommand();
    }

    public Command setManualShootingCommand(DoubleSupplier hoodAngle, DoubleSupplier rpsSpeed) {
        return shooter.manualShootingCommand(hoodAngle, rpsSpeed)
                .alongWith(transport.manualTransport());
    }

    @Override
    public void periodic() {
        super.periodic();
        DogLog.log("Superstructure/RobotAtState", robotAtState.getAsBoolean());
        DogLog.log("Superstructure/IntakeRequested", intakeRequested.getAsBoolean());
        DogLog.log("Superstructure/ShiftActive", ourAllianceShiftActivate.getAsBoolean());
        DogLog.log("Superstructure/InAllianceZone", inAllianceZone.getAsBoolean());
        DogLog.log("Superstructure/InNeutralZone", inNeutralZone.getAsBoolean());
        DogLog.log("Superstructure/CloserToCloseDelivery", closerToCloseDeliveryTrigger.getAsBoolean());
        DogLog.log("Superstructure/UnderTrench", underTrenchTrigger.getAsBoolean());
        DogLog.log("Superstructure/OverBump", overBumpTrigger.getAsBoolean());
    }
}
