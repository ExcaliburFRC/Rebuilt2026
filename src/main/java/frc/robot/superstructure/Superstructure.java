package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Transport;
import monologue.Annotations.Log;
import monologue.Logged;

import static frc.robot.superstructure.RobotState.*;

public class Superstructure implements Logged {
    private RobotState currentRobotState;

    private final Shooter shooter;
    private final Transport transport;
    private final Intake intake;

    private final Trigger robotAtState, ourAllianceShiftActivate;
    private final Trigger inIntermediateZone, inAllianceZone, inNeutralZone;
    private final Trigger closerToCloseDeliveryTrigger;

    private final Trigger intakeRequested;


    private Trigger NO_INTAKE_SHOOT_HUB_TRIGGER, INTAKE_SHOOT_HUB_TRIGGER,
            NO_INTAKE_SHOOT_CLOSE_DELIVERY_TRIGGER, NO_INTAKE_SHOOT_FAR_DELIVERY_TRIGGER,
            INTAKE_SHOOT_CLOSE_DELIVERY_TRIGGER, INTAKE_SHOOT_FAR_DELIVERY_TRIGGER,
            NO_INTAKE_AIM_HUB_TRIGGER, INTAKE_AIM_HUB_TRIGGER, NO_INTAKE_AIM_CLOSE_DELIVERY_TRIGGER,
            NO_INTAKE_AIM_FAR_DELIVERY_TRIGGER, INTAKE_AIM_CLOSE_DELIVERY_TRIGGER,
            INTAKE_AIM_FAR_DELIVERY_TRIGGER;

    public Superstructure(Swerve swerve, Trigger intakeButton) {
        currentRobotState = RobotState.NO_INTAKE_AIM_HUB;

        shooter = new Shooter(swerve::getPose2D, swerve::getRobotRelativeSpeeds);
        transport = new Transport();
        intake = new Intake();

        robotAtState = intake.atPositionTrigger
                .and(transport.atPositionTrigger())
                .and(shooter.isShooterReady());

        ourAllianceShiftActivate = new Trigger(() -> true);

        inAllianceZone = new Trigger(
                () -> shooter.getTurretOnField().getTranslation().getX()
                        < (4.02 - 0.2)
        ); //tag 26 x

        inIntermediateZone = new Trigger(
                () -> shooter.getTurretOnField().getTranslation().getX()
                        < (5.22 + 0.2))
                .and(inAllianceZone.negate());

        inNeutralZone = (inAllianceZone.or(inIntermediateZone)).negate();

        closerToCloseDeliveryTrigger = new Trigger(
                () -> shooter.getTurretOnField().getTranslation().getY() < AllianceUtils.FIELD_WIDTH_METERS / 2);

        intakeRequested = intakeButton;

        initTriggers();
    }

    public void initTriggers() {
        NO_INTAKE_SHOOT_HUB_TRIGGER = intakeRequested.negate()
                .and(ourAllianceShiftActivate)
                .and(inAllianceZone)
                .onTrue(setStateCommand(NO_INTAKE_SHOOT_HUB));

        INTAKE_SHOOT_HUB_TRIGGER = intakeRequested
                .and(ourAllianceShiftActivate)
                .and(inAllianceZone)
                .onTrue(setStateCommand(INTAKE_SHOOT_HUB));

        NO_INTAKE_SHOOT_CLOSE_DELIVERY_TRIGGER = intakeRequested.negate()
                .and(ourAllianceShiftActivate.negate())
                .and(inNeutralZone)
                .and(closerToCloseDeliveryTrigger)
                .onTrue(setStateCommand(NO_INTAKE_SHOOT_CLOSE_DELIVERY));

        NO_INTAKE_SHOOT_FAR_DELIVERY_TRIGGER = intakeRequested.negate()
                .and(ourAllianceShiftActivate.negate())
                .and(inNeutralZone)
                .and(closerToCloseDeliveryTrigger.negate())
                .onTrue(setStateCommand(NO_INTAKE_SHOOT_FAR_DELIVERY));

        INTAKE_SHOOT_CLOSE_DELIVERY_TRIGGER = intakeRequested
                .and(ourAllianceShiftActivate.negate())
                .and(inNeutralZone)
                .and(closerToCloseDeliveryTrigger)
                .onTrue(setStateCommand(INTAKE_SHOOT_CLOSE_DELIVERY));

        INTAKE_SHOOT_FAR_DELIVERY_TRIGGER = intakeRequested
                .and(ourAllianceShiftActivate.negate())
                .and(inNeutralZone)
                .and(closerToCloseDeliveryTrigger.negate())
                .onTrue(setStateCommand(INTAKE_SHOOT_FAR_DELIVERY));

        // also when over bump
        NO_INTAKE_AIM_HUB_TRIGGER = intakeRequested.negate()
                .and(ourAllianceShiftActivate.negate())
                .and(inAllianceZone)
                .onTrue(setStateCommand(NO_INTAKE_AIM_HUB));

        // also when under trench
        INTAKE_AIM_HUB_TRIGGER = intakeRequested
                .and(ourAllianceShiftActivate.negate())
                .and(inAllianceZone)
                .onTrue(setStateCommand(INTAKE_AIM_HUB));

        NO_INTAKE_AIM_CLOSE_DELIVERY_TRIGGER = intakeRequested.negate()
                .and(ourAllianceShiftActivate)
                .and(inNeutralZone)
                .and(closerToCloseDeliveryTrigger)
                .onTrue(setStateCommand(NO_INTAKE_AIM_CLOSE_DELIVERY));

        NO_INTAKE_AIM_FAR_DELIVERY_TRIGGER = intakeRequested.negate()
                .and(ourAllianceShiftActivate)
                .and(inNeutralZone)
                .and(closerToCloseDeliveryTrigger.negate())
                .onTrue(setStateCommand(NO_INTAKE_AIM_FAR_DELIVERY));

        INTAKE_AIM_CLOSE_DELIVERY_TRIGGER = intakeRequested
                .and(ourAllianceShiftActivate)
                .and(inNeutralZone)
                .and(closerToCloseDeliveryTrigger)
                .onTrue(setStateCommand(INTAKE_AIM_CLOSE_DELIVERY));

        INTAKE_AIM_FAR_DELIVERY_TRIGGER = intakeRequested
                .and(ourAllianceShiftActivate)
                .and(inNeutralZone)
                .and(closerToCloseDeliveryTrigger.negate())
                .onTrue(setStateCommand(INTAKE_AIM_FAR_DELIVERY));
    }

    public Command setStateCommand(RobotState robotStateToSet) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> currentRobotState = robotStateToSet),
                shooter.setStateCommand(robotStateToSet.shooterState),
                transport.setStateCommand(robotStateToSet.transportState),
                intake.setStateCommand(robotStateToSet.intakeState)
        );
    }

    public Command setStateCommandAndWait(RobotState robotStateToSet) {
        return setStateCommand(robotStateToSet)
                .andThen(new WaitUntilCommand(robotAtState));
    }


    @Log.NT
    public String getCurrentRobotState() {
        return currentRobotState.name();
    }

    @Log.NT
    public boolean getIntakeRequested() {
        return intakeRequested.getAsBoolean();
    }

    @Log.NT
    public boolean getInIntermediateZone() {
        return inIntermediateZone.getAsBoolean();
    }

    @Log.NT
    public boolean getRobotAtState() {
        return robotAtState.getAsBoolean();
    }

    @Log.NT
    public boolean getOurAllianceShiftActivate() {
        return ourAllianceShiftActivate.getAsBoolean();
    }

    @Log.NT
    public boolean getInAllianceZone() {
        return inAllianceZone.getAsBoolean();
    }

    @Log.NT
    public boolean getInNeutralZone() {
        return inNeutralZone.getAsBoolean();
    }

    @Log.NT
    public boolean getCloserToCloseDeliveryTrigger() {
        return closerToCloseDeliveryTrigger.getAsBoolean();
    }


    @Log.NT
    public boolean getINTAKE_SHOOT_HUB_TRIGGER() {
        return INTAKE_SHOOT_HUB_TRIGGER.getAsBoolean();
    }

    @Log.NT
    public boolean getNO_INTAKE_SHOOT_HUB_TRIGGER() {
        return NO_INTAKE_SHOOT_HUB_TRIGGER.getAsBoolean();
    }

    @Log.NT
    public boolean getNO_INTAKE_SHOOT_CLOSE_DELIVERY_TRIGGER() {
        return NO_INTAKE_SHOOT_CLOSE_DELIVERY_TRIGGER.getAsBoolean();
    }

    @Log.NT
    public boolean getNO_INTAKE_SHOOT_FAR_DELIVERY_TRIGGER() {
        return NO_INTAKE_SHOOT_FAR_DELIVERY_TRIGGER.getAsBoolean();
    }

    @Log.NT
    public boolean getINTAKE_SHOOT_CLOSE_DELIVERY_TRIGGER() {
        return INTAKE_SHOOT_CLOSE_DELIVERY_TRIGGER.getAsBoolean();
    }

    @Log.NT
    public boolean getINTAKE_SHOOT_FAR_DELIVERY_TRIGGER() {
        return INTAKE_SHOOT_FAR_DELIVERY_TRIGGER.getAsBoolean();
    }

    @Log.NT
    public boolean getNO_INTAKE_AIM_HUB_TRIGGER() {
        return NO_INTAKE_AIM_HUB_TRIGGER.getAsBoolean();
    }

    @Log.NT
    public boolean getINTAKE_AIM_HUB_TRIGGER() {
        return INTAKE_AIM_HUB_TRIGGER.getAsBoolean();
    }

    @Log.NT
    public boolean getNO_INTAKE_AIM_CLOSE_DELIVERY_TRIGGER() {
        return NO_INTAKE_AIM_CLOSE_DELIVERY_TRIGGER.getAsBoolean();
    }

    @Log.NT
    public boolean getNO_INTAKE_AIM_FAR_DELIVERY_TRIGGER() {
        return NO_INTAKE_AIM_FAR_DELIVERY_TRIGGER.getAsBoolean();
    }

    @Log.NT
    public boolean getINTAKE_AIM_CLOSE_DELIVERY_TRIGGER() {
        return INTAKE_AIM_CLOSE_DELIVERY_TRIGGER.getAsBoolean();
    }

    @Log.NT
    public boolean getINTAKE_AIM_FAR_DELIVERY_TRIGGER() {
        return INTAKE_AIM_FAR_DELIVERY_TRIGGER.getAsBoolean();
    }

}