package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.additional_utilities.LEDs;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Transport;
import frc.robot.util.LedState;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.FieldConstants.CLOSE_TRENCH_TO_BUMP_X;
import static frc.robot.Constants.FieldConstants.FAR_TRENCH_TO_BUMP_X;
import static frc.robot.superstructure.RobotState.*;

public class Superstructure implements Logged {
    private RobotState currentRobotState;

    public final Shooter shooter;
    private final Transport transport;
    public final Intake intake;
    private final LEDs leds;

    private final Trigger robotAtState, ourAllianceShiftActivate;
    private final Trigger inIntermediateZone, inAllianceZone, inNeutralZone;
    private final Trigger closerToCloseDeliveryTrigger, underTrenchTrigger, overBumpTrigger;

    private final Trigger intakeRequested, shouldDeliver;

    private Trigger NO_INTAKE_SHOOT_HUB_TRIGGER, INTAKE_SHOOT_HUB_TRIGGER,
            NO_INTAKE_SHOOT_CLOSE_DELIVERY_TRIGGER, NO_INTAKE_SHOOT_FAR_DELIVERY_TRIGGER,
            INTAKE_SHOOT_CLOSE_DELIVERY_TRIGGER, INTAKE_SHOOT_FAR_DELIVERY_TRIGGER,
            NO_INTAKE_AIM_HUB_TRIGGER, INTAKE_AIM_HUB_TRIGGER;
//            NO_INTAKE_AIM_CLOSE_DELIVERY_TRIGGER;
//            NO_INTAKE_AIM_FAR_DELIVERY_TRIGGER, INTAKE_AIM_CLOSE_DELIVERY_TRIGGER,
//            INTAKE_AIM_FAR_DELIVERY_TRIGGER;

    public Superstructure(Swerve swerve, Trigger intakeButton, Trigger ourAllianceShiftActivate, Trigger deliveryButton) {
        currentRobotState = RobotState.NO_INTAKE_AIM_HUB;

        shooter = new Shooter(swerve::getPose2D, swerve::getRobotRelativeSpeeds);
//        transport = new Transport(()-> true);
        transport = new Transport(shooter.isShooterReady());
        intake = new Intake();
        leds = LEDs.getInstance();

        robotAtState =
                (transport.atPositionTrigger())
                .and(shooter.isShooterReady());

        this.ourAllianceShiftActivate = new Trigger(ourAllianceShiftActivate);

//        inAllianceZone = new Trigger(
//                () -> {
//                    if (AllianceUtils.isBlueAlliance()) {
//                        return shooter.getTurretOnField().getTranslation().getX()
//                                < (4.02 - 0.2);
//                    }
//                    return shooter.getTurretOnField().getTranslation().getX()
//                            > (12.51 + 0.2);
//                }
//
//        ); //tag 26 x

        inAllianceZone = new Trigger(()-> true);



//        inIntermediateZone = new Trigger(
//                () -> {
//                    if (AllianceUtils.isBlueAlliance()){
//                        return shooter.getTurretOnField().getTranslation().getX() < (5.22 + 0.2);
//                    }
//                    return shooter.getTurretOnField().getTranslation().getX() > (11.3 - 0.2);
//                })
//                .and(inAllianceZone.negate());

        inIntermediateZone = new Trigger(()-> false);

        inNeutralZone = (inAllianceZone.or(inIntermediateZone)).negate();

        closerToCloseDeliveryTrigger = new Trigger(
                () -> shooter.getTurretOnField().getTranslation().getY() < AllianceUtils.FIELD_WIDTH_METERS / 2);

        underTrenchTrigger = new Trigger(
                () -> {
                    double y = shooter.getTurretOnField().getTranslation().getY();
                    return y < CLOSE_TRENCH_TO_BUMP_X || y > FAR_TRENCH_TO_BUMP_X;
                }
        ).and(inIntermediateZone);

        overBumpTrigger = inIntermediateZone.and(underTrenchTrigger.negate());
        intakeRequested = intakeButton;
        this.shouldDeliver = deliveryButton;

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
//                .and(ourAllianceShiftActivate.negate())
//                .and(inNeutralZone)
                .and(shouldDeliver)
                .and(closerToCloseDeliveryTrigger)
                .onTrue(setStateCommand(NO_INTAKE_SHOOT_CLOSE_DELIVERY));

        NO_INTAKE_SHOOT_FAR_DELIVERY_TRIGGER = intakeRequested.negate()
//                .and(ourAllianceShiftActivate.negate())
//                .and(inNeutralZone)
                .and(shouldDeliver)
                .and(closerToCloseDeliveryTrigger.negate())
                .onTrue(setStateCommand(NO_INTAKE_SHOOT_FAR_DELIVERY));

        INTAKE_SHOOT_CLOSE_DELIVERY_TRIGGER = intakeRequested
//                .and(ourAllianceShiftActivate.negate())
//                .and(inNeutralZone)
                .and(shouldDeliver)
                .and(closerToCloseDeliveryTrigger)
                .onTrue(setStateCommand(INTAKE_SHOOT_CLOSE_DELIVERY));

        INTAKE_SHOOT_FAR_DELIVERY_TRIGGER = intakeRequested
//                .and(ourAllianceShiftActivate.negate())
//                .and(inNeutralZone)
                .and(shouldDeliver)
                .and(closerToCloseDeliveryTrigger.negate())
                .onTrue(setStateCommand(INTAKE_SHOOT_FAR_DELIVERY));

//         also when over bump
        NO_INTAKE_AIM_HUB_TRIGGER = intakeRequested.negate()
                .and(ourAllianceShiftActivate.negate())
                .and(inAllianceZone)
                .onTrue(setStateCommand(NO_INTAKE_AIM_HUB));

//         also when under trench
        INTAKE_AIM_HUB_TRIGGER = intakeRequested
                .and(ourAllianceShiftActivate.negate())
                .and(inAllianceZone)
                .onTrue(setStateCommand(INTAKE_AIM_HUB));

//        NO_INTAKE_AIM_CLOSE_DELIVERY_TRIGGER = intakeRequested.negate()
//                .and(ourAllianceShiftActivate)
//                .and(inNeutralZone)
//                .and(shouldDeliver)
//                .and(closerToCloseDeliveryTrigger)
//                .onTrue(setStateCommand(NO_INTAKE_AIM_CLOSE_DELIVERY));

//        NO_INTAKE_AIM_FAR_DELIVERY_TRIGGER = intakeRequested.negate()
//                .and(ourAllianceShiftActivate)
//                .and(inNeutralZone)
//                .and(shouldDeliver)
//                .and(closerToCloseDeliveryTrigger.negate())
//                .onTrue(setStateCommand(NO_INTAKE_AIM_FAR_DELIVERY));

//        INTAKE_AIM_CLOSE_DELIVERY_TRIGGER = intakeRequested
//                .and(ourAllianceShiftActivate)
//                .and(inNeutralZone)
//                .and(shouldDeliver)
//                .and(closerToCloseDeliveryTrigger)
//                .onTrue(setStateCommand(INTAKE_AIM_CLOSE_DELIVERY));

//        INTAKE_AIM_FAR_DELIVERY_TRIGGER = intakeRequested
//                .and(ourAllianceShiftActivate)
//                .and(inNeutralZone)
//                .and(shouldDeliver)
//                .and(closerToCloseDeliveryTrigger.negate())
//                .onTrue(setStateCommand(INTAKE_AIM_FAR_DELIVERY));
    }

    public Command setStateCommand(RobotState robotStateToSet) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> currentRobotState = robotStateToSet),
                shooter.setStateCommand(robotStateToSet.shooterState),
                transport.setStateCommand(robotStateToSet.transportState),
                intake.setStateCommand(robotStateToSet.intakeState)
        );
    }

    private LedState ledStateFor(RobotState state) {
        return switch (state) {
            case NO_INTAKE_SHOOT_HUB,
                 INTAKE_SHOOT_HUB,
                 NO_INTAKE_SHOOT_HUB_test,
                 NO_INTAKE_SHOOT_CLOSE_DELIVERY,
                 NO_INTAKE_SHOOT_FAR_DELIVERY,
                 INTAKE_SHOOT_CLOSE_DELIVERY,
                 INTAKE_SHOOT_FAR_DELIVERY -> shooter.isShooterReady().getAsBoolean()
                    ? LedState.LOCKED_ON_TARGET
                    : LedState.WAITING;
            case NO_INTAKE_AIM_HUB,
                 INTAKE_AIM_HUB,
                 NO_INTAKE_AIM_CLOSE_DELIVERY,
                 NO_INTAKE_AIM_FAR_DELIVERY,
                 INTAKE_AIM_CLOSE_DELIVERY,
                 INTAKE_AIM_FAR_DELIVERY -> LedState.WAITING;
            case IDLE -> LedState.IDLE;
        };
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

//    @Log.NT
//    public boolean getNO_INTAKE_AIM_HUB_TRIGGER() {
//        return NO_INTAKE_AIM_HUB_TRIGGER.getAsBoolean();
//    }
//
//    @Log.NT
//    public boolean getINTAKE_AIM_HUB_TRIGGER() {
//        return INTAKE_AIM_HUB_TRIGGER.getAsBoolean();
//    }
//
//    @Log.NT
//    public boolean getNO_INTAKE_AIM_CLOSE_DELIVERY_TRIGGER() {
//        return NO_INTAKE_AIM_CLOSE_DELIVERY_TRIGGER.getAsBoolean();
//    }
//
//    @Log.NT
//    public boolean getNO_INTAKE_AIM_FAR_DELIVERY_TRIGGER() {
//        return NO_INTAKE_AIM_FAR_DELIVERY_TRIGGER.getAsBoolean();
//    }
//
//    @Log.NT
//    public boolean getINTAKE_AIM_CLOSE_DELIVERY_TRIGGER() {
//        return INTAKE_AIM_CLOSE_DELIVERY_TRIGGER.getAsBoolean();
//    }
//
//    @Log.NT
//    public boolean getINTAKE_AIM_FAR_DELIVERY_TRIGGER() {
//        return INTAKE_AIM_FAR_DELIVERY_TRIGGER.getAsBoolean();
//    }

    public Command coastCommand() {
        return shooter.turretMechanism.coastCommand(this.shooter);
    }

    public Command setManualShootingCommand(DoubleSupplier hoodAngle, DoubleSupplier rpsSpeed) {
        Command command = new ParallelCommandGroup(
                shooter.setAdjustedTurretAngle(),
                shooter.setHoodAngleCommand(hoodAngle),
                shooter.setFlyWheelVelocity(rpsSpeed),
                transport.manualTransport()
        );
        command.addRequirements(shooter);
        return command;
    }
}