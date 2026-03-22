package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Transport;
import monologue.Logged;

public class Superstructure implements Logged {
    private RobotState currentRobotState;

    private final Shooter shooter;
    private final Transport transport;
    private final Intake intake;

    private final Trigger robotAtState, ourAllianceShiftActivate;
    private final Trigger inIntermediateZone, inAllianceZone, inNeutralZone;
    private final Trigger closerToCloseDeliveryTrigger, intakeRequested;

    Superstructure(Swerve swerve, Trigger intakeButton) {
        currentRobotState = RobotState.NO_INTAKE_AIM_HUB;

        shooter = new Shooter(swerve::getPose2D, swerve::getRobotRelativeSpeeds);
        transport = new Transport();
        intake = new Intake();

        robotAtState = intake.atPositionTrigger
                .and(transport.atPositionTrigger())
                .and(shooter.isShooterReady());

        ourAllianceShiftActivate = new Trigger(() -> true);

        inAllianceZone = new Trigger(()-> true);
        inIntermediateZone = new Trigger(() -> true);

        inNeutralZone = inAllianceZone.negate()
                .and(inIntermediateZone).negate();

        closerToCloseDeliveryTrigger = new Trigger(()-> true);

        intakeRequested = intakeButton;

        initTriggers();
    }

    public void initTriggers(){
        
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
}