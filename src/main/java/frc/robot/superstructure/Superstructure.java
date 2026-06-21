package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.LEDs;
import frc.excalib.swerve.Swerve;
import frc.robot.lib.FieldZones;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Transport;
import frc.robot.util.LedState;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;

/**
 * Owns the three subsystems and coordinates them through {@link RobotState}.
 * <p>
 * Field geometry is delegated to {@link FieldZones} and the automatic state selection to
 * {@link RobotStateSelector}, leaving this class responsible only for dispatching a chosen state to
 * each subsystem and exposing a few high-level manual/telemetry hooks.
 */
public class Superstructure implements Logged {
    private RobotState currentRobotState;

    public final Shooter shooter;
    private final Transport transport;
    public final Intake intake;
    private final LEDs leds;

    private final FieldZones fieldZones;
    private final RobotStateSelector stateSelector;

    private final Trigger robotAtState;
    private final Trigger intakeRequested;
    private final Trigger shiftActive;

    public Superstructure(Swerve swerve, Trigger intakeButton, Trigger shiftActivate, Trigger shouldDeliver) {
        currentRobotState = RobotState.NO_INTAKE_AIM_HUB;

        shooter = new Shooter(swerve::getPose2D, swerve::getRobotRelativeSpeeds);
        // Field zones are evaluated from the turret's on-field position (delivery routing, hub side).
        fieldZones = new FieldZones(shooter::getTurretOnField);
        // Transport now gates on the real shooter-ready signal instead of a hard-coded constant.
        transport = new Transport(shooter.isShooterReady());
        intake = new Intake();
        leds = LEDs.getInstance();

        intakeRequested = intakeButton;
        shiftActive = new Trigger(shiftActivate);

        robotAtState = transport.atPositionTrigger().and(shooter.isShooterReady());

        stateSelector = new RobotStateSelector(intakeRequested, shiftActive, fieldZones, this::setStateCommand);
    }

    public Command setStateCommand(RobotState robotStateToSet) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> currentRobotState = robotStateToSet),
                shooter.setStateCommand(robotStateToSet.shooter()),
                transport.setStateCommand(robotStateToSet.transport()),
                intake.setStateCommand(robotStateToSet.intake())
        );
    }

    public Command setStateCommandAndWait(RobotState robotStateToSet) {
        return setStateCommand(robotStateToSet).andThen(new WaitUntilCommand(robotAtState));
    }

    public Command coastCommand() {
        return shooter.coastCommand();
    }

    public Command setManualShootingCommand(DoubleSupplier hoodAngle, DoubleSupplier rpsSpeed) {
        return shooter.manualShoot(hoodAngle, rpsSpeed).alongWith(transport.manualTransport());
    }

    private LedState ledStateFor(RobotState state) {
        return switch (state) {
            case NO_INTAKE_SHOOT_HUB,
                 INTAKE_SHOOT_HUB,
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

    @Log.NT
    public String getCurrentRobotState() {
        return currentRobotState.name();
    }

    @Log.NT
    public boolean getRobotAtState() {
        return robotAtState.getAsBoolean();
    }

    @Log.NT
    public boolean getIntakeRequested() {
        return intakeRequested.getAsBoolean();
    }

    @Log.NT
    public boolean getShiftActive() {
        return shiftActive.getAsBoolean();
    }
}
