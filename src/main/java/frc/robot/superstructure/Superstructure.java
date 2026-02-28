package frc.robot.superstructure;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.Target;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.PhysicalConstants.*;

/**
 * Coordinates all shooting-side subsystems (turret, shooter, transport, intake).
 *
 * ── COMPETITIVE EDGE IDEAS ─────────────────────────────────────────────────
 * Implemented in this PR:
 *  1. Smart transport gate  – drum + shooter transport only feed when flywheel
 *     is at speed AND turret is on target, eliminating wasted / off-speed shots.
 *  2. Drive speed scaling   – 45 % max speed while any targeting is active so
 *     the robot doesn't skid during a shot.
 *  3. Auto pre-spin zone    – flywheel spins up automatically when the robot
 *     enters a configurable radius around the hub, saving ~0.5-1 s per shot.
 *  4. Ready-to-shoot LED    – solid green when isReadyToShoot() so drivers have
 *     instant visual confirmation before feeding.
 *  5. Unjam command         – reverses both transport motors briefly to clear jams.
 *  6. Delivery shooting     – full pipeline (turret + flywheel + gated transport)
 *     targeting the nearest delivery station.
 *  7. currentTarget fix     – was ALWAYS IDLE; turret aimed at field origin (0,0)
 *     on every shot.  Now properly set on entry and cleared on exit.
 *
 * Further ideas to implement:
 *  A. Shot counting         – detect flywheel velocity dip while feeding; count
 *     shots fired and surface on dashboard / auto logging.
 *  B. Two-ball burst mode   – pre-queue two fuel cells, fire both in one command
 *     without re-arming between shots.
 *  C. Dynamic velocity table from real match data – gather shot timestamps +
 *     outcomes to auto-refine the distance→velocity map post-match.
 *  D. Stale-vision rejection – ignore Aurora updates whose position jumps >0.5 m
 *     from last cycle; protects odometry from bad AprilTag reads.
 *  E. On-the-fly PID tuning tab – expose kP/kD of turret + hood PID through
 *     Shuffleboard sliders for in-pit tuning without re-deploying.
 *  F. Auto-centering on fuel ball – use a simple colour-detection pipeline
 *     (or IR beam-break sensor) to auto-aim the intake at a detected ball.
 *  G. Autonomous shot prediction – while path-following, pre-calculate the exact
 *     robot pose at which to start the shot sequence so the ball arrives at the
 *     hub at the correct time.
 *  H. Current-based jam detection – monitor drum / transport motor current; if
 *     it spikes above threshold for >150 ms, auto-schedule unjamCommand().
 *  I. Spin-differential shooting – run top and bottom flywheels at different
 *     speeds (e.g. top 5 % faster) to impart controlled backspin and reduce
 *     bounce-out rate.
 *  J. Turret cable-wrap protection – add a "stale" watchdog: if turret hasn't
 *     moved for >5 s but has a non-IDLE target, nudge it to prevent cable binding.
 * ──────────────────────────────────────────────────────────────────────────
 */
public class Superstructure implements Logged {
    public final Intake intake;
    public final Shooter shooter;
    public final Transport transport;
    public final Turret turret;
    public final Swerve swerve;

    /** Distance from hub (metres) at which the flywheel starts spinning automatically. */
    public static final double PRE_SPIN_ZONE_METERS = 4.0;

    public Target currentTarget = Target.IDLE;

    public final InterpolatingDoubleTreeMap distanceTimeOfFlightMap;

    public final CommandPS5Controller controller;

    public Superstructure(CommandPS5Controller controller, Swerve swerve) {
        intake = new Intake();
        transport = new Transport();

        turret = new Turret(() -> getTurretToTargetVector(() -> currentTarget).get().getAngle().getRadians());
        shooter = new Shooter(() -> getTurretToTargetVector(() -> currentTarget).get().getNorm(), swerve::getPose2D);

        this.swerve = swerve;
        this.controller = controller;

        distanceTimeOfFlightMap = new InterpolatingDoubleTreeMap();
        initDistanceTimeOfFlightMap();
    }

    private void initDistanceTimeOfFlightMap() {
        distanceTimeOfFlightMap.put(2.99, 0.9);
        distanceTimeOfFlightMap.put(2.38, 0.7);
        distanceTimeOfFlightMap.put(3.88, 0.95);
    }

    public Supplier<Translation2d> getTurretToTargetVector(Supplier<Target> target) {
        Translation2d fieldToTargetTranslation = target.get().targetTranslation;
        Translation2d fieldToRobot = swerve.getPose2D().getTranslation();

        Translation2d robotToTarget = (fieldToTargetTranslation.minus(fieldToRobot)).rotateBy(swerve.getRotation2D().unaryMinus());
        Translation2d turretToTarget = robotToTarget.minus(TURRET_OFFSET_TRANSLATION);

        ChassisSpeeds robotSpeeds = swerve.getRobotRelativeSpeeds();

        return () -> {
            Translation2d virtualTargetOffset = new Translation2d(
                    robotSpeeds.vxMetersPerSecond
                            + TURRET_OFFSET_TRANSLATION.getY()
                            * robotSpeeds.omegaRadiansPerSecond,

                    robotSpeeds.vyMetersPerSecond
                            + TURRET_OFFSET_TRANSLATION.getX()
                            * robotSpeeds.omegaRadiansPerSecond
            ).times(distanceTimeOfFlightMap.get(turretToTarget.getNorm()));

            return turretToTarget.minus(virtualTargetOffset);
        };
    }


    public Supplier<Translation2d> getTurretToDeliveryVector() {
        Translation2d fieldToDeliveryTranslation;
        if (swerve.getPose2D().getTranslation().getDistance(DELIVERY_LEFT_POSE.get().getTranslation()) >
                swerve.getPose2D().getTranslation().getDistance(DELIVERY_RIGHT_POSE.get().getTranslation())) {
            fieldToDeliveryTranslation = DELIVERY_RIGHT_POSE.get().getTranslation();
        } else {
            fieldToDeliveryTranslation = DELIVERY_LEFT_POSE.get().getTranslation();
        }
        Translation2d fieldToRobot = swerve.getPose2D().getTranslation();

        Translation2d robotToDelivery = (fieldToDeliveryTranslation.minus(fieldToRobot)).rotateBy(swerve.getRotation2D().unaryMinus());
        Translation2d turretToDelivery = robotToDelivery.minus(TURRET_OFFSET_TRANSLATION);

        return () -> turretToDelivery;
    }

    // ─── Shooting Commands ─────────────────────────────────────────────────────

    /**
     * Full hub-shooting pipeline.
     * Sets currentTarget = HUB so the turret vector is correct, starts the
     * flywheel and turret tracking, and gates the drum transport on readiness.
     */
    public Command shootToHubCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> currentTarget = Target.HUB),
                new ParallelCommandGroup(
                        shooter.shootToHubCommand(),
                        turret.targetHubCommand(),
                        transport.gatedTransportCommand(this::isReadyToShoot)
                )
        ).finallyDo(() -> currentTarget = Target.IDLE);
    }

    /**
     * Pre-spin + track hub without feeding.
     * Sets currentTarget = HUB so the turret aims correctly.
     */
    public Command trackHubCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> currentTarget = Target.HUB),
                new ParallelCommandGroup(
                        shooter.trackHubCommand(),
                        turret.targetHubCommand()
                )
        ).finallyDo(() -> currentTarget = Target.IDLE);
    }

    /**
     * Full delivery-shooting pipeline targeting the nearest delivery station.
     * Automatically selects left or right delivery based on current robot pose.
     */
    public Command shootToDeliveryCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> currentTarget = Target.DELIVERY),
                new ParallelCommandGroup(
                        shooter.shootToDeliveryCommand(),
                        turret.targetDeliveryCommand(),
                        transport.gatedTransportCommand(this::isReadyToShoot)
                )
        ).finallyDo(() -> currentTarget = Target.IDLE);
    }

    /**
     * Silent pre-spin: spins flywheel toward hub velocity without feeding.
     * Use with kCancelSelf so driver commands override it immediately.
     */
    public Command autoPreSpinCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> currentTarget = Target.HUB),
                new ParallelCommandGroup(
                        shooter.trackHubCommand(),
                        turret.targetHubCommand()
                )
        ).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
                .finallyDo(() -> currentTarget = Target.IDLE);
    }

    // ─── Utility Commands ──────────────────────────────────────────────────────

    public Command intakeRollerActivationCommand(double voltage) {
        return intake.rollerManualCommand(voltage);
    }

    /**
     * Reverses both the drum transport and the shooter's internal transport
     * motor to clear fuel jams.  Bind to Triangle button.
     */
    public Command unjamCommand() {
        return new ParallelCommandGroup(
                shooter.unjamTransportCommand(),
                transport.unjamCommand()
        );
    }

    // ─── Readiness & Telemetry ─────────────────────────────────────────────────

    /**
     * True when the flywheel is at target speed AND the turret is on target.
     * Gate all transport feeding on this to avoid off-speed / off-angle shots.
     */
    @Log.NT
    public boolean isReadyToShoot() {
        return shooter.isFlywheelAtSpeed() && turret.isOnTarget();
    }

    /**
     * Returns 0.45 while any shoot/track command is active so the drivetrain
     * runs at 45 % speed, preventing skid that would throw off aiming.
     * Returns 1.0 during normal driving.
     */
    public double getDriveSpeedScale() {
        return shooter.getShooterTarget() != Target.IDLE ? 0.45 : 1.0;
    }

    @Log.NT
    public double getTurretToTargetVectorAngle() {
        return Units.radiansToDegrees(getTurretToTargetVector(() -> currentTarget).get().getAngle().getRadians());
    }

    @Log.NT
    public double getTurretToDeliveryVectorAngle() {
        return Units.radiansToDegrees(getTurretToDeliveryVector().get().getAngle().getRadians());
    }

    @Log.NT
    public double getTurretToTargetVectorDist() {
        return getTurretToTargetVector(() -> currentTarget).get().getNorm();
    }
}
