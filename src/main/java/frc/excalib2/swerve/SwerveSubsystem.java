package frc.excalib2.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.excalib2.swerve.vision.LimelightMegaTag2;
import frc.excalib2.util.AllianceFlip;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * ExcaLib v2 swerve: a thin subsystem over the CTRE {@link SwerveDrivetrain}
 * (Tuner-X-shaped {@code SwerveModuleConstants}), adding:
 *
 * <ul>
 *   <li>command-based integration ({@link #applyRequest});</li>
 *   <li><b>MegaTag2</b> vision fusion with distance/tag-count std-dev scaling and an
 *       external rejection gate (e.g. "turret spinning too fast");</li>
 *   <li>PathPlanner {@code AutoBuilder} wiring with wheel-force feedforwards;</li>
 *   <li>operator-perspective handling, DogLog telemetry, and built-in simulation
 *       (CTRE 5 ms sim thread).</li>
 * </ul>
 *
 * <p>The CTRE base runs its own 250 Hz odometry thread with time-synced CANivore reads —
 * do not route its signals through {@code SignalHub}.
 */
public class SwerveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
    private static final double SIM_LOOP_PERIOD_SECONDS = 0.005;

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();
    private final Alert autoBuilderAlert = new Alert("PathPlanner AutoBuilder configuration failed", Alert.AlertType.kError);

    private Notifier simNotifier;
    private double lastSimTime;
    private boolean hasAppliedOperatorPerspective = false;

    // Vision (optional)
    private LimelightMegaTag2 limelight;
    private Supplier<Rotation2d> cameraYawSupplier;
    private BooleanSupplier visionRejectGate = () -> false;
    private java.util.function.UnaryOperator<edu.wpi.first.math.geometry.Pose2d> cameraPoseToRobotPose =
            java.util.function.UnaryOperator.identity();

    public SwerveSubsystem(SwerveDrivetrainConstants drivetrainConstants,
                           SwerveModuleConstants<?, ?, ?>... moduleConstants) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, moduleConstants);
        CommandScheduler.getInstance().registerSubsystem(this);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    // ── Commands ─────────────────────────────────────────────────────────────

    /** Runs the supplied swerve request every loop while scheduled. */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> setControl(requestSupplier.get()));
    }

    /**
     * Field-centric teleop drive (operator-perspective aware — CTRE flips for red).
     * Joystick inputs are deadbanded <b>and rescaled</b> (no output step at the edge).
     *
     * @param x/y/omega         joystick axes in [-1, 1]; x forward, y left, omega CCW
     * @param maxSpeedMps       full-stick linear speed
     * @param maxOmegaRadPerSec full-stick rotational rate
     */
    public Command fieldCentricDriveCommand(java.util.function.DoubleSupplier x,
                                            java.util.function.DoubleSupplier y,
                                            java.util.function.DoubleSupplier omega,
                                            double maxSpeedMps, double maxOmegaRadPerSec,
                                            double deadband) {
        SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
                .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity);
        return applyRequest(() -> request
                .withVelocityX(edu.wpi.first.math.MathUtil.applyDeadband(x.getAsDouble(), deadband) * maxSpeedMps)
                .withVelocityY(edu.wpi.first.math.MathUtil.applyDeadband(y.getAsDouble(), deadband) * maxSpeedMps)
                .withRotationalRate(edu.wpi.first.math.MathUtil.applyDeadband(omega.getAsDouble(), deadband) * maxOmegaRadPerSec))
                .withName("Field Centric Drive");
    }

    /** Points all modules inward (X-stance) to resist pushing. */
    public Command brakeCommand() {
        SwerveRequest.SwerveDriveBrake request = new SwerveRequest.SwerveDriveBrake();
        return applyRequest(() -> request).withName("Swerve Brake");
    }

    /** Coasts all modules (idle request). */
    public Command idleCommand() {
        SwerveRequest.Idle request = new SwerveRequest.Idle();
        return applyRequest(() -> request).withName("Swerve Idle");
    }

    /**
     * Short-range precision move-to-pose (see {@link DriveToPose} for parameters).
     * For long moves prefer PathPlanner {@code AutoBuilder.pathfindToPose}.
     */
    public Command driveToPoseCommand(Supplier<edu.wpi.first.math.geometry.Pose2d> target,
                                      double translationKp, double headingKp,
                                      edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints translationConstraints,
                                      edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints headingConstraints,
                                      double translationToleranceMeters, double headingToleranceRadians) {
        return new DriveToPose(this, target, translationKp, headingKp,
                translationConstraints, headingConstraints,
                translationToleranceMeters, headingToleranceRadians);
    }

    // ── Vision ───────────────────────────────────────────────────────────────

    /**
     * Enables MegaTag2 fusion.
     *
     * @param limelightName    NT name, e.g. {@code "limelight-turret"}
     * @param cameraYawSupplier field heading of the camera platform (for a turret-mounted
     *                          camera: robot heading + turret angle)
     * @param rejectGate       when true, estimates are discarded (e.g. spinning too fast
     *                          for a reliable solve)
     */
    public SwerveSubsystem withLimelight(String limelightName,
                                         Supplier<Rotation2d> cameraYawSupplier,
                                         BooleanSupplier rejectGate) {
        this.limelight = new LimelightMegaTag2(limelightName);
        this.cameraYawSupplier = cameraYawSupplier;
        this.visionRejectGate = rejectGate;
        return this;
    }

    /**
     * For cameras not rigidly mounted to the chassis (e.g. on a turret): converts the
     * estimate's camera-platform pose into a robot pose before fusion.
     */
    public SwerveSubsystem withVisionPoseTransform(
            java.util.function.UnaryOperator<edu.wpi.first.math.geometry.Pose2d> transform) {
        this.cameraPoseToRobotPose = transform;
        return this;
    }

    private void updateVision() {
        if (limelight == null) {
            return;
        }
        limelight.setRobotOrientation(cameraYawSupplier.get(),
                Math.toDegrees(getState().Speeds.omegaRadiansPerSecond));
        limelight.getEstimate().ifPresent(estimate -> {
            boolean rejected = visionRejectGate.getAsBoolean();
            DogLog.log("Swerve/Vision/Pose", estimate.pose());
            DogLog.log("Swerve/Vision/TagCount", estimate.tagCount());
            DogLog.log("Swerve/Vision/Rejected", rejected);
            if (!rejected) {
                addVisionMeasurement(
                        cameraPoseToRobotPose.apply(estimate.pose()),
                        Utils.fpgaToCurrentTime(estimate.timestampSeconds()),
                        LimelightMegaTag2.stdDevs(estimate));
            }
        });
    }

    // ── Auto ─────────────────────────────────────────────────────────────────

    /** Wires PathPlanner's AutoBuilder to this drivetrain. Call once after construction. */
    public void configureAutoBuilder(PIDConstants translationGains, PIDConstants rotationGains) {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose,
                    this::resetPose,
                    () -> getState().Speeds,
                    (speeds, feedforwards) -> setControl(autoRequest
                            .withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(translationGains, rotationGains),
                    config,
                    () -> !AllianceFlip.isBlue(),
                    this);
        } catch (Exception exception) {
            autoBuilderAlert.set(true);
            DogLog.logFault("AutoBuilderConfigFailed");
        }
    }

    // ── Lifecycle ────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Operator perspective: blue-origin field coordinates; drive "forward" faces away
        // from the operator on both alliances. Re-appliable while disabled (alliance flips).
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(alliance -> {
                setOperatorPerspectiveForward(alliance == DriverStation.Alliance.Red
                        ? Rotation2d.k180deg
                        : Rotation2d.kZero);
                hasAppliedOperatorPerspective = true;
            });
        }

        updateVision();

        var state = getState();
        DogLog.log("Swerve/Pose", state.Pose);
        DogLog.log("Swerve/Speeds", state.Speeds);
        DogLog.log("Swerve/ModuleStates", state.ModuleStates);
        DogLog.log("Swerve/ModuleTargets", state.ModuleTargets);
        DogLog.log("Swerve/OdometryPeriodSeconds", state.OdometryPeriod);
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();
        simNotifier = new Notifier(() -> {
            double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(SIM_LOOP_PERIOD_SECONDS);
    }
}
