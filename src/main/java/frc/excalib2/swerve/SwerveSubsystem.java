package frc.excalib2.swerve;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.excalib2.swerve.vision.LimelightMegaTag2;
import frc.excalib2.util.AllianceFlip;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import static edu.wpi.first.units.Units.Volts;

/**
 * ExcaLib v2 swerve: a subsystem over the CTRE {@link SwerveDrivetrain}
 * (Tuner-X-shaped {@code SwerveModuleConstants}) with full ExcaLib v1 feature parity:
 *
 * <ul>
 *   <li>drive commands: field-centric, robot-centric, heading-locked (v1
 *       {@code turnToAngleCommand}), precision {@link DriveToPose} (v1 {@code pidToPose}),
 *       PathPlanner pathfinding (v1 {@code driveToPose} / {@code pathfindThenFollowPath}),
 *       pathfinding with driver override;</li>
 *   <li><b>MegaTag2</b> vision fusion with distance/tag-count std-dev scaling, an external
 *       rejection gate, a camera→robot pose transform (turret-mounted cameras), and a
 *       {@code sawTagRecently} trigger;</li>
 *   <li>housekeeping: heading/pose reset commands, coast command, brake/idle stances,
 *       drivetrain SysId (translation / steer / rotation);</li>
 *   <li>PathPlanner {@code AutoBuilder} wiring with wheel-force feedforwards, operator
 *       perspective, {@link Field2d} dashboard pose, DogLog telemetry, and built-in
 *       simulation (CTRE 5 ms sim thread).</li>
 * </ul>
 *
 * <p>The CTRE base runs its own 250 Hz odometry thread with time-synced CANivore reads —
 * do not route its signals through {@code SignalHub}.
 */
public class SwerveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
    private static final double SIM_LOOP_PERIOD_SECONDS = 0.005;

    private final SwerveRequest.ApplyRobotSpeeds robotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();
    private final Alert autoBuilderAlert = new Alert("PathPlanner AutoBuilder configuration failed", Alert.AlertType.kError);
    private final Field2d field = new Field2d();

    private Notifier simNotifier;
    private double lastSimTime;
    private boolean hasAppliedOperatorPerspective = false;

    // Vision (optional)
    private LimelightMegaTag2 limelight;
    private Supplier<Rotation2d> cameraYawSupplier;
    private BooleanSupplier visionRejectGate = () -> false;
    private UnaryOperator<Pose2d> cameraPoseToRobotPose = UnaryOperator.identity();
    private double lastAcceptedVisionTimestamp = Double.NEGATIVE_INFINITY;

    public SwerveSubsystem(SwerveDrivetrainConstants drivetrainConstants,
                           SwerveModuleConstants<?, ?, ?>... moduleConstants) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, moduleConstants);
        CommandScheduler.getInstance().registerSubsystem(this);
        SmartDashboard.putData("Field", field);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    // ── Getters (v1: getPose2D / getRotation2D / getRobotRelativeSpeeds) ─────

    public Pose2d getPose() {
        return getState().Pose;
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /** Robot-relative measured speeds. */
    public ChassisSpeeds getSpeeds() {
        return getState().Speeds;
    }

    /** Ground speed magnitude, m/s. */
    public double getVelocityMagnitude() {
        ChassisSpeeds speeds = getSpeeds();
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    // ── Drive commands ───────────────────────────────────────────────────────

    /** Runs the supplied swerve request every loop while scheduled. */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> setControl(requestSupplier.get()));
    }

    /** Drives robot-relative chassis speeds directly (v1 {@code driveRobotRelativeChassisSpeeds}). */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        setControl(robotSpeedsRequest.withSpeeds(speeds));
    }

    /**
     * Field-centric teleop drive (operator-perspective aware — CTRE flips for red).
     * Joystick inputs are deadbanded <b>and rescaled</b> (no output step at the edge).
     *
     * @param x/y/omega         joystick axes in [-1, 1]; x forward, y left, omega CCW
     * @param maxSpeedMps       full-stick linear speed
     * @param maxOmegaRadPerSec full-stick rotational rate
     */
    public Command fieldCentricDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega,
                                            double maxSpeedMps, double maxOmegaRadPerSec, double deadband) {
        SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
        return applyRequest(() -> request
                .withVelocityX(MathUtil.applyDeadband(x.getAsDouble(), deadband) * maxSpeedMps)
                .withVelocityY(MathUtil.applyDeadband(y.getAsDouble(), deadband) * maxSpeedMps)
                .withRotationalRate(MathUtil.applyDeadband(omega.getAsDouble(), deadband) * maxOmegaRadPerSec))
                .withName("Field Centric Drive");
    }

    /** Robot-centric teleop drive (v1 {@code driveCommand} with {@code fieldOriented = false}). */
    public Command robotCentricDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega,
                                            double maxSpeedMps, double maxOmegaRadPerSec, double deadband) {
        SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
        return applyRequest(() -> request
                .withVelocityX(MathUtil.applyDeadband(x.getAsDouble(), deadband) * maxSpeedMps)
                .withVelocityY(MathUtil.applyDeadband(y.getAsDouble(), deadband) * maxSpeedMps)
                .withRotationalRate(MathUtil.applyDeadband(omega.getAsDouble(), deadband) * maxOmegaRadPerSec))
                .withName("Robot Centric Drive");
    }

    /**
     * Field-centric translation with the heading locked to a target direction
     * (v1 {@code turnToAngleCommand}). Heading is closed onboard the request's PID.
     *
     * @param targetHeading blue-frame heading supplier (e.g. face the hub)
     * @param headingKp     rad/s of correction per radian of heading error
     */
    public Command headingLockedDriveCommand(DoubleSupplier x, DoubleSupplier y,
                                             Supplier<Rotation2d> targetHeading,
                                             double maxSpeedMps, double deadband, double headingKp) {
        SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance);
        request.HeadingController.setPID(headingKp, 0, 0);
        request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        return applyRequest(() -> request
                .withVelocityX(MathUtil.applyDeadband(x.getAsDouble(), deadband) * maxSpeedMps)
                .withVelocityY(MathUtil.applyDeadband(y.getAsDouble(), deadband) * maxSpeedMps)
                .withTargetDirection(targetHeading.get()))
                .withName("Heading Locked Drive");
    }

    /** Points all modules inward (X-stance) to resist pushing. */
    public Command brakeCommand() {
        SwerveRequest.SwerveDriveBrake request = new SwerveRequest.SwerveDriveBrake();
        return applyRequest(() -> request).withName("Swerve Brake");
    }

    /** Applies no output (modules hold their configured neutral mode). */
    public Command idleCommand() {
        SwerveRequest.Idle request = new SwerveRequest.Idle();
        return applyRequest(() -> request).withName("Swerve Idle");
    }

    /**
     * Short-range precision move-to-pose (v1 {@code pidToPoseCommand};
     * see {@link DriveToPose} for parameters). For long moves prefer
     * {@link #pathfindToPoseCommand}.
     */
    public Command driveToPoseCommand(Supplier<Pose2d> target,
                                      double translationKp, double headingKp,
                                      TrapezoidProfile.Constraints translationConstraints,
                                      TrapezoidProfile.Constraints headingConstraints,
                                      double translationToleranceMeters, double headingToleranceRadians) {
        return new DriveToPose(this, target, translationKp, headingKp,
                translationConstraints, headingConstraints,
                translationToleranceMeters, headingToleranceRadians);
    }

    // ── PathPlanner commands (v1: driveToPoseCommand / pathfindThenFollowPath) ──

    /** Pathfinds around obstacles to the target pose (requires {@link #configureAutoBuilder}). */
    public Command pathfindToPoseCommand(Pose2d target, PathConstraints constraints) {
        return AutoBuilder.pathfindToPose(target, constraints).withName("Pathfind To Pose");
    }

    /** Pathfinds to a path's start, then follows it. Missing files alert instead of crashing (v1 behavior). */
    public Command pathfindThenFollowPathCommand(String pathName, PathConstraints constraints) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.pathfindThenFollowPath(path, constraints)
                    .withName("Pathfind Then Follow " + pathName);
        } catch (Exception exception) {
            new Alert("Path file not found: " + pathName, Alert.AlertType.kWarning).set(true);
            DogLog.logFault("PathFileMissing/" + pathName);
            return Commands.none();
        }
    }

    /**
     * Pathfinds to a pose, but the driver can take over by moving the sticks; pathfinding
     * resumes when the sticks are released (v1 {@code driveToPoseWithOverrideCommand}).
     */
    public Command pathfindWithOverrideCommand(Pose2d target, PathConstraints constraints,
                                               DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega,
                                               double maxSpeedMps, double maxOmegaRadPerSec, double deadband) {
        BooleanSupplier driverActive = () ->
                Math.abs(x.getAsDouble()) > deadband
                        || Math.abs(y.getAsDouble()) > deadband
                        || Math.abs(omega.getAsDouble()) > deadband;
        return Commands.repeatingSequence(
                        pathfindToPoseCommand(target, constraints).until(driverActive),
                        fieldCentricDriveCommand(x, y, omega, maxSpeedMps, maxOmegaRadPerSec, deadband)
                                .until(() -> !driverActive.getAsBoolean()))
                .withName("Pathfind With Override");
    }

    // ── Housekeeping (v1: resetAngleCommand / resetOdometryCommand / coastCommand) ──

    /** Re-zeros field-centric "forward" to the robot's current heading. Works while disabled. */
    public Command resetHeadingCommand() {
        return runOnce(this::seedFieldCentric).ignoringDisable(true).withName("Reset Heading");
    }

    /** Overwrites odometry with the given pose. Works while disabled. */
    public Command resetPoseCommand(Supplier<Pose2d> pose) {
        return runOnce(() -> resetPose(pose.get())).ignoringDisable(true).withName("Reset Pose");
    }

    /** Coast all motors while held (push the robot around), restore brake on release. */
    public Command coastCommand() {
        return startEnd(
                () -> configNeutralMode(NeutralModeValue.Coast),
                () -> configNeutralMode(NeutralModeValue.Brake))
                .ignoringDisable(true).withName("Swerve Coast");
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
    public SwerveSubsystem withVisionPoseTransform(UnaryOperator<Pose2d> transform) {
        this.cameraPoseToRobotPose = transform;
        return this;
    }

    /** True while a vision estimate was fused within the last {@code seconds} (v1 {@code sawTagRecently}). */
    public Trigger sawTagRecently(double seconds) {
        return new Trigger(() -> Timer.getFPGATimestamp() - lastAcceptedVisionTimestamp < seconds);
    }

    private void updateVision() {
        if (limelight == null) {
            return;
        }
        limelight.setRobotOrientation(cameraYawSupplier.get(),
                Math.toDegrees(getSpeeds().omegaRadiansPerSecond));
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
                lastAcceptedVisionTimestamp = Timer.getFPGATimestamp();
            }
        });
    }

    // ── Auto ─────────────────────────────────────────────────────────────────

    /** Wires PathPlanner's AutoBuilder to this drivetrain. Call once after construction. */
    public void configureAutoBuilder(PIDConstants translationGains, PIDConstants rotationGains) {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    this::getPose,
                    this::resetPose,
                    this::getSpeeds,
                    (speeds, feedforwards) -> setControl(robotSpeedsRequest
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

    // ── SysId (v1: per-module driveSysId/angleSysId → CTRE drivetrain routines) ──

    private SysIdRoutine sysIdRoutine(SwerveRequest.SysIdSwerveTranslation request) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(4), null,
                        state -> SignalLogger.writeString("Swerve/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> setControl(request.withVolts(volts)), null, this));
    }

    private SysIdRoutine steerSysIdRoutine(SwerveRequest.SysIdSwerveSteerGains request) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(4), null,
                        state -> SignalLogger.writeString("Swerve/SysIdSteerState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> setControl(request.withVolts(volts)), null, this));
    }

    /** Drive-motor characterization (translation). Data lands in the hoot log. */
    public Command sysIdTranslation(SysIdRoutine.Direction direction, boolean dynamic) {
        SysIdRoutine routine = sysIdRoutine(new SwerveRequest.SysIdSwerveTranslation());
        return dynamic ? routine.dynamic(direction) : routine.quasistatic(direction);
    }

    /** Steer-motor characterization. Data lands in the hoot log. */
    public Command sysIdSteer(SysIdRoutine.Direction direction, boolean dynamic) {
        SysIdRoutine routine = steerSysIdRoutine(new SwerveRequest.SysIdSwerveSteerGains());
        return dynamic ? routine.dynamic(direction) : routine.quasistatic(direction);
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
        field.setRobotPose(state.Pose);
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
