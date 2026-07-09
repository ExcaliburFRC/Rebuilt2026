package frc.excalib2.swerve;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

/**
 * Move-to-pose: profiled PID on x, y, and heading driving field-centric velocity requests
 * (blue-frame — pass an already-flipped pose for alliance-relative targets, e.g. via
 * {@code AllianceFlip.apply}). Finishes when within tolerance.
 *
 * <p>For long moves across the field prefer PathPlanner {@code AutoBuilder.pathfindToPose};
 * this command is the short-range precision aligner.
 */
public class DriveToPose extends Command {
    private final SwerveSubsystem swerve;
    private final Supplier<Pose2d> targetSupplier;
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController headingController;

    private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance);

    /**
     * @param translationKp    proportional gain, m/s per meter of error
     * @param headingKp        proportional gain, rad/s per radian of error
     * @param constraints      translation velocity/accel limits (m/s, m/s²)
     * @param headingConstraints heading rate limits (rad/s, rad/s²)
     * @param translationToleranceMeters finish tolerance
     * @param headingToleranceRadians    finish tolerance
     */
    public DriveToPose(SwerveSubsystem swerve, Supplier<Pose2d> targetSupplier,
                       double translationKp, double headingKp,
                       TrapezoidProfile.Constraints constraints,
                       TrapezoidProfile.Constraints headingConstraints,
                       double translationToleranceMeters, double headingToleranceRadians) {
        this.swerve = swerve;
        this.targetSupplier = targetSupplier;
        xController = new ProfiledPIDController(translationKp, 0, 0, constraints);
        yController = new ProfiledPIDController(translationKp, 0, 0, constraints);
        headingController = new ProfiledPIDController(headingKp, 0, 0, headingConstraints);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        xController.setTolerance(translationToleranceMeters);
        yController.setTolerance(translationToleranceMeters);
        headingController.setTolerance(headingToleranceRadians);
        addRequirements(swerve);
        setName("Drive To Pose");
    }

    @Override
    public void initialize() {
        Pose2d pose = swerve.getState().Pose;
        var speeds = swerve.getState().Speeds;
        xController.reset(pose.getX(), speeds.vxMetersPerSecond);
        yController.reset(pose.getY(), speeds.vyMetersPerSecond);
        headingController.reset(pose.getRotation().getRadians(), speeds.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        Pose2d pose = swerve.getState().Pose;
        Pose2d target = targetSupplier.get();
        DogLog.log("Swerve/DriveToPose/Target", target);
        swerve.setControl(request
                .withVelocityX(xController.calculate(pose.getX(), target.getX()))
                .withVelocityY(yController.calculate(pose.getY(), target.getY()))
                .withRotationalRate(headingController.calculate(
                        pose.getRotation().getRadians(), target.getRotation().getRadians())));
    }

    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && headingController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(new SwerveRequest.Idle());
    }
}
