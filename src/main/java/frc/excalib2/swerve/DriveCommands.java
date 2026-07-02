package frc.excalib2.swerve;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

/**
 * Teleop drive command factories for {@link SwerveSubsystem}.
 *
 * <p>Joystick inputs are deadbanded <b>and rescaled</b> (no output step at the deadband
 * edge) via {@link MathUtil#applyDeadband}.
 */
public final class DriveCommands {
    private DriveCommands() {
    }

    /**
     * Field-centric teleop drive (operator-perspective aware — CTRE flips for red).
     *
     * @param x/y/omega       joystick axes in [-1, 1]; x forward, y left, omega CCW
     * @param maxSpeedMps     full-stick linear speed
     * @param maxOmegaRadPerSec full-stick rotational rate
     */
    public static Command fieldCentric(SwerveSubsystem swerve,
                                       DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega,
                                       double maxSpeedMps, double maxOmegaRadPerSec, double deadband) {
        SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
                .withDeadband(0) // deadband handled on the inputs, rescaled
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
        return swerve.applyRequest(() -> request
                        .withVelocityX(MathUtil.applyDeadband(x.getAsDouble(), deadband) * maxSpeedMps)
                        .withVelocityY(MathUtil.applyDeadband(y.getAsDouble(), deadband) * maxSpeedMps)
                        .withRotationalRate(MathUtil.applyDeadband(omega.getAsDouble(), deadband) * maxOmegaRadPerSec))
                .withName("Field Centric Drive");
    }

    /** Points all modules inward (X-stance) to resist pushing. */
    public static Command brake(SwerveSubsystem swerve) {
        SwerveRequest.SwerveDriveBrake request = new SwerveRequest.SwerveDriveBrake();
        return swerve.applyRequest(() -> request).withName("Swerve Brake");
    }

    /** Coasts all modules (idle request). */
    public static Command idle(SwerveSubsystem swerve) {
        SwerveRequest.Idle request = new SwerveRequest.Idle();
        return swerve.applyRequest(() -> request).withName("Swerve Idle");
    }
}
