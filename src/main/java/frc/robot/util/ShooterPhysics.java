package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.util.ShooterPhysicsConstants.*;

/**
 * Stateless shooter physics model driven by Pose2d.
 */
public final class ShooterPhysics {

    private static final double TWO_PI = 2.0 * Math.PI;
    private static final double SEC_PER_MIN = 60.0;

    private final Supplier<Pose2d> robotPose;
    private final Pose2d hub;
    private final DoubleSupplier robotForwardVelocity;
    private final DoubleSupplier robotYawRateRadPerSec;

    public ShooterSolution currentSolution = new ShooterSolution(0, 0, 0, 0);

    private final InterpolatingDoubleTreeMap entryAngleMap;

    public ShooterPhysics(
            Supplier<Pose2d> robotPose,
            DoubleSupplier robotForwardVelocity,
            DoubleSupplier robotYawRateRadPerSec
    ) {
        this.robotPose = robotPose;
        this.hub = Constants.FieldConstants.BLUE_HUB_CENTER_POSE.getAsCurrentAlliance();
        this.robotForwardVelocity = robotForwardVelocity;
        this.robotYawRateRadPerSec = robotYawRateRadPerSec;

        this.entryAngleMap = new InterpolatingDoubleTreeMap();
        this.entryAngleMap.put(MIN_DISTANCE, ENTRY_ANGLE_CLOSE_RAD);
        this.entryAngleMap.put(MAX_DISTANCE, ENTRY_ANGLE_FAR_RAD);
    }

    /* ---------------- Public API ---------------- */

    public void solve() {
        Pose2d robot = robotPose.get();

        // Field-relative delta
        double dx = hub.getX() - robot.getX();
        double dy = hub.getY() - robot.getY();

        double distance = Math.hypot(dx, dy);

        double deltaH =
                HUB_HEIGHT_METERS
                        - HUB_ENTRY_OFFSET_METERS
                        - SHOOTER_EXIT_HEIGHT_METERS;

        /* ---------- Ballistic solution ---------- */

        double entryAngle = entryAngleMap.get(distance);

        double hoodAngle = clamp(
                calculateLaunchAngle(distance, deltaH, entryAngle),
                MIN_HOOD_ANGLE_RAD,
                MAX_HOOD_ANGLE_RAD
        );

        double vExit = calculateExitVelocity(distance, deltaH, hoodAngle);

        // Robot translation compensation
        vExit -= robotForwardVelocity.getAsDouble();

        // Aerodynamic drag
        vExit *= (1.0 + DRAG_COEFFICIENT * distance);

        // Magnus lift compensation
        double spinRPS = estimateSpinRPS(vExit);
        double effectiveGravity =
                GRAVITY * (1.0 - MAGNUS_COEFFICIENT * spinRPS);
        if (effectiveGravity > 0.0) {
            vExit *= Math.sqrt(GRAVITY / effectiveGravity);
        }

        double rpm = velocityToRPM(vExit);

        /* ---------- Turret solution ---------- */

        double absoluteTargetAngle = Math.atan2(dy, dx);
        double robotYaw = robot.getRotation().getRadians();

        double flightTime = distance / (vExit * Math.cos(hoodAngle));
        double yawLead = robotYawRateRadPerSec.getAsDouble() * flightTime;

        double turretAngle =
                Rotation2d.fromRadians(absoluteTargetAngle)
                        .minus(Rotation2d.fromRadians(robotYaw))
                        .getRadians()
                        + yawLead;

        currentSolution = new ShooterSolution(
                rpm,
                hoodAngle,
                turretAngle,
                distance
        );
    }

    /* ---------------- Physics ---------------- */

    private static double calculateLaunchAngle(double x, double h, double phi) {
        double tanTheta = (2.0 * h / x) - Math.tan(phi);
        return Math.atan(tanTheta);
    }

    private static double calculateExitVelocity(double x, double h, double theta) {
        double cos = Math.cos(theta);
        double denom = 2.0 * cos * cos * (x * Math.tan(theta) - h);
        if (denom <= 0.0) return 0.0;
        return Math.sqrt((GRAVITY * x * x) / denom);
    }

    private static double velocityToRPM(double v) {
        double eff = COMPRESSION_EFFICIENCY * SLIP_EFFICIENCY;
        double omega = v / (MAIN_ROLLER_RADIUS_METERS * eff);
        return (omega * SEC_PER_MIN) / TWO_PI;
    }

    private static double estimateSpinRPS(double vExit) {
        return vExit / (TWO_PI * FUEL_RADIUS_METERS);
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    /* ---------------- Output ---------------- */

    public record ShooterSolution(
            double rollerRPM,
            double hoodAngleRad,
            double turretAngleRad,
            double distanceMeters
    ) {
    }
}
