package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.util.ShooterPhysicsConstants.*;

/**
 * Stateless shooter physics model.
 * Now supports any arbitrary 3D target point.
 */
public final class ShooterPhysics implements Logged {

    private static final double TWO_PI = 2.0 * Math.PI;

    private final Supplier<Pose2d> robotPose;
    private final DoubleSupplier robotForwardVelocity;
    private final DoubleSupplier robotYawRateRadPerSec;

    public ShooterSolution currentSolution = new ShooterSolution(0, 0, 0, 0);

    private final InterpolatingDoubleTreeMap entryAngleMap;

    public ShooterPhysics(Supplier<Pose2d> robotPose, DoubleSupplier robotForwardVelocity, DoubleSupplier robotYawRateRadPerSec) {
        this.robotPose = robotPose;
        this.robotForwardVelocity = robotForwardVelocity;
        this.robotYawRateRadPerSec = robotYawRateRadPerSec;

        this.entryAngleMap = new InterpolatingDoubleTreeMap();
        this.entryAngleMap.put(MIN_DISTANCE, ENTRY_ANGLE_CLOSE_RAD);
        this.entryAngleMap.put(MAX_DISTANCE, ENTRY_ANGLE_FAR_RAD);
    }

    /**
     * Solves for the shooter parameters given a specific 3D target point.
     * @param target The Translation3d (x,y,z) of the target goal.
     */
    public void solve(Translation3d target) {
        Pose2d robot = robotPose.get();

        // 1. Calculate Horizontal Distance (Ground plane)
        double dx = target.getX() - robot.getX();
        double dy = target.getY() - robot.getY();
        double horizontalDistance = Math.hypot(dx, dy);

        // 2. Calculate Vertical Delta (Z-axis)
        // target.getZ() is the height of the goal.
        // We subtract the exit height of the ball from the robot.
        double deltaH = target.getZ() - SHOOTER_EXIT_HEIGHT_METERS;

        double entryAngle = entryAngleMap.get(horizontalDistance);

        /* ---------- Iterative Ballistic Solver ---------- */

        double hoodAngle = clamp(
                calculateLaunchAngle(horizontalDistance, deltaH, entryAngle),
                MIN_HOOD_ANGLE_RAD,
                MAX_HOOD_ANGLE_RAD
        );

        double vExit = calculateExitVelocity(horizontalDistance, deltaH, hoodAngle);

        for (int i = 0; i < 3; i++) {
            double dragCompensation = 1.0 + (DRAG_COEFFICIENT * horizontalDistance);
            double spinRPS = estimateSpinRPS(vExit);
            double accelMagnus = MAGNUS_COEFFICIENT * spinRPS * GRAVITY;
            double effectiveGravity = GRAVITY - accelMagnus;

            vExit = calculateExitVelocityComplex(horizontalDistance, deltaH, hoodAngle, effectiveGravity) * dragCompensation;
        }

        /* ---------- Compensation & Output ---------- */

        vExit -= robotForwardVelocity.getAsDouble();
        double rollerOmega = velocityToRadPerSec(vExit);

        double absoluteTargetAngle = Math.atan2(dy, dx);
        double robotYaw = robot.getRotation().getRadians();
        double flightTime = horizontalDistance / (vExit * Math.cos(hoodAngle));
        double yawLead = robotYawRateRadPerSec.getAsDouble() * flightTime;

        double turretAngle = Rotation2d.fromRadians(absoluteTargetAngle)
                .minus(Rotation2d.fromRadians(robotYaw))
                .getRadians() + yawLead;

        currentSolution = new ShooterSolution(rollerOmega, hoodAngle, turretAngle, horizontalDistance);
    }

    /* ---------------- Physics Math ---------------- */

    private static double calculateLaunchAngle(double x, double h, double phi) {
        // Simple ballistic geometry for launch angle based on desired entry angle (phi)
        double tanTheta = (2.0 * h / x) - Math.tan(phi);
        return Math.atan(tanTheta);
    }

    private static double calculateExitVelocity(double x, double h, double theta) {
        return calculateExitVelocityComplex(x, h, theta, GRAVITY);
    }

    private static double calculateExitVelocityComplex(double x, double h, double theta, double g) {
        double cos = Math.cos(theta);
        double denom = 2.0 * cos * cos * (x * Math.tan(theta) - h);
        if (denom <= 0.0) return 0.0;
        return Math.sqrt((g * x * x) / denom);
    }

    private static double velocityToRadPerSec(double vExit) {
        double eff = COMPRESSION_EFFICIENCY * SLIP_EFFICIENCY;
        double denominator = (MAIN_ROLLER_RADIUS_METERS + (HELPER_RATIO * HELPER_ROLLER_RADIUS_METERS)) * eff;
        return (2.0 * vExit) / denominator;
    }

    private static double estimateSpinRPS(double vExit) {
        double omegaMain = velocityToRadPerSec(vExit);
        double vMain = omegaMain * MAIN_ROLLER_RADIUS_METERS;
        double vHelper = (omegaMain * HELPER_RATIO) * HELPER_ROLLER_RADIUS_METERS;
        double tangentialDiff = vMain - vHelper;
        return tangentialDiff / (TWO_PI * FUEL_RADIUS_METERS);
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    /* ---------------- Boilerplate & Logging ---------------- */

    public record ShooterSolution(double rollerOmega, double hoodAngleRad, double turretAngleRad, double distanceMeters) {}

    @Log.NT public double getRollerRadPerSecSolution() { return currentSolution.rollerOmega; }
    @Log.NT public double getHoodAngleRadSolution() { return currentSolution.hoodAngleRad; }
    @Log.NT public double getTurretAngleRadSolution() { return currentSolution.turretAngleRad; }
    @Log.NT public double getTargetDistance() { return currentSolution.distanceMeters; }
}