package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;
import monologue.Annotations;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.util.ShooterPhysicsConstants.*;

/**
 * Stateless shooter physics model driven by Pose2d.
 */
public final class ShooterPhysics implements Logged {

    private static final double TWO_PI = 2.0 * Math.PI;
    private static final double SEC_PER_MIN = 60.0;

    private final Supplier<Pose2d> robotPose;
    private final Pose2d hub;
    private final DoubleSupplier robotForwardVelocity;
    private final DoubleSupplier robotYawRateRadPerSec;

    public ShooterSolution currentSolution = new ShooterSolution(0, 0, 0, 0);

    private final InterpolatingDoubleTreeMap entryAngleMap;

    public ShooterPhysics(Supplier<Pose2d> robotPose, DoubleSupplier robotForwardVelocity, DoubleSupplier robotYawRateRadPerSec) {
        this.robotPose = robotPose;
        this.hub = Constants.FieldConstants.BLUE_HUB_CENTER_POSE.getAsCurrentAlliance();
        this.robotForwardVelocity = robotForwardVelocity;
        this.robotYawRateRadPerSec = robotYawRateRadPerSec;

        this.entryAngleMap = new InterpolatingDoubleTreeMap();
        this.entryAngleMap.put(MIN_DISTANCE, ENTRY_ANGLE_CLOSE_RAD);
        this.entryAngleMap.put(MAX_DISTANCE, ENTRY_ANGLE_FAR_RAD);
    }


    public void solve() {
        Pose2d robot = robotPose.get();

        double dx = hub.getX() - robot.getX();
        double dy = hub.getY() - robot.getY();
        double distance = Math.hypot(dx, dy);

        double deltaH = HUB_HEIGHT_METERS - HUB_ENTRY_OFFSET_METERS - SHOOTER_EXIT_HEIGHT_METERS;
        double entryAngle = entryAngleMap.get(distance);

        /* ---------- Iterative Ballistic Solver ---------- */
        // 1. Initial A for launch angle
        double hoodAngle = clamp(calculateLaunchAngle(distance, deltaH, entryAngle), MIN_HOOD_ANGLE_RAD, MAX_HOOD_ANGLE_RAD);

        // 2. Iteratively solve for velocity considering Drag and Magnus
        double vExit = calculateExitVelocity(distance, deltaH, hoodAngle);

        // Run 3 iterations to converge on the correct velocity
        for (int i = 0; i < 3; i++) {
            double flightTime = distance / (vExit * Math.cos(hoodAngle));

            // Calculate average velocity loss due to drag: F_drag = -k * v
            double dragLoss = 1.0 + (DRAG_COEFFICIENT * distance);

            // Calculate lift acceleration: a_magnus = C * v * spin
            double spinRPS = estimateSpinRPS(vExit);
            double accelMagnus = MAGNUS_COEFFICIENT * spinRPS * GRAVITY;
            double effectiveGravity = GRAVITY - accelMagnus;

            // Recalculate required velocity with the "new" physics environment
            vExit = calculateExitVelocityComplex(distance, deltaH, hoodAngle, effectiveGravity) * dragLoss;
        }

        // 3. Compensation for Robot Movement
        // Subtract forward component of robot velocity from required exit velocity
        vExit -= robotForwardVelocity.getAsDouble();
        double rpm = velocityToRPM(vExit);

        /* ---------- Turret Solution ---------- */
        double absoluteTargetAngle = Math.atan2(dy, dx);
        double robotYaw = robot.getRotation().getRadians();

        // Account for flight time in the turret lead
        double flightTime = distance / (vExit * Math.cos(hoodAngle));
        double yawLead = robotYawRateRadPerSec.getAsDouble() * flightTime;

        double turretAngle = Rotation2d.fromRadians(absoluteTargetAngle).minus(Rotation2d.fromRadians(robotYaw)).getRadians() + yawLead;

        currentSolution = new ShooterSolution(rpm, hoodAngle, turretAngle, distance);
    }

    /**
     * Enhanced velocity calculation that allows for variable gravity (Magnus effect).
     */
    private static double calculateExitVelocityComplex(double x, double h, double theta, double g) {
        double cos = Math.cos(theta);
        double denom = 2.0 * cos * cos * (x * Math.tan(theta) - h);
        if (denom <= 0.0) return 0.0;
        return Math.sqrt((g * x * x) / denom);
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

    public record ShooterSolution(double rollerRPM, double hoodAngleRad, double turretAngleRad, double distanceMeters) {
    }

    @Log.NT
    public double getRollerRPMSolution(){
        return currentSolution.rollerRPM;
    }

    @Log.NT
    public double getHoodAngleRadSolution(){
        return currentSolution.hoodAngleRad;
    }

    @Log.NT
    public double getTurretAngleRadSolution(){
        return currentSolution.turretAngleRad;
    }
}
