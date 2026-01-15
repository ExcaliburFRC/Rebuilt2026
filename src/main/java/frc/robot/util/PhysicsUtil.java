package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

/**
 * Physics utility for the C-Shooter
 */
public class PhysicsUtil {
    public static final double G_FORCE = 9.80665;
    public static final double SEC_IN_MIN = 60.0;
    public static final double ROTATION_IN_RAD = 2 * Math.PI;

    public static final double FUEL_DIAMETER = Units.inchesToMeters(0);

    // Interpolation map: Distance (m) -> Desired Entry Angle (radians)
    // Note: Entry angle is negative because the ball is falling into the goal.
    public static final InterpolatingDoubleTreeMap targetEntryAngleMap;

    static {
        targetEntryAngleMap = new InterpolatingDoubleTreeMap();

        targetEntryAngleMap.put(0.5, Units.degreesToRadians(-80));
        targetEntryAngleMap.put(4.0, Units.degreesToRadians(-40));
    }

    /**
     * Step 1: Calculate the necessary Launch Angle (Theta) to achieve a desired Entry Angle (Phi).
     * This uses the trajectory equation: tan(phi) = tan(theta) - (gx / (v0^2 * cos^2(theta)))
     * simplified to: tan(theta) = (2h/x) - tan(phi)
     */
    public static double calculateLaunchAngle(double x, double h, double targetPhiRad) {
        double tanTheta = (2.0 * h / x) - Math.tan(targetPhiRad);
        return Math.atan(tanTheta);
    }

    /**
     * Step 2: Calculate the required exit velocity (v0) in m/s for a given coordinate and angle.
     */
    public static double calculateRequiredV0(double x, double h, double thetaRad) {
        double cosT = Math.cos(thetaRad);
        double tanT = Math.tan(thetaRad);

        double numerator = G_FORCE * Math.pow(x, 2);
        double denominator = 2 * Math.pow(cosT, 2) * (x * tanT - h);

        if (denominator <= 0) return -1; // Physics check: Target is unreachable at this angle

        return Math.sqrt(numerator / denominator);
    }

    /**
     * Step 3: Convert the required V0 to Motor RPM.
     * Based on: v0 = (omega * rMain + omega * rHood) / 2
     */
    public static double calculateRequiredRPM(double requiredV0, double rMainMeters, double rHoodMeters) {
        // Solving for omega: omega = (2 * v0) / (rMain + rHood)
        double omega = (2.0 * requiredV0) / (rMainMeters + rHoodMeters);
        return (omega * SEC_IN_MIN) / ROTATION_IN_RAD;
    }

    /**
     * Bonus: Calculate the resulting ball spin in Rotations Per Second (RPS).
     * Based on: omega_ball = (vMain - vHood) / dBall
     */
    public static double calculateBallSpinRPS(double motorRPM, double rMain, double rHood) {
        double motorOmega = (motorRPM * ROTATION_IN_RAD) / SEC_IN_MIN;
        double vMain = motorOmega * rMain;
        double vHood = motorOmega * rHood;

        double ballOmegaRadPerSec = (vMain - vHood) / FUEL_DIAMETER;
        return ballOmegaRadPerSec / ROTATION_IN_RAD;
    }

    /**
     * Master Method: Get the RPM and Angle needed for a specific distance.
     * Useful for your Shooting Command.
     */
    public static ShooterState getTargetShooterState(double distanceMeters, double deltaHMeters) {
        double rMain = Units.inchesToMeters(4.0);
        double rHood = Units.inchesToMeters(2.0);

        double targetPhi = targetEntryAngleMap.get(distanceMeters);
        double launchAngle = calculateLaunchAngle(distanceMeters, deltaHMeters, targetPhi);
        double v0 = calculateRequiredV0(distanceMeters, deltaHMeters, launchAngle);
        double rpm = calculateRequiredRPM(v0, rMain, rHood);

        return new ShooterState(rpm, launchAngle);
    }

    /**
     * Simple wrapper to hold the result
     */
        public record ShooterState(double rpm, double angleRad) {
    }
}