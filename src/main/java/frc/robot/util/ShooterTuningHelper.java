// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Tuning utility for shooter system performance optimization.
 * Provides methods to test and refine hood angles and flywheel velocities
 * in real-time from SmartDashboard.
 */
public class ShooterTuningHelper {

    // Tuning constants that can be adjusted via SmartDashboard
    private static double tunedHoodAngle = 0.5;
    private static double tunedFlyWheelVelocity = 20.0;
    private static double tuningDistance = 3.0; // meters

    /**
     * Initializes tuning widget on SmartDashboard for real-time shooter adjustment.
     * Call this once during RobotContainer initialization.
     */
    public static void initializeTuningWidgets() {
        SmartDashboard.putNumber("Shooter Tuning/Hood Angle", tunedHoodAngle);
        SmartDashboard.putNumber("Shooter Tuning/Flywheel Velocity", tunedFlyWheelVelocity);
        SmartDashboard.putNumber("Shooter Tuning/Test Distance (m)", tuningDistance);
        SmartDashboard.putString("Shooter Tuning/Instructions",
            "Adjust sliders and press buttons to tune shooter");
    }

    /**
     * Updates tuning values from SmartDashboard.
     * Call in robotPeriodic() to read live adjustments.
     */
    public static void updateTuningValues() {
        tunedHoodAngle = SmartDashboard.getNumber("Shooter Tuning/Hood Angle", 0.5);
        tunedFlyWheelVelocity = SmartDashboard.getNumber("Shooter Tuning/Flywheel Velocity", 20.0);
        tuningDistance = SmartDashboard.getNumber("Shooter Tuning/Test Distance (m)", 3.0);
    }

    /**
     * Gets the current tuned hood angle
     */
    public static double getTunedHoodAngle() {
        return tunedHoodAngle;
    }

    /**
     * Gets the current tuned flywheel velocity
     */
    public static double getTunedFlyWheelVelocity() {
        return tunedFlyWheelVelocity;
    }

    /**
     * Gets the test distance for tuning
     */
    public static double getTuningDistance() {
        return tuningDistance;
    }

    /**
     * Records successful shot data to file for later analysis.
     * Helps build accurate distance-to-angle/velocity maps.
     */
    public static void recordSuccessfulShot(double distance, double hoodAngle, double flywheelVelocity) {
        String logEntry = String.format(
            "Distance: %.2f m, Hood: %.3f, Velocity: %.1f m/s",
            distance, hoodAngle, flywheelVelocity
        );
        System.out.println("Shot Recorded: " + logEntry);
        SmartDashboard.putString("Shooter Tuning/Last Shot", logEntry);
    }

    /**
     * Generates a recommendation for hood angle based on distance.
     * Uses linear interpolation between known good values.
     */
    public static double recommendHoodAngle(double distanceMeters) {
        // Interpolate between known good values
        // These should be updated based on actual testing
        if (distanceMeters <= 2.0) return 0.2;
        else if (distanceMeters <= 3.0) return 0.4;
        else if (distanceMeters <= 4.0) return 0.6;
        else return 0.8;
    }

    /**
     * Generates a recommendation for flywheel velocity based on distance.
     */
    public static double recommendFlywheelVelocity(double distanceMeters) {
        // Interpolate between known good values
        if (distanceMeters <= 2.0) return 18.0;
        else if (distanceMeters <= 3.0) return 22.0;
        else if (distanceMeters <= 4.0) return 25.0;
        else return 28.0;
    }
}

