package frc.robot.subsystems.shooter;

import frc.excalib.control.gains.Gains;

/**
 * Constants for the Shooter subsystem including motor IDs, PID gains, and preset angles.
 */
public class ShooterConstants {
    // ===== CAN IDs =====
    public static final int HOOD_MOTOR_ID = 30;
    public static final int FLYWHEEL_MOTOR_LOW_ID = 31;
    public static final int FLYWHEEL_MOTOR_TOP_ID = 32;
    public static final int HOOD_ENCODER_ID = 34;

    // ===== Flywheel Constants =====
    public static final double FLYWHEEL_MAX_ACCELERATION = 30;
    public static final double FLYWHEEL_TOLERANCE = 0.5;

    public static final double HOOD_TOLERANCE = 0.02;
    public static final double FLYWHEEL_MAX_JERK = 0;
//    public static final Gains FLYWHEEL_GAINS = new Gains(0.2, 0, 0, 0.33, 0.156*0.93, 0, 0);
    public static final Gains FLYWHEEL_GAINS = new Gains();

    // ===== Hood Angle Constants =====
    public static final double HOOD_MAX_ANGLE_LIMIT_IN_TRENCH = 0.2;
    public static final double HOOD_MIN_ANGLE_LIMIT = 0;
    public static final double HOOD_MAX_ANGLE_LIMIT = 0.9;
    public static final double POSITION_CONVERSION_FACTOR = -0.590137;
//    public static final Gains HOOD_GAINS = new Gains(10.8, 0, 0.15, 0.3125, 0, 0, 0);
    public static final Gains HOOD_GAINS = new Gains();

}


