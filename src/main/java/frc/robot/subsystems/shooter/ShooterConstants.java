package frc.robot.subsystems.shooter;

import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.limits.SoftLimit;

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
    public static final double FLYWHEEL_TOLERANCE = 2;

    public static final double HOOD_TOLERANCE = 0.02;
    public static final double FLYWHEEL_MAX_JERK = 0;
    public static final Gains FLYWHEEL_GAINS = new Gains(0.2, 0, 0, 0.33, 0.156, 0, 0);
//    public static final Gains FLYWHEEL_GAINS = new Gains();

    // ===== Hood Angle Constants =====
    public static final double HOOD_MAX_ANGLE_LIMIT_IN_TRENCH = 0;
    public static final double HOOD_MIN_ANGLE_LIMIT = 0;
    public static final double HOOD_MAX_ANGLE_LIMIT = 0.9;
    public static final double POSITION_CONVERSION_FACTOR = -0.590137;
    public static final Gains HOOD_GAINS = new Gains(10.8, 0, 0.15, 0.3125, 0, 0, 0);
//    public static final Gains HOOD_GAINS = new Gains();


    public static final int TURRET_MOTOR_ID = 42;
    public static final int TURRET_ENCODER_ID = 41;
    public static final double MAX_LIMIT = 1.5;
    public static final double MIN_LIMIT = -5.4;
    public static final double PID_TOLERANCE = 0.04;

    public static final ContinuousSoftLimit TURRET_CONTINUOUS_SOFTLIMIT = new ContinuousSoftLimit(
            () -> MIN_LIMIT,
            () -> MAX_LIMIT
    );

    public static final SoftLimit SOFT_LIMIT = new SoftLimit(() -> MIN_LIMIT, () -> MAX_LIMIT);
    public static final Gains TURRET_GAINS = new Gains(7, 0, 0.4, 0.4, 0, 0, 0);
//    public static final Gains TURRET_GAINS = new Gains();

    public static final double ENCODER_POSITION_CONVERSION_FACTOR = -0.2 * 2 * Math.PI * 0.9823;
    public static final double MOTOR_POSITION_CONVERSION_FACTOR = 0.102434 * 2 * Math.PI / 5 * 0.9875*0.9919; //

}


