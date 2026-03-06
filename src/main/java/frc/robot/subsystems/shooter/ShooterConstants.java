package frc.robot.subsystems.shooter;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.excalib.control.gains.Gains;

/**
 * Constants for the Shooter subsystem including motor IDs, PID gains, and preset angles.
 */
public class ShooterConstants {
    // ===== CAN IDs =====
    public static final int HOOD_MOTOR_ID = 30;
    public static final int FLYWHEEL_MOTOR_LOW_ID = 31;
    public static final int FLYWHEEL_MOTOR_TOP_ID = 32;
    public static final int TRANSPORT_MOTOR_ID = 33;
    public static final int HOOD_ENCODER_ID = 34;

    // ===== Flywheel Constants =====
    public static final double FLY_WHEEL_MAX_ACCELERATION = 30;
    public static final double FLY_WHEEL_TOLERANCE = 1;
    public static final double DEFAULT_FLYWHEEL_VELOCITY = 3;
    public static final double FLY_WHEEL_MAX_JERK = 0;
    public static final Gains FLYWHEEL_GAINS = new Gains(0.1, 0, 0, 0.3, 0.1581, 0, 0);

    // ===== Hood Angle Constants =====
    public static final double HOOD_MAX_ANGLE_LIMIT_IN_TRENCH = 0.2;
    public static final double HOOD_MIN_ANGLE_LIMIT = 0;
    public static final double HOOD_MAX_ANGLE_LIMIT = 0.9;
    public static final double POSITION_CONVERSION_FACTOR = -0.590137;
    public static final Gains HOOD_PID_GAINS = new Gains(13.1, 0, 0, 0.425, 0, 0, 0);

    // ===== Transport Constants =====
    public static final double TRANSPORT_VOLTAGE = -6;
}
