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
    public static final double FLY_WHEEL_MAX_ACCELERATION = 3;
    public static final double FLY_WHEEL_TOLERANCE = 1;
    public static final double DEFAULT_FLYWHEEL_VELOCITY = 3;
    public static final double FLY_WHEEL_MAX_JERK = 0;
    public static final Gains FLYWHEEL_GAINS = new Gains(1, 0, 0, 0, 0.173, 0, 0);

    // ===== Hood Angle Constants =====
    public static final double HOOD_MAX_ANGLE_LIMIT_IN_TRENCH = 0.2;
    public static final double HOOD_MIN_ANGLE_LIMIT = 0;
    public static final double HOOD_MAX_ANGLE_LIMIT = 0.9;
    public static final double POSITION_CONVERSION_FACTOR = -0.590137;
    public static final Gains HOOD_PID_GAINS = new Gains(13.1, 0, 0, 0.425, 0, 0, 0);

    // ===== Hood Angle Presets (in radians, converted from 0-1 scale) =====
    // These presets provide fixed hood angles for manual control
    public static final double HOOD_ANGLE_FLAT = 0.0;        // Shoot straight
    public static final double HOOD_ANGLE_LOW = 0.3;         // Low angle shot (distance ~2-3m)
    public static final double HOOD_ANGLE_MEDIUM = 0.5;      // Medium angle shot (distance ~3-4m)
    public static final double HOOD_ANGLE_HIGH = 0.8;        // High angle shot (distance ~4-5m)

    // ===== Transport Constants =====
    public static final double TRANSPORT_VOLTAGE = 2;
    public static final int FINAL_VEL = 0;

    // ===== Velocity Limits =====
    public static final double MAX_VELOCITY = 30.0; // Max flywheel velocity in m/s
}
