package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.excalib.control.gains.Gains;

public class ShooterConstants {
    public static final int HOOD_MOTOR_ID = 0;
    public static final int FLYWHEEL_MOTOR_ID = 0;
    public static final int SUPPORT_WHEEL_MOTOR_ID = 0;
    public static final int FLY_WHEEL_MAX_ACCELERATION = 0;
    public static final int TRANSPORT_MOTOR_ID = 0;
    public static final int TRANSPORT_VOLTAGE = 0;
    public static final int FLY_WHEEL_MAX_JERK = 0;
    public static final int HOOD_MIN_ANGLE_LIMIT = 0;
    public static final int HOOD_MAX_ANGLE_LIMIT = 0;
    public static final Gains FLYWHEEL_GAINS = new Gains();
    public static final double HOOD_MAX_VELOCITY_LIMIT = 0;
    public static final double HOOD_MAX_ACCELERATION_LIMIT = 0;
    public static final double POSITION_CONVERSION_FACTOR = 0;
    public static final Gains HOOD_PID_GAINS = new Gains(1, 1, 1);
    public static final int BEAM_BREAK_CHANNEL = 0;
    public static final TrapezoidProfile.Constraints HOOD_CONSTRAINTS = new TrapezoidProfile.Constraints(HOOD_MAX_VELOCITY_LIMIT, HOOD_MAX_ACCELERATION_LIMIT);
}
