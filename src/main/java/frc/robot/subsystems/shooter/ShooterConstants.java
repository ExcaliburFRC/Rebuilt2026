package frc.robot.subsystems.shooter;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.excalib.control.gains.Gains;

public class ShooterConstants {
    public static final int HOOD_MOTOR_ID = 30;
    public static final int FLYWHEEL_MOTOR_ID = 31;
    public static final int TRANSPORT_MOTOR_ID = 33;
    public static final int HOOD_ENCODER_ID = 34;

    public static final double FLY_WHEEL_MAX_ACCELERATION = 3;
    public static final double FLY_WHEEL_TOLERANCE = 1;

    public static final double DEFAULT_FLYWHEEL_VELOCITY = 3;

    public static final double HOOD_MAX_ANGLE_LIMIT_IN_TRENCH = 0.8;
    public static final double HOOD_MIN_ANGLE_LIMIT = 0.830093;
    public static final double HOOD_MAX_ANGLE_LIMIT = 1.320525;
    public static final double FLY_WHEEL_MAX_JERK = 0;

    public static final double TRANSPORT_VOLTAGE = 0;
    public static final Gains FLYWHEEL_GAINS = new Gains(1,0,0,0, 0.173, 0, 0);
    public static final double POSITION_CONVERSION_FACTOR = 0.350566;
    public static final Gains HOOD_PID_GAINS = new Gains(21, 0, 0);
    public static final int FINAL_VEL = 0;
    public static final double STATIC_SHOOTING_VELOCITY = 0;
}
