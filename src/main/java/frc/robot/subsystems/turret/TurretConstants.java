package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;

public class TurretConstants {
    public static final double ROTATIONS_TO_RAD = 2 * Math.PI;

    public static final int TURRET_MOTOR_ID = 32;
    public static final int TURRET_ENCODER_ID = 31;
    public static final double MIN_LIMIT = -Math.PI;
    public static final double MAX_LIMIT = Math.PI;
    public static final double PID_TOLERANCE = 0;

    public static final ContinuousSoftLimit TURRET_CONTINUOUS_SOFTLIMIT = new ContinuousSoftLimit(
            () -> MIN_LIMIT,
            () -> MAX_LIMIT
    );

    public static final Gains TURRET_GAINS = new Gains(4.6,0.6,0);
    public static final Translation2d TURRET_OFFSET_RELATIVE_ROBOT = new Translation2d();
    public static final double ENCODER_POSITION_CONVERSION_FACTOR = 0.04412283798;
    public static final double MOTOR_POSITION_CONVERSION_FACTOR =-0.02756947507;
}
