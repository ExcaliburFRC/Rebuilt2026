package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.limits.SoftLimit;

public class TurretConstants {
    public static final double ROTATIONS_TO_RAD = 2 * Math.PI;

    public static final int TURRET_MOTOR_ID = 32;
    public static final int TURRET_ENCODER_ID = 31;
    public static final double MIN_LIMIT = -1.1 * Math.PI;
    public static final double MAX_LIMIT = 1.1 * Math.PI;
    public static final double PID_TOLERANCE = 0.02
            ;

    public static final ContinuousSoftLimit TURRET_CONTINUOUS_SOFTLIMIT = new ContinuousSoftLimit(
            () -> MIN_LIMIT,
            () -> MAX_LIMIT
    );

    public static final SoftLimit SOFT_LIMIT = new SoftLimit(()-> MIN_LIMIT, ()-> MAX_LIMIT);
    public static final Gains TURRET_GAINS = new Gains(3.5, 0, 0,0.485,0,0,0);
    public static final Translation2d TURRET_OFFSET_RELATIVE_ROBOT = new Translation2d();
    public static final double ENCODER_POSITION_CONVERSION_FACTOR = 0.2755698424688534;
    public static final double MOTOR_POSITION_CONVERSION_FACTOR = 0.17532001955723459; //
}
