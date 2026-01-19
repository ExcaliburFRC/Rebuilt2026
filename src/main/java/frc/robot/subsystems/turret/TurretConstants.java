package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;

public class TurretConstants {
    public static final int TURRET_MOTOR_ID = 0;
    public static final int TURRET_ENCODER_ID = 0;
    public static final double MIN_LIMIT = 0;
    public static final double MAX_LIMIT = 0;
    public static final ContinuousSoftLimit TURRET_CONTINOUS_SOFTLIMIT = new ContinuousSoftLimit(()-> MIN_LIMIT,
            ()-> MAX_LIMIT);
    public static final Gains TURRET_GAINS = new Gains();
    public static final double PID_TOLLERANCE = 0;
    public static final Translation2d TURRET_OFFSET_RELATIVE_ROBOT = new Translation2d();
    public static final double ROTATIONS_TO_RAD = 2 * Math.PI;
}
