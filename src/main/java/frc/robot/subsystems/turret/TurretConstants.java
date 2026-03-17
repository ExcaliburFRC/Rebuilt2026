package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.limits.SoftLimit;

public class TurretConstants {
    public static final double ROTATIONS_TO_RAD = 2 * Math.PI;

    public static final int TURRET_MOTOR_ID = 42;
    public static final int CABLE_MOTOR_ID = 42;
    public static final int TURRET_ENCODER_ID = 41;
    public static final double MAX_LIMIT = 3.54;
    public static final double MIN_LIMIT = -1.5;
    public static final double PID_TOLERANCE = 0.01;

    public static final ContinuousSoftLimit TURRET_CONTINUOUS_SOFTLIMIT = new ContinuousSoftLimit(
            () -> MIN_LIMIT,
            () -> MAX_LIMIT
    );

    public static final SoftLimit SOFT_LIMIT = new SoftLimit(() -> MIN_LIMIT, () -> MAX_LIMIT);
    public static final Gains TURRET_GAINS = new Gains(4, 0.1, 0, 0.39, 0, 0, 0);

    public static final double ENCODER_POSITION_CONVERSION_FACTOR = -0.2 * 2 * Math.PI * 0.9823;
    public static final double MOTOR_POSITION_CONVERSION_FACTOR = 0.102434 * 2 * Math.PI / 5 * 0.9875; //
}