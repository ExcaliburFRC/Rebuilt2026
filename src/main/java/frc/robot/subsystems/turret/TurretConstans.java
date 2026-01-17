package frc.robot.subsystems.turret;

import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;

public class TurretConstans {
    public static final int TURRET_MOTOR_ID = 0;
    public static final int TURRET_ENCODER_ID = 0;
    public static final ContinuousSoftLimit TURRET_CONTINOUS_SOFTLIMIT = new ContinuousSoftLimit(()-> 0,()-> 0);
    public static final Gains TURRET_GAINS = new Gains();
    public static final double PID_TOLLERANCE = 0;
    public static final double TURRET_DISTENCE_FROM_ROBOT = 0;
}
