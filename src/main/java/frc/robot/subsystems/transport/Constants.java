package frc.robot.subsystems.transport;

import frc.excalib.control.gains.Gains;

public class Constants {
    public static final int DRUM_MOTOR_ID = 0; //TODO assign ID
    public static final int ENCODER_ID = 0; //TODO assign ID
    public static final Gains GAINS = new Gains();
    public static final double PIDTOLERANCE = 0; //TODO assign ID
    public static final int numOfSlots = 6;
    public static final double angleOfSlots = 2*Math.PI*numOfSlots;
}
