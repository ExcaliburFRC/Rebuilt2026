package frc.robot.subsystems.transport;

import frc.excalib.control.gains.Gains;

public class TransportConstants {
    public static final int DRUM_MOTOR_ID = 20;
    public static final Gains GAINS = new Gains(1,0,0,0.23,0,0,0);
    public static final double TRANSPORT_VOLTAGE = 4.0;
    public static final double TRANSPORT_VELOCITY = 5.0; // m/s target velocity while feeding
    public static final double MAX_ACCELERATION = 10;
    public static final double MAX_JERK = 10;
}
