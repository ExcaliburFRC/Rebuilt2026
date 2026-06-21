package frc.robot.subsystems.transport;

import frc.excalib.control.gains.Gains;

public class TransportConstants {
    public static final int DRUM_MOTOR_ID = 20;
    //    public static final Gains GAINS = new Gains(1,0,0,0.3,0,0,0);
    public static final Gains DRUM_GAINS = new Gains(0, 0, 0, 1, 0.34, 0, 0);
    public static final double MAX_ACCELERATION = 10;
    public static final double MAX_JERK = 10;

    public static final double DRUM_TOLERANCE = 10;
    public static final double TRANSPORT_TOLERANCE = 10;
    public static final int TRANSPORT_MOTOR_ID = 33;
    public static final Gains TRANSPORT_PID_GAINS = new Gains(0, 0, 0, 1.4, 0.4, 0, 0);

    public static final double TRANSPORT_MAX_ACCELERATION = 10;
    public static final double TRANSPORT_MAX_JERK = 10;

    // ==== Velocity conversion factors (motor rotations -> linear units) ==== //
    public static final double DRUM_VELOCITY_CONVERSION_FACTOR = 0.39898 / 9;
    public static final double TRANSPORT_VELOCITY_CONVERSION_FACTOR = 0.0731;

    // ==== Manual override ==== //
    public static final double MANUAL_TRANSPORT_VELOCITY = 5;
}
