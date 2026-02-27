package frc.robot.subsystems.intake;

import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;

public class IntakeConstants {

    // ==== ID's ==== //
    public static final int FOUR_BAR_MOTOR_ID = 10;
    public static final int ROLLER_MOTOR_ID = 11;
    public static final int ANGLE_ENCODER_ID = 12;


    // ==== Arm Initialization Constants ==== //
    public static final double ARM_MIN_VELOCITY_LIMIT = -1.5;
    public static final double ARM_MAX_VELOCITY_LIMIT = 1.5;
    public static final int ARM_MASS = 1;
    public static final double MAX_OFFSET = 0;
    public static final double INTAKE_MAX_ANGLE = 1;
    public static final double INTAKE_MIN_ANGLE = 0;

    // ==== Intake Angles and Tolerances ==== //
    public static final double INTAKE_ANGLE_TOLERANCE = 0.05;

    // ==== Other ====//
    public static final double ROTATION_TO_RAD = 2 * Math.PI;
    public static final SoftLimit ARM_VELOCITY_LIMIT = new SoftLimit(
            () -> ARM_MIN_VELOCITY_LIMIT,
            () -> ARM_MAX_VELOCITY_LIMIT
    );



}
