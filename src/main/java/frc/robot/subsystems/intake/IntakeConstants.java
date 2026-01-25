package frc.robot.subsystems.intake;

import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;

public class IntakeConstants {

    // ==== ID's ==== //
    public static final int FOUR_BAR_MOTOR_ID = 0;
    public static final int ROLLER_MOTOR_ID = 0;
    public static final int ANGLE_ENCODER_ID = 0;

    // ==== Voltages ==== //
    public static final int INTAKE_ROLLER_VOLTAGE = 0;
    public static final int STOW_ROLLER_VOLTAGE = 0;

    // ==== Arm Initialization Constants ==== //
    public static final int ARM_MIN_VELOCITY_LIMIT = 0;
    public static final int ARM_MAX_VELOCITY_LIMIT = 0;
    public static final int ARM_MASS = 0;
    public static final double MAX_OFFSET = 0;
    public static final double INTAKE_MAX_ANGLE = 0;
    public static final double INTAKE_MIN_ANGLE = 0;

    // ==== Intake Angles and Tolerances ==== //
    public static final int INTAKE_ANGLE_TOLERANCE = 0;

    // ==== Other ====//
    public static final Gains INTAKE_GAINS = new Gains();
    public static final double ROTATION_TO_RAD = 2 * Math.PI;
    public static final SoftLimit ARM_VELOCITY_LIMIT = new SoftLimit(
            () -> ARM_MIN_VELOCITY_LIMIT,
            () -> ARM_MAX_VELOCITY_LIMIT
    );
    public static final Gains ARM_POSITION_GAINS = new Gains();
    public static final double ARM_MASS_KG = 0;
}
