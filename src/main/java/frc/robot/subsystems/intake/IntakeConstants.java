package frc.robot.subsystems.intake;

import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;

public class IntakeConstants {

    // ==== ID's ==== //
    public static final int LEFT_FOUR_BAR_MOTOR_ID = 10;
    public static final int RIGHT_FOUR_BAR_MOTOR_ID = 34;
    public static final int ROLLER_MOTOR_ID = 13;
    public static final int ANGLE_ENCODER_ID = 12;


    // ==== Arm Initialization Constants ==== //
    public static final double ARM_MIN_VELOCITY_LIMIT = -1.5;
    public static final double ARM_MAX_VELOCITY_LIMIT = 1.5;
    // Max voltage the arm can output — tune this to control arm speed
    public static final double ARM_MAX_OUTPUT_VOLTAGE = 1.5;
    public static final int ARM_MASS = 1;
    public static final double MAX_OFFSET = 0;
    public static final double INTAKE_MAX_ANGLE = 2.4;
    public static final double INTAKE_MIN_ANGLE = 0;
    public static final Gains ARM_POSITION_GAINS = new Gains(2, 0, 0, 0, 0, 0, 0.8



    );


    // ==== Intake Angles and Tolerances ==== //
    public static final double INTAKE_ANGLE_TOLERANCE = 0.003;

    // ==== Per-state setpoints (consumed by IntakeStates) ==== //
    public static final double OPEN_ANGLE = 0;
    public static final double OPEN_ROLLER_VOLTAGE = 0.75;
    public static final double CLOSE_ANGLE = 2;
    public static final double CLOSE_ROLLER_VOLTAGE = 0;
    public static final double PUMP_ANGLE = 0;
    public static final double PUMP_ROLLER_VOLTAGE = 0.8;

    // ==== Manual / pump motion ==== //
    public static final double MANUAL_INTAKE_VOLTAGE = 0.5;
    public static final double PUMP_EXTENDED_ANGLE = 2;
    public static final double PUMP_RETRACTED_ANGLE = 0;
    public static final double PUMP_STEP_TIMEOUT_SEC = 0.5;

    // ==== Drivetrain conversion / calibration ==== //
    // armMotorGroup gear reduction: one full output rotation per 14.14 motor rotations.
    public static final double ARM_GEAR_RATIO = 14.14;
    public static final double ARM_POSITION_CONVERSION_FACTOR = 2 * Math.PI / ARM_GEAR_RATIO;
    // Absolute angle the arm boots at (mechanical hard stop), measured empirically.
    public static final double ARM_STARTING_ANGLE = Math.PI - 0.732601 - 0.159;

    // ==== Other ====//
    public static final double ROTATION_TO_RAD = 2 * Math.PI;
    public static final SoftLimit ARM_VELOCITY_LIMIT = new SoftLimit(
            () -> ARM_MIN_VELOCITY_LIMIT,
            () -> ARM_MAX_VELOCITY_LIMIT
    );



}
