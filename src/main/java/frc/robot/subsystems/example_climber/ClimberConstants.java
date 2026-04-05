package frc.robot.subsystems.example_climber;

// ═══════════════════════════════════════════════════════════════════
//  STEP 1 — CONSTANTS
//  Put all numbers here. No magic numbers anywhere else.
// ═══════════════════════════════════════════════════════════════════

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.excalib.control.gains.Gains;

public class ClimberConstants {

    // ── CAN IDs ──────────────────────────────────────────────────
    public static final int ARM_MOTOR_ID   = 50;
    public static final int WINCH_MOTOR_ID = 51;
    public static final int ARM_ENCODER_ID = 52;

    // ── Arm ──────────────────────────────────────────────────────
    /** Gear ratio from motor shaft to arm output shaft. */
    public static final double ARM_GEAR_RATIO       = 60.0;
    public static final double ARM_MIN_ANGLE_RAD    = 0.0;
    public static final double ARM_MAX_ANGLE_RAD    = Math.PI;          // 180°
    public static final double ARM_STOW_ANGLE_RAD   = 0.05;
    public static final double ARM_CLIMB_ANGLE_RAD  = Math.toRadians(120);
    public static final double ARM_TOLERANCE_RAD    = 0.05;
    public static final double ARM_MASS_KG          = 3.0;
    public static final double ARM_LENGTH_M         = 0.4;

    /** Converts encoder rotations → radians at the output shaft. */
    public static final double ARM_POSITION_CONVERSION = (2 * Math.PI) / ARM_GEAR_RATIO;

    public static final Gains ARM_GAINS = new Gains(
            8.0, 0.0, 0.1,      // kp, ki, kd
            0.0, 0.0, 0.0, 0.9  // ks, kv, ka, kg
    );

    // ── Winch ─────────────────────────────────────────────────────
    public static final double WINCH_GEAR_RATIO          = 30.0;
    public static final double WINCH_DRUM_RADIUS_M       = 0.015;       // 15 mm drum
    public static final double WINCH_MIN_LENGTH_M        = 0.0;
    public static final double WINCH_MAX_LENGTH_M        = 0.5;
    public static final double WINCH_CARRIAGE_MASS_KG    = 4.0;
    public static final double WINCH_TOLERANCE_M         = 0.01;
    public static final double WINCH_RETRACT_LENGTH_M    = 0.0;
    public static final double WINCH_EXTEND_LENGTH_M     = 0.45;

    /** Converts motor rotations → metres of rope retracted. */
    public static final double WINCH_POSITION_CONVERSION =
            (2 * Math.PI * WINCH_DRUM_RADIUS_M) / WINCH_GEAR_RATIO;

    public static final Gains WINCH_GAINS = new Gains(
            15.0, 0.0, 0.2,     // kp, ki, kd
            0.0, 0.0, 0.0, 0.0  // ks, kv, ka, kg
    );

    public static final TrapezoidProfile.Constraints WINCH_CONSTRAINTS =
            new TrapezoidProfile.Constraints(0.5, 0.25); // m/s, m/s²
}
