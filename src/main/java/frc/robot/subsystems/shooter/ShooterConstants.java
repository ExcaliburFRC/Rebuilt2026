package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import frc.excalib2.control.CurrentBudget;
import frc.excalib2.control.Gains;
import frc.excalib2.control.MotionConstraints;

/**
 * Shooter constants. Ratios are derived from the ExcaLib v1 conversion factors so device
 * positions match the pre-migration mechanism units exactly (derivations inline).
 * ⚠ All gains are VOLTAGE-mode ports of the v1 software-PID gains (V/rad → V/rotation);
 * the TorqueCurrentFOC switch is a planned SysId session.
 */
public class ShooterConstants {
    // ===== CAN IDs =====
    public static final int HOOD_MOTOR_ID = 30;
    public static final int FLYWHEEL_MOTOR_LOW_ID = 31;
    public static final int FLYWHEEL_MOTOR_TOP_ID = 32;
    public static final int HOOD_ENCODER_ID = 34;
    public static final int TURRET_MOTOR_ID = 42;
    public static final int TURRET_ENCODER_ID = 41;

    // ===== Flywheel =====
    /** v1 velocity factor 40/48 mech-units per motor rotation → rotor per mechanism = 48/40. */
    public static final double FLYWHEEL_ROTOR_PER_MECHANISM = 48.0 / 40.0;
    public static final double FLYWHEEL_TOLERANCE_RPS = 2;
    public static final Gains FLYWHEEL_GAINS = new Gains(0.2, 0, 0, 0.33, 0.156, 0);
    public static final Gains FLYWHEEL_SIM_GAINS = new Gains(0.05, 0, 0, 0, 0.11, 0);
    public static final CurrentBudget FLYWHEEL_BUDGET = new CurrentBudget(120, 80);

    // ===== Hood =====
    /**
     * v1 motor conversion: −0.590137 × (−0.208/1.497) × 1.0231 = 0.083893 rad per rotor
     * rotation → rotor rotations per hood rotation = 2π / 0.083893.
     */
    public static final double HOOD_ROTOR_PER_MECHANISM = 2 * Math.PI / 0.083893;
    /** v1: hood radians = CANcoder rotations × −0.590137 (used to seed at boot). */
    public static final double HOOD_RAD_PER_ENCODER_ROTATION = -0.590137;
    public static final double HOOD_MIN_ANGLE_RAD = 0;
    public static final double HOOD_MAX_ANGLE_RAD = 0.9;
    /** Under the trench the hood must stay down (v1 volatile trench limit). */
    public static final double HOOD_MAX_ANGLE_IN_TRENCH_RAD = 0;
    public static final double HOOD_TOLERANCE_RAD = 0.02;
    /**
     * v1: kP 10.8 V/rad, kD 0.15, asymmetric kS +0.375/−0.25 → symmetric 0.3125 (device kS
     * is symmetric; ±0.06 V delta noted as a behavior change).
     */
    public static final Gains HOOD_GAINS =
            new Gains(10.8 * 2 * Math.PI, 0, 0.15 * 2 * Math.PI, 0.3125, 0, 0);
    public static final Gains HOOD_SIM_GAINS = new Gains(40, 0, 0);
    public static final CurrentBudget HOOD_BUDGET = new CurrentBudget(40, 30);

    // ===== Turret =====
    /**
     * v1 motor conversion: 0.102434 × 2π/5 × 0.9875 × 0.9919 = 0.126083 rad per rotor
     * rotation → rotor rotations per turret rotation = 2π / 0.126083.
     */
    public static final double TURRET_ROTOR_PER_MECHANISM = 2 * Math.PI / 0.126083;
    /** v1: turret radians = CANcoder rotations × (−0.2 × 2π × 0.9823) (seed at boot). */
    public static final double TURRET_RAD_PER_ENCODER_ROTATION = -0.2 * 2 * Math.PI * 0.9823;
    public static final double TURRET_MIN_ANGLE_RAD = -5.4;
    public static final double TURRET_MAX_ANGLE_RAD = 1.5;
    public static final double TURRET_TOLERANCE_RAD = 0.04;
    public static final Gains TURRET_GAINS =
            new Gains(7 * 2 * Math.PI, 0, 0.4 * 2 * Math.PI, 0.4, 0, 0);
    public static final Gains TURRET_SIM_GAINS = new Gains(50, 0, 0);
    /** v1 profile: 2π rad/s cruise, 100π rad/s² accel. */
    public static final MotionConstraints TURRET_MOTION = new MotionConstraints(
            Units.radiansToRotations(2 * Math.PI), Units.radiansToRotations(100 * Math.PI), 0);
    public static final CurrentBudget TURRET_BUDGET = new CurrentBudget(120, 80);
}
