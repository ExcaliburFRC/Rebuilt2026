package frc.robot.subsystems.shooter;

import static frc.robot.Constants.FieldConstants.FUEL_DIAMETER;

/**
 * ============================================================================
 *  BALLISTIC MODEL TUNING TABLE  —  *** FILL IN THE REAL ROBOT NUMBERS HERE ***
 * ============================================================================
 *
 * This is the single place where every physical / mechanical property used by
 * the new physics-based shooter math lives. The solver ({@link BallisticSolver})
 * and {@link Shooter} read everything from here, so you only ever edit this file
 * to re-tune the shot.
 *
 * Every value still carrying a "TODO: measure" comment is a placeholder I could
 * not know — replace it with a measured value off the robot. Search this file
 * for "TODO" to find them all.
 *
 * UNITS (be strict — the physics depends on it):
 *   - lengths / heights : METERS
 *   - angles            : RADIANS, measured from the HORIZONTAL (0 = flat shot,
 *                         +pi/2 = straight up)
 *   - speeds            : METERS PER SECOND for the ball; ROTATIONS PER SECOND
 *                         for the flywheel setpoint (whatever unit the FlyWheel
 *                         mechanism is configured in)
 *   - mass              : KILOGRAMS
 */
public final class BallisticConstants {

    private BallisticConstants() {}

    // ========================================================================
    // SECTION A — LAUNCH GEOMETRY
    // ========================================================================

    /**
     * Height of the ball above the floor at the instant it leaves the shooter.
     * If the exit height changes appreciably as the hood moves, pick the value
     * at a typical shooting hood angle (or wire this to a function of hood angle
     * later — see Shooter#exitHeightForGoal).
     */
    public static final double BALL_EXIT_HEIGHT_M = 0.60; // TODO: measure

    /** Height of the HIGH goal opening the ball must drop into. */
    public static final double HIGH_GOAL_HEIGHT_M = 2.64; // TODO: measure

    /** Height of the LOW goal opening. */
    public static final double LOW_GOAL_HEIGHT_M = 0.90; // TODO: measure

    /**
     * Desired descent angle into each goal, measured DOWNWARD from horizontal.
     * This is the extra constraint that makes the (hood-angle, flywheel-speed)
     * pair unique. A steeper angle drops more cleanly into a high opening but
     * needs more flywheel speed. 60 deg is a sane starting point for a high goal.
     */
    public static final double HIGH_GOAL_DESCENT_ANGLE_RAD = Math.toRadians(60.0); // TODO: tune
    public static final double LOW_GOAL_DESCENT_ANGLE_RAD  = Math.toRadians(20.0); // TODO: tune

    // ========================================================================
    // SECTION B — BALL & AIR (drag model)
    // ========================================================================

    /** Mass of one fuel ball. */
    public static final double BALL_MASS_KG = 0.2164; // TODO: measure

    /** Ball diameter — pulled from the field constant so there is one source of truth. */
    public static final double BALL_DIAMETER_M = FUEL_DIAMETER; // = 0.15 (from FieldConstants)

    /** Aerodynamic drag coefficient. A smooth sphere is ~0.47; foam/dimpled differs. */
    public static final double DRAG_COEFFICIENT = 0.50; // TODO: tune

    /** Air density (kg/m^3). 1.205 at sea level / 20 C. Lower it for high-altitude events. */
    public static final double AIR_DENSITY = 1.205; // TODO: adjust for altitude if needed

    // ========================================================================
    // SECTION C — HOOD ANGLE  <->  PHYSICAL LAUNCH ANGLE
    // ------------------------------------------------------------------------
    // The solver produces a real launch angle (radians from horizontal). The
    // hood mechanism is commanded in the encoder "hood value" units already used
    // elsewhere in Shooter (the 0..~0.9 range). Define the straight-line mapping
    // between the two with two calibration points you measure on the robot:
    //
    //   point LOW :  hood value HOOD_CAL_VALUE_LOW  produces launch angle HOOD_CAL_ANGLE_LOW
    //   point HIGH:  hood value HOOD_CAL_VALUE_HIGH produces launch angle HOOD_CAL_ANGLE_HIGH
    //
    // e.g. "at hood 0.0 the ball leaves at 35 deg, at hood 0.9 it leaves at 80 deg".
    // ========================================================================

    public static final double HOOD_CAL_VALUE_LOW  = 0.0;                 // TODO: measure
    public static final double HOOD_CAL_ANGLE_LOW  = Math.toRadians(35.0); // TODO: measure

    public static final double HOOD_CAL_VALUE_HIGH = 0.9;                  // TODO: measure
    public static final double HOOD_CAL_ANGLE_HIGH = Math.toRadians(80.0); // TODO: measure

    /** Converts a physical launch angle (rad from horizontal) into a hood encoder value. */
    public static double hoodValueForLaunchAngle(double launchAngleRad) {
        double slope = (HOOD_CAL_VALUE_HIGH - HOOD_CAL_VALUE_LOW)
                     / (HOOD_CAL_ANGLE_HIGH - HOOD_CAL_ANGLE_LOW);
        return HOOD_CAL_VALUE_LOW + slope * (launchAngleRad - HOOD_CAL_ANGLE_LOW);
    }

    /** Inverse of {@link #hoodValueForLaunchAngle} — handy for clamping / display. */
    public static double launchAngleForHoodValue(double hoodValue) {
        double slope = (HOOD_CAL_ANGLE_HIGH - HOOD_CAL_ANGLE_LOW)
                     / (HOOD_CAL_VALUE_HIGH - HOOD_CAL_VALUE_LOW);
        return HOOD_CAL_ANGLE_LOW + slope * (hoodValue - HOOD_CAL_VALUE_LOW);
    }

    // ========================================================================
    // SECTION D — BALL EXIT SPEED  <->  FLYWHEEL SETPOINT
    // ------------------------------------------------------------------------
    // The solver produces the required ball exit speed (m/s). Convert it to the
    // flywheel velocity setpoint the FlyWheel mechanism expects.
    //
    //   wheel surface speed = ball speed / efficiency        (foam slips on wheel)
    //   wheel rev/s         = wheel surface speed / (pi * wheel diameter)
    //   setpoint            = wheel rev/s * SETPOINT_PER_WHEEL_REV
    //
    // SETPOINT_PER_WHEEL_REV is the gear factor between the flywheel-mechanism
    // setpoint unit and one revolution of the physical launching wheel. If the
    // setpoint IS the wheel's RPS, leave it 1.0. If the setpoint is the motor's
    // RPS through a reduction, set it to that ratio.
    // ========================================================================

    /** Diameter of the physical wheel that contacts the ball. */
    public static final double FLYWHEEL_WHEEL_DIAMETER_M = 0.10; // TODO: measure

    /**
     * Speed transfer efficiency = (ball surface speed) / (wheel surface speed).
     * Foam-on-wheel slip is typically 0.4..0.7. 1.0 means no slip.
     */
    public static final double FLYWHEEL_SPEED_TRANSFER_EFFICIENCY = 0.50; // TODO: measure

    /** Flywheel-setpoint units per one revolution of the launching wheel. */
    public static final double SETPOINT_PER_WHEEL_REV = 1.0; // TODO: confirm gearing

    /** Converts required ball exit speed (m/s) into a flywheel velocity setpoint. */
    public static double flywheelSetpointForBallSpeed(double ballSpeedMps) {
        double wheelSurfaceSpeed = ballSpeedMps / FLYWHEEL_SPEED_TRANSFER_EFFICIENCY;
        double wheelRevPerSec = wheelSurfaceSpeed / (Math.PI * FLYWHEEL_WHEEL_DIAMETER_M);
        return wheelRevPerSec * SETPOINT_PER_WHEEL_REV;
    }

    // ========================================================================
    // SECTION E — SOLVER NUMERICS (CPU vs accuracy — usually leave as-is)
    // ========================================================================

    /** Gravitational acceleration. */
    public static final double GRAVITY = 9.80665;

    /** Integration time step for the trajectory (s). Smaller = more accurate, more CPU. */
    public static final double INTEGRATION_DT = 0.004;

    /** Hard cap on simulated flight time (s) so a bad guess can't loop forever. */
    public static final double MAX_FLIGHT_TIME = 4.0;

    /** Ball exit-speed search bounds (m/s) for the inner solve. */
    public static final double SPEED_SEARCH_MIN_MPS = 1.0;
    public static final double SPEED_SEARCH_MAX_MPS = 35.0; // TODO: cap to your real max wheel speed
    public static final int    SPEED_SEARCH_ITERATIONS = 18;

    /** Launch-angle search bounds (rad) for the outer solve. */
    public static final double ANGLE_SEARCH_MIN_RAD = Math.toRadians(20.0);
    public static final double ANGLE_SEARCH_MAX_RAD = Math.toRadians(82.0);
    public static final int    ANGLE_SEARCH_ITERATIONS = 16;

    /** Fixed-point iterations for the shoot-while-moving (shot lead) loop. */
    public static final int LEAD_ITERATIONS = 3;

    // ---- Precomputed drag term: k = 0.5 * rho * Cd * A / m  (so a_drag = -k*|v|*v) ----
    public static final double DRAG_K;
    static {
        double area = Math.PI * (BALL_DIAMETER_M / 2.0) * (BALL_DIAMETER_M / 2.0);
        DRAG_K = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * area / BALL_MASS_KG;
    }
}
