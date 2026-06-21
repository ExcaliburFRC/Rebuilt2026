package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.BallisticConstants.*;

/**
 * Physics-based shot solver.
 *
 * Replaces the old distance->value lookup tables. For a given horizontal range
 * and height difference to the target, it numerically integrates projectile
 * motion WITH aerodynamic drag and finds the unique (launch angle, exit speed)
 * pair that both:
 *   1. passes the ball through the target point, and
 *   2. arrives with a chosen descent angle (so it drops cleanly into the goal).
 *
 * The search is a nested 1-D bisection:
 *   - OUTER: choose the launch angle so the descent angle at the target matches
 *            the requested descent angle.
 *   - INNER: for that launch angle, choose the exit speed so the trajectory is
 *            exactly at the target height when it reaches the target range.
 *
 * Everything tunable lives in {@link BallisticConstants}. This class holds no
 * robot-specific numbers.
 */
public class BallisticSolver {

    /**
     * @param launchAngleRad launch angle from horizontal (feeds the hood)
     * @param ballSpeedMps   ball exit speed relative to the robot (feeds the flywheel)
     * @param timeOfFlight   simulated flight time, used by the shot-lead loop
     * @param valid          false if no physically reachable solution was found
     */
    public record ShotSolution(double launchAngleRad, double ballSpeedMps,
                               double timeOfFlight, boolean valid) {}

    public static final ShotSolution INVALID = new ShotSolution(0, 0, 0, false);

    /**
     * Solve for a shot.
     *
     * @param horizontalRange     horizontal ground distance from the shooter to the target (m)
     * @param heightDelta         target height minus ball exit height (m); positive = target is higher
     * @param desiredDescentAngle desired descent angle into the target, downward from horizontal (rad)
     */
    public ShotSolution solve(double horizontalRange, double heightDelta, double desiredDescentAngle) {
        if (horizontalRange < 1e-3) return INVALID;

        double loAngle = ANGLE_SEARCH_MIN_RAD;
        double hiAngle = ANGLE_SEARCH_MAX_RAD;

        double fLo = descentError(loAngle, horizontalRange, heightDelta, desiredDescentAngle);
        double fHi = descentError(hiAngle, horizontalRange, heightDelta, desiredDescentAngle);

        // descentError is increasing in launch angle (steeper launch -> steeper descent).
        // If the root is not bracketed, the requested descent angle is unreachable at this
        // range; fall back to the bound that gets closest to it.
        if (Double.isNaN(fLo) && Double.isNaN(fHi)) return INVALID;
        if (Double.isNaN(fLo)) loAngle = nudgeToValid(loAngle, hiAngle, horizontalRange, heightDelta, true);
        if (Double.isNaN(fHi)) hiAngle = nudgeToValid(hiAngle, loAngle, horizontalRange, heightDelta, false);

        fLo = descentError(loAngle, horizontalRange, heightDelta, desiredDescentAngle);
        fHi = descentError(hiAngle, horizontalRange, heightDelta, desiredDescentAngle);
        if (Double.isNaN(fLo) || Double.isNaN(fHi)) return INVALID;

        double angle;
        if (fLo * fHi > 0) {
            // Not bracketed: pick the endpoint with the smaller descent error.
            angle = Math.abs(fLo) <= Math.abs(fHi) ? loAngle : hiAngle;
        } else {
            for (int i = 0; i < ANGLE_SEARCH_ITERATIONS; i++) {
                double mid = 0.5 * (loAngle + hiAngle);
                double fMid = descentError(mid, horizontalRange, heightDelta, desiredDescentAngle);
                if (Double.isNaN(fMid)) { loAngle = mid; continue; }
                if (fMid > 0) hiAngle = mid; else loAngle = mid;
            }
            angle = 0.5 * (loAngle + hiAngle);
        }

        double speed = solveSpeed(angle, horizontalRange, heightDelta);
        if (Double.isNaN(speed)) return INVALID;

        double[] state = simulateToRange(angle, speed, horizontalRange);
        if (state == null) return INVALID;

        return new ShotSolution(angle, speed, state[2], true);
    }

    // ---- outer-search helper: descent angle error for a given launch angle ----
    private double descentError(double launchAngle, double range, double heightDelta, double desiredDescent) {
        double speed = solveSpeed(launchAngle, range, heightDelta);
        if (Double.isNaN(speed)) return Double.NaN;
        double[] state = simulateToRange(launchAngle, speed, range);
        if (state == null) return Double.NaN;
        double descent = state[1]; // already downward-positive
        return descent - desiredDescent;
    }

    // Walk from `from` toward `toward` until a launch angle produces a valid speed solve.
    private double nudgeToValid(double from, double toward, double range, double heightDelta, boolean unusedDir) {
        int steps = 8;
        for (int i = 1; i <= steps; i++) {
            double a = from + (toward - from) * ((double) i / steps);
            if (!Double.isNaN(solveSpeed(a, range, heightDelta))) return a;
        }
        return from;
    }

    /**
     * Inner solve: for a fixed launch angle, find the exit speed so the trajectory
     * height equals heightDelta when it reaches the target range. Height-at-range is
     * monotonic increasing in speed, so a bisection converges.
     *
     * @return NaN if even the max speed cannot reach the target height/range.
     */
    private double solveSpeed(double launchAngle, double range, double heightDelta) {
        double lo = SPEED_SEARCH_MIN_MPS;
        double hi = SPEED_SEARCH_MAX_MPS;

        if (heightAtRange(launchAngle, hi, range) < heightDelta) return Double.NaN; // unreachable

        for (int i = 0; i < SPEED_SEARCH_ITERATIONS; i++) {
            double mid = 0.5 * (lo + hi);
            if (heightAtRange(launchAngle, mid, range) < heightDelta) lo = mid; else hi = mid;
        }
        return 0.5 * (lo + hi);
    }

    private double heightAtRange(double launchAngle, double speed, double range) {
        double[] state = simulateToRange(launchAngle, speed, range);
        return state == null ? Double.NEGATIVE_INFINITY : state[0];
    }

    /**
     * Integrate the trajectory (semi-implicit Euler) until the ball reaches the
     * target horizontal range, then linearly interpolate the crossing.
     *
     * Returns {height, descentAngle (downward-positive), time} at x == range, or
     * null if the ball never reaches that range (fell short).
     */
    private double[] simulateToRange(double launchAngle, double speed, double range) {
        double x = 0.0, y = 0.0;
        double vx = speed * Math.cos(launchAngle);
        double vy = speed * Math.sin(launchAngle);
        double t = 0.0;

        double px = x, py = y, pvx = vx, pvy = vy, pt = t;

        while (t < MAX_FLIGHT_TIME) {
            if (x >= range && x > px) {
                double frac = (range - px) / (x - px);
                double yAt = py + frac * (y - py);
                double vxAt = pvx + frac * (vx - pvx);
                double vyAt = pvy + frac * (vy - pvy);
                double tAt = pt + frac * (t - pt);
                double descent = Math.atan2(-vyAt, vxAt); // >0 when descending
                return new double[]{yAt, descent, tAt};
            }

            px = x; py = y; pvx = vx; pvy = vy; pt = t;

            double v = Math.hypot(vx, vy);
            double ax = -DRAG_K * v * vx;
            double ay = -GRAVITY - DRAG_K * v * vy;

            vx += ax * INTEGRATION_DT;
            vy += ay * INTEGRATION_DT;
            x += vx * INTEGRATION_DT;
            y += vy * INTEGRATION_DT;
            t += INTEGRATION_DT;

            // If the ball is descending and has dropped far below the target, it
            // will never climb back — bail out as "fell short".
            if (vx <= 0) return null;
        }
        return null;
    }
}
