package frc.excalib2.control;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

/**
 * Motion-profile constraints for MotionMagic, in <b>mechanism units</b> (rotations,
 * rotations/s, rotations/s²) — the device works in mechanism units because
 * {@code SensorToMechanismRatio} is configured on the motor.
 *
 * <p>Two flavors:
 * <ul>
 *   <li><b>Expo</b> ({@link #expo}) — exponential profile shaped by the motor model
 *       ({@code kV}/{@code kA}, always in volts even under TorqueCurrentFOC). Time-optimal,
 *       smoother and more efficient than trapezoidal; <b>preferred for positional moves</b>.
 *       An optional cruise-velocity cap bounds top speed.</li>
 *   <li><b>Trapezoidal</b> ({@link #trapezoidal}) — classic cruise/accel/jerk profile. Use when
 *       constant cruise behavior is specifically wanted (e.g. synchronized mechanisms) or for
 *       {@code MotionMagicVelocity} ramping.</li>
 * </ul>
 */
public record MotionConstraints(
        double cruiseVelocity,
        double acceleration,
        double jerk,
        double expoKV,
        double expoKA,
        boolean useExpo) {

    /** Trapezoidal profile: cruise velocity (units/s), acceleration (units/s²), jerk (units/s³, 0 = disabled). */
    public static MotionConstraints trapezoidal(double cruiseVelocity, double acceleration, double jerk) {
        return new MotionConstraints(cruiseVelocity, acceleration, jerk, 0, 0, false);
    }

    /**
     * Exponential profile with no cruise cap (fully time-optimal).
     *
     * @param expoKV volts per mechanism-unit/s (start: 12 / free-speed-in-mechanism-units)
     * @param expoKA volts per mechanism-unit/s² (higher = softer response)
     */
    public static MotionConstraints expo(double expoKV, double expoKA) {
        return new MotionConstraints(0, 0, 0, expoKV, expoKA, true);
    }

    /** Exponential profile with a cruise-velocity cap (units/s). */
    public static MotionConstraints expo(double expoKV, double expoKA, double cruiseVelocityCap) {
        return new MotionConstraints(cruiseVelocityCap, 0, 0, expoKV, expoKA, true);
    }

    /** Converts to Phoenix 6 MotionMagic configs. */
    public MotionMagicConfigs toMotionMagicConfigs() {
        MotionMagicConfigs configs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(cruiseVelocity)
                .withMotionMagicAcceleration(acceleration)
                .withMotionMagicJerk(jerk);
        if (useExpo) {
            configs.withMotionMagicExpo_kV(expoKV).withMotionMagicExpo_kA(expoKA);
        }
        return configs;
    }
}
