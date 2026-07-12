package frc.excalib.control.motor.motor_specs;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;

/**
 * Per-mechanism current allowance, applied as Phoenix 6 current-limit configs.
 *
 * <ul>
 *   <li><b>Stator limit</b> — caps torque and heating; the primary "how strong may this
 *       mechanism be" knob.</li>
 *   <li><b>Supply limit</b> — protects the breaker and the battery. When supply current
 *       exceeds {@code supplyLimit} for {@code supplyLowerTime} seconds it drops to
 *       {@code supplyLowerLimit} (breaker-trip protection / brownout mitigation).</li>
 *   <li><b>Torque-current peaks</b> — hard output clamp for TorqueCurrentFOC requests
 *       (which bypass the stator limit by design in Phoenix 6).</li>
 * </ul>
 *
 * <p>Keep every mechanism's budget in one named table (in {@code Constants}) so the whole
 * robot's current allocation is reviewable at a glance. Apply with
 * {@code TalonFXMotor.applyCurrentBudget(budget)}.
 */
public record CurrentBudget(
        double statorLimit,
        double supplyLimit,
        double supplyLowerLimit,
        double supplyLowerTime,
        double peakForwardTorqueCurrent,
        double peakReverseTorqueCurrent) {

    /**
     * Basic budget: stator + supply limits. Supply-lower defaults to the supply limit
     * (no time-based reduction); torque peaks default to the stator limit.
     */
    public CurrentBudget(double statorLimit, double supplyLimit) {
        this(statorLimit, supplyLimit, supplyLimit, 0, statorLimit, -statorLimit);
    }

    /** Returns a copy with time-based supply reduction (breaker protection). */
    public CurrentBudget withSupplyLower(double lowerLimit, double lowerTimeSeconds) {
        return new CurrentBudget(statorLimit, supplyLimit, lowerLimit, lowerTimeSeconds,
                peakForwardTorqueCurrent, peakReverseTorqueCurrent);
    }

    /** Returns a copy with explicit TorqueCurrentFOC output peaks. */
    public CurrentBudget withTorquePeaks(double peakForward, double peakReverse) {
        return new CurrentBudget(statorLimit, supplyLimit, supplyLowerLimit, supplyLowerTime,
                peakForward, peakReverse);
    }

    /** Converts to Phoenix 6 current-limit configs (all enables on). */
    public CurrentLimitsConfigs toCurrentLimitsConfigs() {
        return new CurrentLimitsConfigs()
                .withStatorCurrentLimit(statorLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(supplyLimit)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(supplyLowerLimit)
                .withSupplyCurrentLowerTime(supplyLowerTime);
    }

    /** Converts to Phoenix 6 torque-current output clamps. */
    public TorqueCurrentConfigs toTorqueCurrentConfigs() {
        return new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(peakForwardTorqueCurrent)
                .withPeakReverseTorqueCurrent(peakReverseTorqueCurrent);
    }
}
