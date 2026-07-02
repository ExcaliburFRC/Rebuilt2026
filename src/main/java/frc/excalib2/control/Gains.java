package frc.excalib2.control;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

/**
 * Immutable closed-loop gain set for one control slot.
 *
 * <p>Units depend on the {@link ControlMode} the mechanism runs in: volts-per-unit for
 * {@link ControlMode#VOLTAGE}, amps-per-unit for {@link ControlMode#TORQUE_CURRENT_FOC}.
 * Gains therefore do <b>not</b> transfer between modes — keep separate sets.
 *
 * <p>Every {@code frc.excalib2} mechanism config carries two of these: one for the real
 * robot and one for simulation (sim physics rarely matches real friction/inertia).
 */
public record Gains(
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kA,
        double kG,
        GravityTypeValue gravityType,
        StaticFeedforwardSignValue staticFeedforwardSign) {

    /** All-zero gains. */
    public static Gains empty() {
        return pid(0, 0, 0);
    }

    /** Feedback-only gains. */
    public static Gains pid(double kP, double kI, double kD) {
        return new Gains(kP, kI, kD, 0, 0, 0, 0,
                GravityTypeValue.Elevator_Static, StaticFeedforwardSignValue.UseVelocitySign);
    }

    /** Returns a copy with static/velocity/acceleration feedforward. */
    public Gains withSVA(double kS, double kV, double kA) {
        return new Gains(kP, kI, kD, kS, kV, kA, kG, gravityType, staticFeedforwardSign);
    }

    /**
     * Returns a copy with gravity feedforward.
     *
     * @param kG          gravity feedforward (volts or amps, by control mode)
     * @param gravityType {@link GravityTypeValue#Arm_Cosine} for pivots/arms (kG scaled by
     *                    cos(angle), requires mechanism zero = horizontal),
     *                    {@link GravityTypeValue#Elevator_Static} for constant-gravity loads
     */
    public Gains withGravity(double kG, GravityTypeValue gravityType) {
        return new Gains(kP, kI, kD, kS, kV, kA, kG, gravityType, staticFeedforwardSign);
    }

    /** Returns a copy with an explicit static-feedforward sign source. */
    public Gains withStaticSign(StaticFeedforwardSignValue sign) {
        return new Gains(kP, kI, kD, kS, kV, kA, kG, gravityType, sign);
    }

    /** Converts to Phoenix 6 slot-0 configs. */
    public Slot0Configs toSlot0Configs() {
        return new Slot0Configs()
                .withKP(kP).withKI(kI).withKD(kD)
                .withKS(kS).withKV(kV).withKA(kA).withKG(kG)
                .withGravityType(gravityType)
                .withStaticFeedforwardSign(staticFeedforwardSign);
    }
}
