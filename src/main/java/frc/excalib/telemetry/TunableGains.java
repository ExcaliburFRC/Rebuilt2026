package frc.excalib.telemetry;

import frc.excalib.control.gains.Gains;

import java.util.function.Consumer;

/**
 * A {@link Gains} bundle whose every term (kP, kI, kD, kS, kV, kA, kG) is a live-tunable
 * {@link TunableNumber} under {@code /Tuning/<key>/...}. Edit them from AdvantageScope or
 * Elastic while the robot runs and re-apply to the controller with {@link #pollAndApply}.
 *
 * <p>When tuning mode is off (the competition default) every term returns its compiled-in
 * value, so the tuned gains stay the numbers of record and nothing is published.
 *
 * <h2>Usage</h2>
 * Wrap a gains constant, then poll once per loop from the subsystem's {@code periodic()}:
 * <pre>{@code
 * // field
 * private final TunableGains hoodGains = new TunableGains("Shooter/HoodGains", HOOD_GAINS);
 *
 * // periodic()
 * hoodGains.pollAndApply(hashCode(), g -> angleController.setPID(g.kp, g.ki, g.kd));
 * }</pre>
 * The callback fires once at startup (applying the defaults) and then only when a value
 * changes — so with tuning disabled it re-applies the exact compiled gains a single time.
 */
public class TunableGains {
    private final TunableNumber kp, ki, kd, ks, kv, ka, kg;

    public TunableGains(String key, Gains gains) {
        kp = new TunableNumber(key + "/kP", gains.kp);
        ki = new TunableNumber(key + "/kI", gains.ki);
        kd = new TunableNumber(key + "/kD", gains.kd);
        ks = new TunableNumber(key + "/kS", gains.ks);
        kv = new TunableNumber(key + "/kV", gains.kv);
        ka = new TunableNumber(key + "/kA", gains.ka);
        kg = new TunableNumber(key + "/kG", gains.kg);
    }

    /** A fresh {@link Gains} built from the current tunable values. */
    public Gains get() {
        return new Gains(kp.get(), ki.get(), kd.get(), ks.get(), kv.get(), ka.get(), kg.get());
    }

    /**
     * Runs {@code apply} with the current gains whenever any term changed (and once at startup).
     * @param callerId a stable id unique to the call site, e.g. {@code hashCode()}
     */
    public void pollAndApply(int callerId, Consumer<Gains> apply) {
        TunableNumber.ifChanged(callerId, values -> apply.accept(get()), kp, ki, kd, ks, kv, ka, kg);
    }
}
