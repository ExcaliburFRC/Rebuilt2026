package frc.excalib.telemetry;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/**
 * A live-tunable number published under {@code /Tuning/<key>} — edit it from AdvantageScope
 * or Elastic while the robot runs. When tuning mode is off (the competition default) the
 * compiled-in default is always returned and nothing is published, so tuned constants stay
 * the numbers of record.
 *
 * <p>{@link #ifChanged} runs a callback when any of a group of tunables changes — the hook
 * for re-applying gains to a device without a redeploy.
 *
 * <p>Pattern credit: 6328 {@code LoggedTunableNumber}, re-expressed over plain NetworkTables.
 */
public class TunableNumber implements DoubleSupplier {
    private static boolean tuningEnabled = false;

    private final double defaultValue;
    private final DoubleEntry entry;
    private final Map<Integer, Double> lastValues = new HashMap<>();

    /** Enables tuning mode. Call once at startup in dev builds; never at competition. */
    public static void enableTuning() {
        tuningEnabled = true;
    }

    public static boolean isTuningEnabled() {
        return tuningEnabled;
    }

    public TunableNumber(String key, double defaultValue) {
        this.defaultValue = defaultValue;
        if (tuningEnabled) {
            entry = NetworkTableInstance.getDefault()
                    .getDoubleTopic("/Tuning/" + key)
                    .getEntry(defaultValue);
            entry.set(defaultValue);
        } else {
            entry = null;
        }
    }

    public double get() {
        return entry == null ? defaultValue : entry.get(defaultValue);
    }

    @Override
    public double getAsDouble() {
        return get();
    }

    /**
     * True once per change, per caller id. Typical use:
     * {@code if (kP.hasChanged(hashCode())) applyGains();}
     */
    public boolean hasChanged(int callerId) {
        double value = get();
        Double last = lastValues.get(callerId);
        if (last == null || last != value) {
            lastValues.put(callerId, value);
            return true;
        }
        return false;
    }

    /** Runs {@code action} with current values whenever any of {@code tunables} changed. */
    public static void ifChanged(int callerId, Consumer<double[]> action, TunableNumber... tunables) {
        boolean changed = false;
        for (TunableNumber tunable : tunables) {
            changed |= tunable.hasChanged(callerId);
        }
        if (changed) {
            double[] values = new double[tunables.length];
            for (int i = 0; i < tunables.length; i++) {
                values[i] = tunables[i].get();
            }
            action.accept(values);
        }
    }
}
