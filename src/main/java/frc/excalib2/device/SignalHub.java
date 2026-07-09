package frc.excalib2.device;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Central StatusSignal registry: deliberate update frequencies, one batched
 * {@code refreshAll} per CAN bus per loop, and bus-utilization optimization.
 *
 * <p>Lifecycle:
 * <ol>
 *   <li>during construction, every device registers its signals with a {@link Rate} tier;</li>
 *   <li>after all devices exist, {@link #optimizeAll()} disables every unregistered signal;</li>
 *   <li>{@link #refreshAll()} runs once per loop <b>before</b> {@code CommandScheduler.run()},
 *       so commands act on fresh data.</li>
 * </ol>
 *
 * <p>The CTRE swerve drivetrain manages its own 250 Hz time-synced signals internally and
 * does not go through this hub.
 */
public final class SignalHub {

    /** Update-frequency tiers. */
    public enum Rate {
        /** 100 Hz — position, velocity, closed-loop reference. */
        FAST(100.0),
        /** 50 Hz — currents, voltage. */
        MEDIUM(50.0),
        /** 4 Hz — temperature, housekeeping. */
        SLOW(4.0);

        public final double hertz;

        Rate(double hertz) {
            this.hertz = hertz;
        }
    }

    private static final Map<String, List<BaseStatusSignal>> SIGNALS_BY_BUS = new HashMap<>();
    private static final Map<String, BaseStatusSignal[]> CACHE_BY_BUS = new HashMap<>();
    private static final List<ParentDevice> DEVICES = new ArrayList<>();

    private SignalHub() {
    }

    /** Registers signals at the given rate tier and includes them in the per-loop batch refresh. */
    public static void register(String busName, Rate rate, BaseStatusSignal... signals) {
        BaseStatusSignal.setUpdateFrequencyForAll(rate.hertz, signals);
        List<BaseStatusSignal> list = SIGNALS_BY_BUS.computeIfAbsent(busName, k -> new ArrayList<>());
        for (BaseStatusSignal signal : signals) {
            list.add(signal);
        }
        CACHE_BY_BUS.put(busName, list.toArray(new BaseStatusSignal[0]));
    }

    /**
     * Sets signals' update rate without adding them to the batch refresh — for signals a
     * device reads on demand (e.g. faults scanned at 1 Hz by the fault reporter).
     */
    public static void setRateOnly(Rate rate, BaseStatusSignal... signals) {
        BaseStatusSignal.setUpdateFrequencyForAll(rate.hertz, signals);
    }

    /** Registers a device for {@link #optimizeAll()}. */
    public static void registerDevice(ParentDevice device) {
        DEVICES.add(device);
    }

    /** Batch-refreshes all registered signals, one CAN frame read per bus. Call once per loop. */
    public static void refreshAll() {
        for (BaseStatusSignal[] signals : CACHE_BY_BUS.values()) {
            if (signals.length > 0) {
                BaseStatusSignal.refreshAll(signals);
            }
        }
    }

    /** Disables all unregistered signals on all registered devices. Call once after robot construction. */
    public static void optimizeAll() {
        if (!DEVICES.isEmpty()) {
            ParentDevice.optimizeBusUtilizationForAll(DEVICES.toArray(new ParentDevice[0]));
        }
    }

    /**
     * Latency-compensated read: value extrapolated by its slope over the signal's latency
     * (e.g. position compensated by velocity).
     */
    public static double latencyCompensated(StatusSignal<?> value, StatusSignal<?> slope) {
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(value, slope);
    }
}
