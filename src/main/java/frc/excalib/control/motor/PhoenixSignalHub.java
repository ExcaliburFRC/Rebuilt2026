package frc.excalib.control.motor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;

import java.util.ArrayList;
import java.util.List;

/**
 * Central Phoenix 6 CAN bus-utilization optimizer.
 *
 * <p>Phoenix 6 devices broadcast a large set of status signals by default; most are unused
 * and just consume CAN bandwidth. {@link ParentDevice#optimizeBusUtilizationForAll} disables
 * every signal whose update frequency was <b>not explicitly set</b>, keeping only the ones a
 * device actually needs — a large reduction in bus load, especially with many motors.
 *
 * <p><b>Contract (order matters):</b>
 * <ol>
 *   <li>during construction each device sets the update frequency of the signals it reads
 *       ({@code TalonFXMotor} does this for position/velocity/current/voltage/temperature,
 *       and its faults; {@link #registerCANcoder} does it for the encoder position signals);</li>
 *   <li>the device registers here;</li>
 *   <li>after <b>all</b> devices exist, call {@link #optimizeAll()} exactly once — every
 *       signal not explicitly configured above is then disabled.</li>
 * </ol>
 *
 * <p>A signal read every loop but never given an update frequency will be silently disabled
 * by the optimize pass, so only register a device once its used signals are configured.
 *
 * <p>The Pigeon IMU is intentionally left unregistered: it reads yaw, tilt, acceleration and
 * angular-velocity signals across the codebase, so blanket optimization would risk disabling
 * one. Register it explicitly with {@link #register} only after setting every used signal.
 *
 * <p>Batched per-loop refresh of the registered signals is handled separately by
 * {@code TalonFXMotor.refreshAll()}.
 */
public final class PhoenixSignalHub {
    private static final List<ParentDevice> DEVICES = new ArrayList<>();

    private PhoenixSignalHub() {
    }

    /** Registers any Phoenix 6 device for the optimize pass. Used signals must already be configured. */
    public static void register(ParentDevice device) {
        DEVICES.add(device);
    }

    /**
     * Sets the update frequency of a CANcoder's absolute and relative position signals (both
     * are used in this robot) and registers it for optimization.
     *
     * @param positionHz update rate for the position signals (100 Hz matches the motor rate)
     */
    public static void registerCANcoder(CANcoder encoder, double positionHz) {
        BaseStatusSignal.setUpdateFrequencyForAll(positionHz,
                encoder.getAbsolutePosition(), encoder.getPosition());
        register(encoder);
    }

    /** Disables all unregistered signals on every registered device. Call once after robot construction. */
    public static void optimizeAll() {
        if (!DEVICES.isEmpty()) {
            ParentDevice.optimizeBusUtilizationForAll(DEVICES.toArray(new ParentDevice[0]));
        }
    }

    /**
     * Latency-compensated read: {@code value} extrapolated by {@code slope} over the signal's
     * latency (e.g. position compensated by velocity). Both signals must be freshly refreshed.
     */
    public static double latencyCompensated(StatusSignal<?> value, StatusSignal<?> slope) {
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(value, slope);
    }
}
