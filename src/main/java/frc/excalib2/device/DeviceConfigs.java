package frc.excalib2.device;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Alert;

import java.util.function.Supplier;

/**
 * Config apply + read-back verify + retry for Phoenix 6 devices.
 *
 * <p>Every device configuration in {@code frc.excalib2} goes through {@link #applyVerified}:
 * <ol>
 *   <li>apply the full configuration, retrying up to {@value #MAX_ATTEMPTS} times on a
 *       non-OK status (CAN hiccups during boot are real);</li>
 *   <li>read the configuration back from the device and compare it to what was requested
 *       (serialized-token comparison with numeric tolerance);</li>
 *   <li>on failure, raise a persistent dashboard {@link Alert} and log the fault — a
 *       half-configured motor must never be silent.</li>
 * </ol>
 *
 * <p>Pattern credit: Team 254 {@code Phoenix6Util.checkErrorAndRetry} and
 * {@code TalonFXConfigEquality} (via 3061-lib), re-expressed generically.
 */
public final class DeviceConfigs {
    private static final int MAX_ATTEMPTS = 5;
    private static final double CONFIG_TIMEOUT_SECONDS = 0.1;
    private static final double NUMERIC_TOLERANCE = 1e-3;

    private DeviceConfigs() {
    }

    /** Applies and verifies a full TalonFX configuration. Returns true on verified success. */
    public static boolean applyVerified(String deviceName, TalonFX motor, TalonFXConfiguration config) {
        boolean applied = retry(deviceName, () -> motor.getConfigurator().apply(config, CONFIG_TIMEOUT_SECONDS));
        TalonFXConfiguration readBack = new TalonFXConfiguration();
        boolean refreshed = retry(deviceName, () -> motor.getConfigurator().refresh(readBack, CONFIG_TIMEOUT_SECONDS));
        boolean verified = applied && refreshed && serializedEquals(config.serialize(), readBack.serialize());
        report(deviceName, applied, verified);
        return verified;
    }

    /** Applies and verifies a full CANcoder configuration. Returns true on verified success. */
    public static boolean applyVerified(String deviceName, CANcoder encoder, CANcoderConfiguration config) {
        boolean applied = retry(deviceName, () -> encoder.getConfigurator().apply(config, CONFIG_TIMEOUT_SECONDS));
        CANcoderConfiguration readBack = new CANcoderConfiguration();
        boolean refreshed = retry(deviceName, () -> encoder.getConfigurator().refresh(readBack, CONFIG_TIMEOUT_SECONDS));
        boolean verified = applied && refreshed && serializedEquals(config.serialize(), readBack.serialize());
        report(deviceName, applied, verified);
        return verified;
    }

    /** Invokes a Phoenix call until OK or attempts are exhausted. */
    public static boolean retry(String deviceName, Supplier<StatusCode> call) {
        StatusCode code = StatusCode.OK;
        for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
            code = call.get();
            if (code.isOK()) {
                return true;
            }
        }
        DogLog.logFault("ConfigApplyFailed/" + deviceName + "/" + code.getName());
        return false;
    }

    private static void report(String deviceName, boolean applied, boolean verified) {
        if (!verified) {
            new Alert(
                    "Device config " + (applied ? "verify" : "apply") + " FAILED: " + deviceName,
                    Alert.AlertType.kError)
                    .set(true);
            DogLog.logFault("ConfigVerifyFailed/" + deviceName);
        }
    }

    /**
     * Compares two Phoenix serialized-config strings token by token; numeric tokens compare
     * with {@value #NUMERIC_TOLERANCE} tolerance (the device stores some values in fixed
     * point, so exact float equality would false-alarm).
     */
    static boolean serializedEquals(String expected, String actual) {
        String[] expectedTokens = expected.split("[\\s,;:=]+");
        String[] actualTokens = actual.split("[\\s,;:=]+");
        if (expectedTokens.length != actualTokens.length) {
            return false;
        }
        for (int i = 0; i < expectedTokens.length; i++) {
            String e = expectedTokens[i];
            String a = actualTokens[i];
            if (e.equals(a)) {
                continue;
            }
            try {
                if (Math.abs(Double.parseDouble(e) - Double.parseDouble(a)) <= NUMERIC_TOLERANCE) {
                    continue;
                }
            } catch (NumberFormatException ignored) {
                // fall through to failure — non-numeric mismatch
            }
            return false;
        }
        return true;
    }
}
