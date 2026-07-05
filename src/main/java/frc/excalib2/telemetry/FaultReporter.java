package frc.excalib2.telemetry;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import dev.doglog.DogLog;

import java.util.ArrayList;
import java.util.List;

/**
 * Periodic device-fault scanner: registered devices' key faults surface as dashboard
 * {@link Alert}s and DogLog faults. Rate-limited internally to 1 Hz — call {@link #poll()}
 * every loop from {@code robotPeriodic}.
 *
 * <p>Pattern credit: 3061-lib {@code FaultReporter}/self-check, simplified.
 */
public final class FaultReporter {
    private static final double SCAN_PERIOD_SECONDS = 1.0;
    private static final List<MonitoredDevice> DEVICES = new ArrayList<>();
    private static double lastScanTimestamp = 0;

    private FaultReporter() {
    }

    private static final class MonitoredDevice {
        final String name;
        final StatusSignal<Boolean> hardwareFault;
        final StatusSignal<Boolean> deviceTempFault;
        final StatusSignal<Boolean> bootDuringEnable;
        final StatusSignal<Boolean> undervoltage;
        final Alert hardwareAlert;
        final Alert tempAlert;
        final Alert bootAlert;
        final Alert undervoltageAlert;

        MonitoredDevice(String name, TalonFX talon) {
            this.name = name;
            hardwareFault = talon.getFault_Hardware();
            deviceTempFault = talon.getFault_DeviceTemp();
            bootDuringEnable = talon.getStickyFault_BootDuringEnable();
            undervoltage = talon.getStickyFault_Undervoltage();
            hardwareAlert = new Alert(name + ": hardware fault", Alert.AlertType.kError);
            tempAlert = new Alert(name + ": overtemperature", Alert.AlertType.kWarning);
            bootAlert = new Alert(name + ": boot during enable (sticky)", Alert.AlertType.kWarning);
            undervoltageAlert = new Alert(name + ": undervoltage (sticky)", Alert.AlertType.kWarning);
        }

        void scan() {
            hardwareFault.refresh();
            deviceTempFault.refresh();
            bootDuringEnable.refresh();
            undervoltage.refresh();
            update(hardwareAlert, hardwareFault, "HardwareFault");
            update(tempAlert, deviceTempFault, "DeviceTempFault");
            update(bootAlert, bootDuringEnable, "BootDuringEnable");
            update(undervoltageAlert, undervoltage, "Undervoltage");
        }

        private void update(Alert alert, StatusSignal<Boolean> fault, String faultName) {
            boolean active = fault.getStatus().isOK() && fault.getValue();
            alert.set(active);
            if (active) {
                DogLog.logFault(name + "/" + faultName);
            }
        }
    }

    /** Registers a TalonFX for fault scanning. Motor does this automatically. */
    public static void register(String name, TalonFX talon) {
        DEVICES.add(new MonitoredDevice(name, talon));
    }

    private static final Alert CAN_UTILIZATION_ALERT =
            new Alert("RIO CAN bus utilization critical (>90%)", Alert.AlertType.kWarning);

    /** Scans all registered devices at most once per second. Call every loop. */
    public static void poll() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            return; // fault signals are not modeled in sim — refreshing them just spams stale-CAN warnings
        }
        double now = Timer.getFPGATimestamp();
        if (now - lastScanTimestamp < SCAN_PERIOD_SECONDS) {
            return;
        }
        lastScanTimestamp = now;
        for (MonitoredDevice device : DEVICES) {
            device.scan();
        }
        scanCanBusHealth();
    }

    /** RIO CAN bus health logging + utilization alert (v1 {@code CANHealthMonitor} equivalent). */
    private static void scanCanBusHealth() {
        var canStatus = edu.wpi.first.wpilibj.RobotController.getCANStatus();
        DogLog.log("CAN/RioUtilizationPercent", canStatus.percentBusUtilization * 100.0);
        DogLog.log("CAN/BusOffCount", canStatus.busOffCount);
        DogLog.log("CAN/TxFullCount", canStatus.txFullCount);
        DogLog.log("CAN/ReceiveErrorCount", canStatus.receiveErrorCount);
        DogLog.log("CAN/TransmitErrorCount", canStatus.transmitErrorCount);
        CAN_UTILIZATION_ALERT.set(canStatus.percentBusUtilization > 0.9);
    }
}
