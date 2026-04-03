package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.excalib.additional_utilities.CANHealthMonitor;
import frc.excalib.additional_utilities.PerformanceMetricsTracker;
import frc.robot.Constants;
import monologue.Annotations.Log;
import monologue.Logged;

/**
 * Manages robot-wide telemetry, diagnostics, and performance metrics.
 * Consolidation here keeps RobotContainer and Robot lean.
 */
public class TelemetryManager implements Logged {
    private static final double VOLTAGE_WARNING = 12.0;
    private static final double VOLTAGE_HYSTERESIS = 0.5;

    private final PowerDistribution pdh = new PowerDistribution(Constants.PDH_PORT, PowerDistribution.ModuleType.kRev);
    private final CANHealthMonitor canMonitor = new CANHealthMonitor();
    private final PerformanceMetricsTracker metricsTracker = new PerformanceMetricsTracker();
    
    private final Alert lowBatteryAlert = new Alert("Battery voltage is low", Alert.AlertType.kWarning);
    private final Alert primaryDisconnected = new Alert("Primary controller disconnected", Alert.AlertType.kWarning);
    
    private boolean batteryLow = false;

    public void update() {
        // Core diagnostic updates
        canMonitor.update();
        metricsTracker.recordPowerConsumption(pdh.getTotalPower());
        
        // Battery monitoring
        double voltage = pdh.getVoltage();
        if (voltage < VOLTAGE_WARNING - VOLTAGE_HYSTERESIS) {
            batteryLow = true;
        } else if (voltage > VOLTAGE_WARNING + VOLTAGE_HYSTERESIS) {
            batteryLow = false;
        }
        lowBatteryAlert.set(batteryLow);
        
        // Connectivity checks
        primaryDisconnected.set(!DriverStation.isJoystickConnected(Constants.PRIMARY_CONTROLLER_PORT));
    }

    public void recordLoopStart() {
        metricsTracker.recordLoopStart();
    }

    public void recordLoopEnd() {
        metricsTracker.recordLoopEnd();
    }

    public PerformanceMetricsTracker getMetricsTracker() {
        return metricsTracker;
    }

    @Log.NT
    public double getBatteryVoltage() {
        return pdh.getVoltage();
    }

    @Log.NT
    public boolean isBatteryLow() {
        return batteryLow;
    }

    @Log.NT
    public boolean isCanHealthy() {
        return canMonitor.isHealthy();
    }
}
