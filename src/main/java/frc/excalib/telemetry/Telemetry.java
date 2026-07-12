package frc.excalib.telemetry;

import com.ctre.phoenix6.SignalLogger;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * Telemetry lifecycle for the Excalibur library.
 *
 * <p><b>DogLog</b> is the log of record for robot logic, mechanism values and state
 * machines (WPILib DataLog + NetworkTables). <b>Phoenix {@code SignalLogger}</b> captures
 * high-rate device signals to a {@code .hoot} file. Both open in AdvantageScope.
 *
 * <p>Monologue (the existing {@code @Log.NT} subsystem telemetry) is left untouched and
 * runs alongside this — DogLog only carries the library-level telemetry added by
 * {@link FaultReporter}, {@link LoopTimer}, the state machines and the vision client.
 *
 * <p>Call {@link #init} once from the {@code Robot} constructor.
 */
public final class Telemetry {
    private static PowerDistribution powerDistribution;

    private Telemetry() {
    }

    /**
     * Starts both logging systems and enables automatic power-distribution logging.
     *
     * @param competition at competition, NetworkTables publishing is disabled (bandwidth);
     *                    logs still go to file
     * @param pdh         the power distribution hub whose channel currents and rail voltage
     *                    are logged every loop; {@code null} to disable
     */
    public static void init(boolean competition, PowerDistribution pdh) {
        DogLog.setOptions(new DogLogOptions()
                .withNtPublish(!competition)
                .withCaptureDs(true));
        powerDistribution = pdh;
        if (pdh != null) {
            DogLog.setPdh(pdh);
        }
        SignalLogger.start();
    }

    /**
     * Logs total current, voltage, temperature and per-channel current from the power
     * distribution hub. {@link DogLog#setPdh} already captures most of this automatically;
     * this adds the explicit robot-level totals under a stable key. Call once per loop.
     */
    public static void logPowerDistribution() {
        if (powerDistribution == null) {
            return;
        }
        DogLog.log("Power/TotalCurrentAmps", powerDistribution.getTotalCurrent());
        DogLog.log("Power/VoltageVolts", powerDistribution.getVoltage());
        DogLog.log("Power/TemperatureCelsius", powerDistribution.getTemperature());
        DogLog.log("Power/TotalPowerWatts", powerDistribution.getTotalPower());
        DogLog.log("Power/TotalEnergyJoules", powerDistribution.getTotalEnergy());
    }
}
