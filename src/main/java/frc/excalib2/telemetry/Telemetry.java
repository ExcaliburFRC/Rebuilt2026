package frc.excalib2.telemetry;

import com.ctre.phoenix6.SignalLogger;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * Telemetry lifecycle for ExcaLib v2: <b>DogLog is the log of record</b> (robot logic,
 * mechanism values, state machines → WPILib DataLog + NT), and <b>Phoenix
 * {@code SignalLogger}</b> captures high-rate device signals (hoot). Both open in
 * AdvantageScope.
 *
 * <p>Call {@link #init} once from the robot constructor.
 */
public final class Telemetry {
    private Telemetry() {
    }

    /**
     * Starts both logging systems.
     *
     * @param competition at competition: no NT publishing (bandwidth), logs to file only
     * @param pdh         power distribution hub for automatic channel/voltage logging (nullable)
     */
    public static void init(boolean competition, PowerDistribution pdh) {
        DogLog.setOptions(new DogLogOptions()
                .withNtPublish(!competition)
                .withCaptureDs(true));
        if (pdh != null) {
            DogLog.setPdh(pdh);
        }
        SignalLogger.start();
    }
}
