package frc.excalib2.telemetry;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;

/**
 * Main-loop timing telemetry (port of ExcaLib v1's {@code PerformanceMetricsTracker},
 * logging to DogLog instead of printing summaries; power logging comes from
 * {@code DogLog.setPdh}).
 *
 * <p>Call {@link #start()} at the top of {@code robotPeriodic} and {@link #end()} at the
 * bottom. Logged: current/max loop time and a 20 ms overrun counter — watch them in
 * AdvantageScope.
 */
public final class LoopTimer {
    private static final double OVERRUN_THRESHOLD_MS = 20.0;

    private static double loopStartSeconds;
    private static double maxLoopMs = 0;
    private static int overrunCount = 0;

    private LoopTimer() {
    }

    public static void start() {
        loopStartSeconds = Timer.getFPGATimestamp();
    }

    public static void end() {
        double loopMs = (Timer.getFPGATimestamp() - loopStartSeconds) * 1000.0;
        maxLoopMs = Math.max(maxLoopMs, loopMs);
        if (loopMs > OVERRUN_THRESHOLD_MS) {
            overrunCount++;
        }
        DogLog.log("Robot/LoopMs", loopMs);
        DogLog.log("Robot/MaxLoopMs", maxLoopMs);
        DogLog.log("Robot/LoopOverruns", overrunCount);
    }

    /** Clears the max/overrun statistics (e.g. at mode changes). */
    public static void reset() {
        maxLoopMs = 0;
        overrunCount = 0;
    }
}
