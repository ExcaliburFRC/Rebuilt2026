package frc.excalib2.mechanisms;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Roller archetype: intake / indexer / feeder rollers.
 *
 * <p>Open-loop by intent (rollers rarely need closed loop): duty-cycle, voltage, or
 * torque-current output, plus <b>current-spike game-piece detection</b> — a debounced
 * trigger on stator current (pattern credit: 254 {@code CurrentSpikeDetector},
 * 6328 RollerSystem).
 */
public class RollerMechanism extends Mechanism {
    private double outputVolts;

    public RollerMechanism(MechanismConfig config) {
        super(config);
    }

    public void runVolts(double volts) {
        outputVolts = volts;
        motor.setVolts(volts);
    }

    /** Torque-current output (FOC only; prefer {@link #runVolts} in sim). */
    public void runAmps(double amps) {
        motor.setAmps(amps);
    }

    public void runDutyCycle(double dutyCycle) {
        motor.setDutyCycle(dutyCycle);
    }

    @Override
    public void stop() {
        outputVolts = 0;
        super.stop();
    }

    public double getStatorAmps() {
        return motor.statorAmps();
    }

    public double getVelocityRotationsPerSecond() {
        return motor.velocityRotationsPerSecond();
    }

    /**
     * Builds a game-piece / stall detector: true once stator current stays above
     * {@code thresholdAmps} for {@code debounceSeconds}.
     */
    public Trigger currentSpike(double thresholdAmps, double debounceSeconds) {
        Debouncer debouncer = new Debouncer(debounceSeconds, Debouncer.DebounceType.kRising);
        return new Trigger(() -> debouncer.calculate(motor.statorAmps() > thresholdAmps));
    }

    @Override
    protected void logExtras() {
        DogLog.log(config.name + "/OutputVolts", outputVolts);
    }
}
