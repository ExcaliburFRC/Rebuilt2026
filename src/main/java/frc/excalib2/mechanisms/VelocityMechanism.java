package frc.excalib2.mechanisms;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.RotationsPerSecond;

/**
 * Velocity archetype: flywheel / shooter wheel / drum.
 *
 * <p>Setpoints run through onboard {@code MotionMagicVelocity} — the acceleration/jerk
 * limits in {@code MechanismConfig.motion(...)} shape the spin-up ramp, and the slot gains
 * ({@code kS/kV/kA} + PID) close the loop at 1 kHz on the controller.
 */
public class VelocityMechanism extends Mechanism {
    private double goalRotationsPerSecond;
    private boolean hasGoal = false;

    /** At-speed: has an active setpoint and velocity is within the configured tolerance. */
    public final Trigger atSpeed = new Trigger(
            () -> hasGoal
                    && Math.abs(motor.velocityRotationsPerSecond() - goalRotationsPerSecond)
                    <= config.toleranceRotations);

    public VelocityMechanism(MechanismConfig config) {
        super(config);
    }

    public void setVelocity(AngularVelocity velocity) {
        setVelocityRotationsPerSecond(velocity.in(RotationsPerSecond));
    }

    /** Commands a profiled velocity setpoint (mechanism rotations per second). */
    public void setVelocityRotationsPerSecond(double rotationsPerSecond) {
        goalRotationsPerSecond = rotationsPerSecond;
        hasGoal = true;
        motor.setVelocityGoal(rotationsPerSecond);
    }

    /** Stops (neutral output) and clears the setpoint ({@link #atSpeed} goes false). */
    @Override
    public void stop() {
        hasGoal = false;
        super.stop();
    }

    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(motor.velocityRotationsPerSecond());
    }

    public double getVelocityRotationsPerSecond() {
        return motor.velocityRotationsPerSecond();
    }

    public double getGoalRotationsPerSecond() {
        return goalRotationsPerSecond;
    }

    @Override
    protected void logExtras() {
        DogLog.log(config.name + "/GoalRPS", goalRotationsPerSecond);
        DogLog.log(config.name + "/HasGoal", hasGoal);
        DogLog.log(config.name + "/AtSpeed", atSpeed.getAsBoolean());
    }
}
