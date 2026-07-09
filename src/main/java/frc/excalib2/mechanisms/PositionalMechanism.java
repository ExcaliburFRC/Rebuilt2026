package frc.excalib2.mechanisms;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Rotations;

/**
 * Positional archetype: arm / pivot / hood / turret.
 *
 * <p>Goals run through onboard MotionMagic (Expo by default — configure via
 * {@code MechanismConfig.motion(...)}) with gravity feedforward from the gains' {@code kG}
 * and {@code GravityType}. Soft limits clamp every goal; mechanisms configured with
 * {@code continuousWrap()} (turrets) get their goal wrapped to the nearest equivalent
 * angle inside the soft-limit range (shortest legal path — port of ExcaLib v1's
 * {@code ContinuousSoftLimit} idea).
 */
public class PositionalMechanism extends Mechanism {
    private double goalRotations;
    private boolean hasGoal = false;

    /** At-goal: has an active goal and position is within the configured tolerance. */
    public final Trigger atGoal = new Trigger(
            () -> hasGoal && Math.abs(motor.positionRotations() - goalRotations) <= config.toleranceRotations);

    public PositionalMechanism(MechanismConfig config) {
        super(config);
    }

    /** Commands a profiled move to {@code goal}. Call every loop or once — the profile runs onboard. */
    public void setGoal(Angle goal) {
        setGoalRotations(goal.in(Rotations));
    }

    /** {@link #setGoal(Angle)} in mechanism rotations. */
    public void setGoalRotations(double rotations) {
        double target = rotations;
        if (config.continuousWrap) {
            target = wrapToRange(motor.positionRotations(), rotations);
        }
        target = clampToSoftLimits(target);

        goalRotations = target;
        hasGoal = true;
        if (config.motion.useExpo()) {
            motor.setExpoPositionGoal(target);
        } else {
            motor.setTrapezoidPositionGoal(target);
        }
    }

    /** Stops and clears the goal ({@link #atGoal} goes false). */
    @Override
    public void stop() {
        hasGoal = false;
        super.stop();
    }

    public Angle getPosition() {
        return Rotations.of(motor.positionRotations());
    }

    public double getPositionRotations() {
        return motor.positionRotations();
    }

    public double getVelocityRotationsPerSecond() {
        return motor.velocityRotationsPerSecond();
    }

    public double getGoalRotations() {
        return goalRotations;
    }

    /** Overwrites the mechanism position (homing / seeding from an external reference). */
    public void resetPosition(Angle position) {
        motor.resetPosition(position.in(Rotations));
    }

    /**
     * Chooses the equivalent of {@code desired} (mod 1 rotation) nearest to {@code current}
     * that still lies inside the soft-limit range.
     */
    private double wrapToRange(double current, double desired) {
        double nearest = current + MathUtil.inputModulus(desired - current, -0.5, 0.5);
        double min = config.reverseSoftLimitRotations != null ? config.reverseSoftLimitRotations : Double.NEGATIVE_INFINITY;
        double max = config.forwardSoftLimitRotations != null ? config.forwardSoftLimitRotations : Double.POSITIVE_INFINITY;
        if (nearest > max) {
            nearest -= 1.0;
        } else if (nearest < min) {
            nearest += 1.0;
        }
        return nearest;
    }

    private double clampToSoftLimits(double rotations) {
        double result = rotations;
        if (config.forwardSoftLimitRotations != null) {
            result = Math.min(result, config.forwardSoftLimitRotations);
        }
        if (config.reverseSoftLimitRotations != null) {
            result = Math.max(result, config.reverseSoftLimitRotations);
        }
        return result;
    }

    @Override
    protected void logExtras() {
        DogLog.log(config.name + "/GoalRotations", goalRotations);
        DogLog.log(config.name + "/HasGoal", hasGoal);
        DogLog.log(config.name + "/AtGoal", atGoal.getAsBoolean());
    }
}
