package frc.excalib2.mechanisms;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Meters;

/**
 * Linear extension archetype: elevator stage / telescoping arm / linear slide.
 *
 * <p>Like {@link PositionalMechanism} but the mechanism unit is <b>meters</b> (configure the
 * device with a rotor-rotations-per-meter ratio via {@code rotorToMechanismRatio}, and use
 * the {@code *Meters} config helpers for limits/tolerance). Goals run through onboard
 * MotionMagic (Expo by default).
 *
 * <p>Two gravity modes (port of ExcaLib v1 {@code LinearExtension}, which was always the
 * arm-mounted case):
 * <ul>
 *   <li><b>Constant gravity</b> — use {@link #LinearExtension(MechanismConfig)} and put a
 *       {@code kG} with {@code GravityType.Elevator_Static} in the config gains; the
 *       controller holds the load onboard. Right for a vertical elevator.</li>
 *   <li><b>Dynamic gravity</b> — use
 *       {@link #LinearExtension(MechanismConfig, DoubleSupplier, double)} for an extension
 *       mounted on a pivoting arm, where the gravity component along the slide axis varies
 *       with the arm angle. Applies {@code gravityFeedforward * sin(angle)} as a supplemental
 *       feedforward on top of MotionMagic each command (volts in VOLTAGE mode, amps in
 *       TorqueCurrentFOC). Keep the config gains' {@code kG} at 0 in this mode.</li>
 * </ul>
 */
public class LinearExtension extends Mechanism {
    private final DoubleSupplier gravityAngleRadians; // null → constant-gravity mode
    private final double gravityFeedforward;

    private double goalMeters;
    private boolean hasGoal = false;

    /** At-goal: has an active goal and position is within the configured tolerance (meters). */
    public final Trigger atGoal = new Trigger(
            () -> hasGoal && Math.abs(motor.positionRotations() - goalMeters) <= config.toleranceRotations);

    /** Constant-gravity elevator (onboard kG via the config gains). */
    public LinearExtension(MechanismConfig config) {
        this(config, null, 0);
    }

    /**
     * Arm-mounted extension with dynamic gravity feedforward.
     *
     * @param gravityAngleRadians the mounting arm's angle from horizontal (radians); the
     *                            gravity component along the slide is {@code sin(angle)}
     * @param gravityFeedforward  the gravity coefficient in the control mode's units (volts in
     *                            VOLTAGE mode, amps in TorqueCurrentFOC) — output needed to hold
     *                            the load when fully vertical
     */
    public LinearExtension(MechanismConfig config, DoubleSupplier gravityAngleRadians, double gravityFeedforward) {
        super(config);
        this.gravityAngleRadians = gravityAngleRadians;
        this.gravityFeedforward = gravityFeedforward;
    }

    public void setLength(Distance length) {
        setLengthMeters(length.in(Meters));
    }

    /** Commands a profiled move to {@code meters}, clamped to the soft limits, with gravity FF. */
    public void setLengthMeters(double meters) {
        goalMeters = clampToSoftLimits(meters);
        hasGoal = true;
        double gravityFF = gravityAngleRadians == null
                ? 0
                : gravityFeedforward * Math.sin(gravityAngleRadians.getAsDouble());
        if (config.motion.useExpo()) {
            motor.setExpoPositionGoal(goalMeters, gravityFF);
        } else {
            motor.setTrapezoidPositionGoal(goalMeters, gravityFF);
        }
    }

    /** Stops and clears the goal ({@link #atGoal} goes false). */
    @Override
    public void stop() {
        hasGoal = false;
        super.stop();
    }

    public Distance getLength() {
        return Meters.of(motor.positionRotations());
    }

    public double getLengthMeters() {
        return motor.positionRotations();
    }

    public double getVelocityMetersPerSecond() {
        return motor.velocityRotationsPerSecond();
    }

    public double getGoalMeters() {
        return goalMeters;
    }

    /** Overwrites the mechanism position (homing). */
    public void resetLength(Distance length) {
        motor.resetPosition(length.in(Meters));
    }

    private double clampToSoftLimits(double meters) {
        double result = meters;
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
        DogLog.log(config.name + "/GoalMeters", goalMeters);
        DogLog.log(config.name + "/HasGoal", hasGoal);
        DogLog.log(config.name + "/AtGoal", atGoal.getAsBoolean());
    }
}
