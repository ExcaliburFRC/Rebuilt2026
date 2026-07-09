package frc.excalib2.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib2.statemachine.StateMachine;

/**
 * Base for the top-level robot-goal coordinator.
 *
 * <p>A concrete superstructure declares a goal enum (each goal = a value-tuple of
 * per-mechanism setpoints/states — 1678 {@code SuperstructureGoal} pattern), builds a
 * {@link StateMachine} over it whose {@code onEnter} actions fan the goal out to the
 * subsystem state machines, and encodes <b>interlocks as transition guards</b>. Illegal
 * mechanism combinations are unrepresentable (no enum entry) or unreachable (no edge).
 *
 * <p>Driver/operator/zone {@link Trigger}s request goals; autonomous named commands call
 * the same {@link #request} path, so teleop and auto behavior cannot diverge.
 */
public abstract class Superstructure<G extends Enum<G>> extends SubsystemBase {
    protected final StateMachine<G> machine;

    protected Superstructure(StateMachine<G> machine) {
        this.machine = machine;
    }

    /** Instant goal request (pending if guard-blocked, dropped if illegal). */
    public Command request(G goal) {
        return machine.requestCommand(goal);
    }

    /** Goal request that finishes once the goal is active. */
    public Command requestAndWait(G goal) {
        return machine.requestAndWaitCommand(goal);
    }

    public G currentGoal() {
        return machine.getCurrentState();
    }

    /** Trigger: the superstructure is currently pursuing {@code goal}. */
    public Trigger in(G goal) {
        return machine.in(goal);
    }

    @Override
    public void periodic() {
        machine.periodic();
    }
}
