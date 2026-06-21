package frc.robot.lib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

/**
 * Base class for subsystems driven by a finite set of states.
 * <p>
 * Encapsulates the {@code currentState} field plus the {@link #setStateCommand(Enum)} dispatch that
 * was previously copy-pasted into every subsystem. Concrete subsystems read the active state through
 * the protected {@link #state()} accessor and expose their own {@code @Log.NT} getter (so the
 * NetworkTables key name stays subsystem-specific and Monologue scanning is not relied on across the
 * inheritance boundary).
 *
 * @param <S> the enum type describing this subsystem's states
 */
public abstract class StateMachineSubsystem<S extends Enum<S>> extends SubsystemBase implements Logged {
    private S currentState;

    protected StateMachineSubsystem(S initialState) {
        this.currentState = initialState;
    }

    /** @return the currently active state. */
    protected S state() {
        return currentState;
    }

    /** @return the name of the currently active state, for telemetry. */
    public String currentStateName() {
        return currentState.name();
    }

    /**
     * @param next the state to transition into
     * @return a command that atomically sets the active state (does not require this subsystem, so it
     * composes inside higher-level {@code ParallelCommandGroup}s the way the old per-subsystem
     * {@code setStateCommand} did).
     */
    public Command setStateCommand(S next) {
        return Commands.runOnce(() -> currentState = next);
    }
}
