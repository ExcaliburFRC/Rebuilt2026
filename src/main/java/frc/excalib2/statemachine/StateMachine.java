package frc.excalib2.statemachine;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.EnumMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * A typed, enum-based state machine with an explicit transition table, guards, and
 * enter/while/exit actions — declarative, command-friendly, and never blocking.
 *
 * <h2>Semantics</h2>
 * <ul>
 *   <li><b>Default-deny:</b> a transition not declared in the table is illegal.
 *       {@link #request} for it fails (logged); illegal sequences are uncommandable.</li>
 *   <li><b>Guards:</b> each edge may carry a {@link BooleanSupplier}. A request whose edge
 *       guard is false becomes <i>pending</i> and is retried every {@link #periodic()} until
 *       it passes or another request replaces it.</li>
 *   <li><b>Actions:</b> {@code onEnter}/{@code onExit} runnables execute synchronously at the
 *       transition; a {@code whileIn} command is scheduled on entry and cancelled on exit
 *       (its requirements integrate with command-based as usual).</li>
 *   <li><b>Triggers:</b> {@link #in(Enum)} exposes states as WPILib {@link Trigger}s for
 *       bindings.</li>
 * </ul>
 *
 * <p>Tick {@link #periodic()} from the owning subsystem. All calls are main-loop only.
 *
 * <p>Pattern credits: 254 transition-table superstructure (simplified — no path planning),
 * MA5951 per-subsystem states (concept).
 */
public final class StateMachine<S extends Enum<S>> {
    private final String name;
    private final Map<S, Map<S, BooleanSupplier>> edges;
    private final Map<S, BooleanSupplier> fromAnyEdges;
    private final Map<S, Runnable> onEnter;
    private final Map<S, Runnable> onExit;
    private final Map<S, Command> whileIn;

    private S current;
    private S pending = null;
    private String lastDenied = "";

    private StateMachine(Builder<S> builder) {
        this.name = builder.name;
        this.edges = builder.edges;
        this.fromAnyEdges = builder.fromAnyEdges;
        this.onEnter = builder.onEnter;
        this.onExit = builder.onExit;
        this.whileIn = builder.whileIn;
        this.current = builder.initial;
        enterState(builder.initial);
    }

    public static <S extends Enum<S>> Builder<S> builder(String name, S initialState) {
        return new Builder<>(name, initialState);
    }

    // ── Requests ─────────────────────────────────────────────────────────────

    /**
     * Requests a transition to {@code target}.
     *
     * @return true if the transition happened now; false if it is pending on a guard or was
     * denied (no legal edge — logged and surfaced via telemetry)
     */
    public boolean request(S target) {
        if (target == current) {
            pending = null;
            return true;
        }
        BooleanSupplier guard = guardFor(current, target);
        if (guard == null) {
            lastDenied = current + "->" + target;
            DogLog.logFault(name + "/IllegalTransition/" + lastDenied);
            pending = null;
            return false;
        }
        if (guard.getAsBoolean()) {
            transitionTo(target);
            pending = null;
            return true;
        }
        pending = target;
        return false;
    }

    /** Instant request; a guard-blocked request stays pending, an illegal one is dropped. */
    public Command requestCommand(S target) {
        return Commands.runOnce(() -> request(target)).withName(name + " -> " + target);
    }

    /** Requests {@code target} and finishes only once the machine is in it. */
    public Command requestAndWaitCommand(S target) {
        return Commands.runOnce(() -> request(target))
                .andThen(Commands.waitUntil(() -> current == target))
                .withName(name + " -> " + target + " (wait)");
    }

    /** Jumps to a state without consulting the table (init/reset/fault recovery only). */
    public void forceState(S target) {
        pending = null;
        if (target != current) {
            transitionTo(target);
        }
    }

    // ── Queries ──────────────────────────────────────────────────────────────

    public S getCurrentState() {
        return current;
    }

    /** Trigger that is true while the machine is in {@code state}. */
    public Trigger in(S state) {
        return new Trigger(() -> current == state);
    }

    /** The request currently blocked on its guard, or null. */
    public S getPending() {
        return pending;
    }

    // ── Lifecycle ────────────────────────────────────────────────────────────

    /** Retries pending requests and publishes telemetry. Call once per loop. */
    public void periodic() {
        if (pending != null) {
            S target = pending;
            BooleanSupplier guard = guardFor(current, target);
            if (guard == null) { // state changed since the request; edge may no longer exist
                lastDenied = current + "->" + target;
                pending = null;
            } else if (guard.getAsBoolean()) {
                transitionTo(target);
                pending = null;
            }
        }
        DogLog.log(name + "/State", current.name());
        DogLog.log(name + "/Pending", pending == null ? "" : pending.name());
        DogLog.log(name + "/LastDenied", lastDenied);
    }

    // ── Internals ────────────────────────────────────────────────────────────

    private BooleanSupplier guardFor(S from, S to) {
        Map<S, BooleanSupplier> fromEdges = edges.get(from);
        BooleanSupplier guard = fromEdges != null ? fromEdges.get(to) : null;
        if (guard == null) {
            guard = fromAnyEdges.get(to);
        }
        return guard;
    }

    private void transitionTo(S target) {
        exitState(current);
        current = target;
        enterState(target);
    }

    private void enterState(S state) {
        Runnable enter = onEnter.get(state);
        if (enter != null) {
            enter.run();
        }
        Command command = whileIn.get(state);
        if (command != null) {
            command.schedule();
        }
    }

    private void exitState(S state) {
        Command command = whileIn.get(state);
        if (command != null) {
            command.cancel();
        }
        Runnable exit = onExit.get(state);
        if (exit != null) {
            exit.run();
        }
    }

    // ── Builder ──────────────────────────────────────────────────────────────

    public static final class Builder<S extends Enum<S>> {
        private static final BooleanSupplier ALWAYS = () -> true;

        private final String name;
        private final S initial;
        private final Map<S, Map<S, BooleanSupplier>> edges;
        private final Map<S, BooleanSupplier> fromAnyEdges;
        private final Map<S, Runnable> onEnter;
        private final Map<S, Runnable> onExit;
        private final Map<S, Command> whileIn;

        @SuppressWarnings("unchecked")
        private Builder(String name, S initial) {
            this.name = name;
            this.initial = initial;
            Class<S> type = (Class<S>) initial.getClass();
            this.edges = new EnumMap<>(type);
            this.fromAnyEdges = new EnumMap<>(type);
            this.onEnter = new EnumMap<>(type);
            this.onExit = new EnumMap<>(type);
            this.whileIn = new EnumMap<>(type);
        }

        /** Runs synchronously whenever the machine enters {@code state}. */
        public Builder<S> onEnter(S state, Runnable action) {
            onEnter.put(state, action);
            return this;
        }

        /** Scheduled on entry, cancelled on exit. Requirements apply as usual. */
        public Builder<S> whileIn(S state, Command command) {
            whileIn.put(state, command);
            return this;
        }

        /** Runs synchronously whenever the machine leaves {@code state}. */
        public Builder<S> onExit(S state, Runnable action) {
            onExit.put(state, action);
            return this;
        }

        /** Declares an always-allowed edge. */
        public Builder<S> transition(S from, S to) {
            return transition(from, to, ALWAYS);
        }

        /** Declares a guarded edge. */
        @SuppressWarnings("unchecked")
        public Builder<S> transition(S from, S to, BooleanSupplier guard) {
            edges.computeIfAbsent(from, k -> new EnumMap<>((Class<S>) k.getClass())).put(to, guard);
            return this;
        }

        /** Declares an edge into {@code to} from every state (e.g. an IDLE escape). */
        public Builder<S> transitionFromAny(S to) {
            return transitionFromAny(to, ALWAYS);
        }

        public Builder<S> transitionFromAny(S to, BooleanSupplier guard) {
            fromAnyEdges.put(to, guard);
            return this;
        }

        public StateMachine<S> build() {
            return new StateMachine<>(this);
        }
    }
}
