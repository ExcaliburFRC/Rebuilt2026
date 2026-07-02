package frc.excalib2.statemachine;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.*;

class StateMachineTest {
    private enum TestState {
        IDLE, SPINNING, SHOOTING
    }

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        CommandScheduler.getInstance().cancelAll();
    }

    @AfterEach
    void teardown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    @Test
    void initialStateEntersOnFirstTick() {
        AtomicInteger entered = new AtomicInteger();
        StateMachine<TestState> machine = new StateMachine<>("test", TestState.IDLE)
                .onEnter(TestState.IDLE, entered::incrementAndGet);

        assertEquals(0, entered.get()); // declarations first, actions on first tick
        machine.periodic();
        assertEquals(TestState.IDLE, machine.getCurrentState());
        assertEquals(1, entered.get());
        machine.periodic();
        assertEquals(1, entered.get()); // start fires exactly once
    }

    @Test
    void legalTransitionRunsExitThenEnter() {
        List<String> order = new ArrayList<>();
        StateMachine<TestState> machine = new StateMachine<>("test", TestState.IDLE)
                .onExit(TestState.IDLE, () -> order.add("exit-idle"))
                .onEnter(TestState.SPINNING, () -> order.add("enter-spinning"))
                .transition(TestState.IDLE, TestState.SPINNING);

        assertTrue(machine.request(TestState.SPINNING));
        assertEquals(TestState.SPINNING, machine.getCurrentState());
        assertEquals(List.of("exit-idle", "enter-spinning"), order);
    }

    @Test
    void undeclaredTransitionIsDenied() {
        StateMachine<TestState> machine = new StateMachine<>("test", TestState.IDLE)
                .transition(TestState.IDLE, TestState.SPINNING);

        assertFalse(machine.request(TestState.SHOOTING));
        assertEquals(TestState.IDLE, machine.getCurrentState());
        assertNull(machine.getPending());
    }

    @Test
    void guardedTransitionStaysPendingUntilGuardPasses() {
        AtomicBoolean guard = new AtomicBoolean(false);
        StateMachine<TestState> machine = new StateMachine<>("test", TestState.IDLE)
                .transition(TestState.IDLE, TestState.SPINNING, guard::get);

        assertFalse(machine.request(TestState.SPINNING));
        assertEquals(TestState.IDLE, machine.getCurrentState());
        assertEquals(TestState.SPINNING, machine.getPending());

        machine.periodic();
        assertEquals(TestState.IDLE, machine.getCurrentState());

        guard.set(true);
        machine.periodic();
        assertEquals(TestState.SPINNING, machine.getCurrentState());
        assertNull(machine.getPending());
    }

    @Test
    void fromAnyEdgeWorksFromEveryState() {
        StateMachine<TestState> machine = new StateMachine<>("test", TestState.IDLE)
                .transition(TestState.IDLE, TestState.SPINNING)
                .transition(TestState.SPINNING, TestState.SHOOTING)
                .transitionFromAny(TestState.IDLE);

        machine.request(TestState.SPINNING);
        machine.request(TestState.SHOOTING);
        assertEquals(TestState.SHOOTING, machine.getCurrentState());
        assertTrue(machine.request(TestState.IDLE));
        assertEquals(TestState.IDLE, machine.getCurrentState());
    }

    @Test
    void requestingCurrentStateSucceedsAndClearsPending() {
        AtomicBoolean guard = new AtomicBoolean(false);
        StateMachine<TestState> machine = new StateMachine<>("test", TestState.IDLE)
                .transition(TestState.IDLE, TestState.SPINNING, guard::get);

        machine.request(TestState.SPINNING);
        assertEquals(TestState.SPINNING, machine.getPending());
        assertTrue(machine.request(TestState.IDLE));
        assertNull(machine.getPending());
    }

    @Test
    void whileInCommandIsScheduledOnEntryAndCancelledOnExit() {
        AtomicInteger executes = new AtomicInteger();
        AtomicBoolean cancelled = new AtomicBoolean(false);
        Command spinCommand = Commands.run(executes::incrementAndGet)
                .finallyDo(interrupted -> cancelled.set(interrupted));

        StateMachine<TestState> machine = new StateMachine<>("test", TestState.IDLE)
                .whileIn(TestState.SPINNING, spinCommand)
                .transition(TestState.IDLE, TestState.SPINNING)
                .transition(TestState.SPINNING, TestState.IDLE);

        machine.request(TestState.SPINNING);
        CommandScheduler.getInstance().run();
        assertTrue(executes.get() > 0);
        assertTrue(spinCommand.isScheduled());

        machine.request(TestState.IDLE);
        assertFalse(spinCommand.isScheduled());
        assertTrue(cancelled.get());
    }

    @Test
    void in_triggerTracksState() {
        StateMachine<TestState> machine = new StateMachine<>("test", TestState.IDLE)
                .transition(TestState.IDLE, TestState.SPINNING);

        assertTrue(machine.in(TestState.IDLE).getAsBoolean());
        machine.request(TestState.SPINNING);
        assertTrue(machine.in(TestState.SPINNING).getAsBoolean());
        assertFalse(machine.in(TestState.IDLE).getAsBoolean());
    }

    @Test
    void forceStateBypassesTable() {
        StateMachine<TestState> machine = new StateMachine<>("test", TestState.IDLE);
        machine.forceState(TestState.SHOOTING);
        assertEquals(TestState.SHOOTING, machine.getCurrentState());
    }
}
