package frc.robot.subsystems.transport;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.excalib2.control.CurrentBudget;
import frc.excalib2.control.Gains;
import frc.excalib2.sim.MechanismSim;

public class TransportConstants {
    public static final int DRUM_MOTOR_ID = 20;
    public static final int TRANSPORT_MOTOR_ID = 33;

    /**
     * Legacy unit conversion (ExcaLib v1 velocity conversion factors): the old code
     * commanded velocities in "legacy linear units" = motor rotations × factor. State
     * setpoints and tolerances below are still authored in those units and converted, so
     * behavior is bit-identical to the pre-migration robot.
     */
    public static final double DRUM_LEGACY_UNITS_PER_ROTATION = 0.39898 / 9;
    public static final double TRANSPORT_LEGACY_UNITS_PER_ROTATION = 0.0731;

    /** Legacy-unit velocity tolerance (10 units, from v1) converted per mechanism. */
    public static final double DRUM_TOLERANCE_RPS = 10 / DRUM_LEGACY_UNITS_PER_ROTATION;
    public static final double TRANSPORT_TOLERANCE_RPS = 10 / TRANSPORT_LEGACY_UNITS_PER_ROTATION;

    /**
     * Ported v1 gains (volts-based, converted from legacy units to mechanism rotations):
     * kV_new = kV_old × legacyUnitsPerRotation. kP was 0 in v1 (pure feedforward) — kept.
     * ⚠ VOLTAGE-mode gains; switch to TorqueCurrentFOC needs SysId (planned tuning session).
     */
    public static final Gains DRUM_GAINS =
            Gains.pid(0, 0, 0).withSVA(1.0, 0.34 * DRUM_LEGACY_UNITS_PER_ROTATION, 0);
    public static final Gains TRANSPORT_GAINS =
            Gains.pid(0, 0, 0).withSVA(1.4, 0.4 * TRANSPORT_LEGACY_UNITS_PER_ROTATION, 0);

    /** Sim gains: plain velocity loop on the ideal model. */
    public static final Gains SIM_GAINS = Gains.pid(0.1, 0, 0).withSVA(0, 0.12, 0);

    public static final CurrentBudget CURRENT_BUDGET = CurrentBudget.of(80, 80);

    public static final MechanismSim.ModelFactory DRUM_SIM_MODEL =
            MechanismSim.rotational(DCMotor.getKrakenX60Foc(1), 0.005, 9.0);
    public static final MechanismSim.ModelFactory TRANSPORT_SIM_MODEL =
            MechanismSim.rotational(DCMotor.getKrakenX60Foc(1), 0.005, 13.7);
}
