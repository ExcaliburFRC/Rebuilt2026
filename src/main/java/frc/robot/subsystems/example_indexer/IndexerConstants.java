package frc.robot.subsystems.example_indexer;

// ═══════════════════════════════════════════════════════════════════
//  SIMPLE EXAMPLE — INDEXER (single flywheel motor)
//
//  This is the minimal case: one motor, one FlyWheel mechanism.
//  Demonstrates the same IO pattern in the fewest lines possible.
//
//  Files:
//    IndexerConstants.java  ← numbers
//    IndexerIO.java         ← interface
//    IndexerIOSim.java      ← simulation (3 meaningful lines)
//    IndexerIOTalonFX.java  ← real robot
//    Indexer.java           ← subsystem + commands
// ═══════════════════════════════════════════════════════════════════

import frc.excalib.control.gains.Gains;

public class IndexerConstants {
    public static final int    MOTOR_ID         = 60;
    public static final double GEAR_RATIO        = 1.0;  // direct drive
    public static final double VELOCITY_RPM_MAX  = 4000;
    public static final double INTAKE_VELOCITY   = 1500; // rot/min
    public static final double EJECT_VELOCITY    = -1500;
    public static final double SHOOT_VOLTAGE     = 6.0;  // volts for feed shot
    public static final double MOTOR_MOI_KG_M2   = 0.008;

    /** Converts motor rotations → rotations (1:1 here, but set if geared). */
    public static final double VELOCITY_CONVERSION = 1.0;

    public static final Gains GAINS = new Gains(
            0.001, 0, 0,     // kp, ki, kd  (light PID — FF-dominant)
            0.3,  0.00215, 0, 0  // ks, kv (from sysid), ka, kg
    );
}
