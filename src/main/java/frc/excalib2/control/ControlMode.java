package frc.excalib2.control;

/**
 * Selects which Phoenix 6 control-request family a mechanism uses.
 *
 * <p>{@link #TORQUE_CURRENT_FOC} is the default on real hardware (Phoenix Pro licensed):
 * torque-native, voltage-sag-immune closed loop. {@link #VOLTAGE} is the fallback used in
 * simulation and for debugging; every mechanism keeps a working voltage path.
 * {@link #DUTY_CYCLE} exists for bring-up only.
 */
public enum ControlMode {
    /** MotionMagic*TorqueCurrentFOC / TorqueCurrentFOC requests. Gains are in amps. */
    TORQUE_CURRENT_FOC,
    /** MotionMagic*Voltage / VoltageOut requests. Gains are in volts. */
    VOLTAGE,
    /** DutyCycleOut only — open loop bring-up. */
    DUTY_CYCLE
}
