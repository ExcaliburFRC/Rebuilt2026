package frc.robot.subsystems.transport;

import frc.excalib.control.motor.controllers.Motor;
import frc.robot.util.VoidMotor;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for the Transport subsystem.
 * Separates hardware interaction from logic to enable AdvantageKit log replay.
 */
public interface TransportIO {

    @AutoLog
    class TransportIOInputs {
        /** Drum motor velocity in converted units (rot/s). */
        public double drumMotorVelocityRotPerSec = 0.0;
        /** Drum motor applied voltage in volts. */
        public double drumMotorAppliedVolts = 0.0;
        /** Drum motor supply current in amps. */
        public double drumMotorCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs from hardware. */
    default void updateInputs(TransportIOInputs inputs) {}

    /**
     * Returns the drum motor for mechanism construction.
     * Default returns a no-op motor for simulation.
     */
    default Motor getDrumMotor() {
        return new VoidMotor();
    }
}
