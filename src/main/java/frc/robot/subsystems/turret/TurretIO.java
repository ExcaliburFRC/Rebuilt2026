package frc.robot.subsystems.turret;

import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.VoidMotor;
import frc.robot.util.Target;
import monologue.Annotations.Log;
import monologue.Logged;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

/**
 * Hardware abstraction interface for the Turret subsystem.
 * Separates hardware interaction from logic to enable AdvantageKit log replay.
 */
public interface TurretIO {

    @AutoLog
    class TurretIOInputs {
        /** Turret CANcoder absolute position in rotations (raw). */
        public double encoderAbsolutePositionRotations = 0.0;
        /** Turret CANcoder position in rotations (relative, set from absolute at init). */
        public double encoderPositionRotations = 0.0;

        /** Turret motor applied voltage in volts. */
        public double motorAppliedVolts = 0.0;
        /** Turret motor supply current in amps. */
        public double motorCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs from hardware. */
    default void updateInputs(TurretIOInputs inputs) {}

    /**
     * Returns the turret motor for mechanism construction.
     * Default returns a no-op motor for simulation.
     */
    default Motor getTurretMotor() {
        return new VoidMotor();
    }
}
