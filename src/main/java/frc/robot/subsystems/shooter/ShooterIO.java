package frc.robot.subsystems.shooter;

import frc.excalib.control.motor.controllers.Motor;
import frc.robot.util.VoidMotor;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for the Shooter subsystem.
 * Separates hardware interaction from logic to enable AdvantageKit log replay.
 */
public interface ShooterIO {

    @AutoLog
    class ShooterIOInputs {
        /** Hood CANcoder absolute position in rotations (raw). */
        public double hoodEncoderAbsolutePositionRotations = 0.0;
        /** Hood CANcoder position in rotations (relative, set from absolute at init). */
        public double hoodEncoderPositionRotations = 0.0;

        /** Hood motor position in converted units. */
        public double hoodMotorPositionConverted = 0.0;
        /** Hood motor applied voltage in volts. */
        public double hoodMotorAppliedVolts = 0.0;
        /** Hood motor supply current in amps. */
        public double hoodMotorCurrentAmps = 0.0;

        /** Flywheel motor group velocity in converted units (rot/s). */
        public double flywheelVelocityRotPerSec = 0.0;
        /** Flywheel top motor applied voltage in volts. */
        public double flywheelTopMotorAppliedVolts = 0.0;
        /** Flywheel top motor supply current in amps. */
        public double flywheelTopMotorCurrentAmps = 0.0;
        /** Flywheel low motor applied voltage in volts. */
        public double flywheelLowMotorAppliedVolts = 0.0;
        /** Flywheel low motor supply current in amps. */
        public double flywheelLowMotorCurrentAmps = 0.0;

        /** Transport motor applied voltage in volts. */
        public double transportMotorAppliedVolts = 0.0;
        /** Transport motor supply current in amps. */
        public double transportMotorCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs from hardware. */
    default void updateInputs(ShooterIOInputs inputs) {}

    /**
     * Returns the hood motor for mechanism construction.
     * Default returns a no-op motor for simulation.
     */
    default Motor getHoodMotor() {
        return new VoidMotor();
    }

    /**
     * Returns the flywheel low motor for mechanism construction.
     * Default returns a no-op motor for simulation.
     */
    default Motor getFlywheelMotorLow() {
        return new VoidMotor();
    }

    /**
     * Returns the flywheel top motor for mechanism construction.
     * Default returns a no-op motor for simulation.
     */
    default Motor getFlywheelMotorTop() {
        return new VoidMotor();
    }

    /**
     * Returns the transport motor for mechanism construction.
     * Default returns a no-op motor for simulation.
     */
    default Motor getTransportMotor() {
        return new VoidMotor();
    }
}
