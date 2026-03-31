package frc.robot.subsystems.intake;

import frc.excalib.control.motor.controllers.Motor;
import frc.robot.util.VoidMotor;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for the Intake subsystem.
 * Separates hardware interaction from logic to enable AdvantageKit log replay.
 */
public interface IntakeIO {

    @AutoLog
    class IntakeIOInputs {
        /** Absolute position of the arm angle encoder, in rotations (raw). */
        public double angleEncoderAbsolutePositionRotations = 0.0;

        /** Four-bar motor position in converted units (radians). */
        public double fourBarMotorPositionRad = 0.0;
        /** Four-bar motor velocity in converted units (rad/s). */
        public double fourBarMotorVelocityRadPerSec = 0.0;
        /** Four-bar motor applied voltage in volts. */
        public double fourBarMotorAppliedVolts = 0.0;
        /** Four-bar motor supply current in amps. */
        public double fourBarMotorCurrentAmps = 0.0;

        /** Roller motor applied voltage in volts. */
        public double rollerMotorAppliedVolts = 0.0;
        /** Roller motor supply current in amps. */
        public double rollerMotorCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs from hardware. */
    default void updateInputs(IntakeIOInputs inputs) {}

    /**
     * Returns the four-bar arm motor used to construct mechanism abstractions.
     * The default returns a no-op motor suitable for simulation.
     */
    default Motor getFourBarMotor() {
        return new VoidMotor();
    }

    /**
     * Returns the roller motor used to construct mechanism abstractions.
     * The default returns a no-op motor suitable for simulation.
     */
    default Motor getRollerMotor() {
        return new VoidMotor();
    }
}
