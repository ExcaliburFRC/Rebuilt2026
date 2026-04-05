package frc.robot.subsystems.example_climber;

// ═══════════════════════════════════════════════════════════════════
//  STEP 2 — IO INTERFACE
//
//  Rules:
//  • All inputs are declared in the @AutoLog inner class → AK logs them
//  • Motor getters return the Motor object so the Subsystem can wire
//    it into the mechanism.
//  • Default implementations return VoidMotor / zero so the interface
//    compiles even before you write the real/sim implementations.
// ═══════════════════════════════════════════════════════════════════

import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.VoidMotor;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

    // ── Logged inputs ───────────────────────────────────────────
    @AutoLog
    class ClimberIOInputs {
        // Arm
        public double armPositionRad         = 0.0;
        public double armVelocityRadPerSec   = 0.0;
        public double armAppliedVolts        = 0.0;
        public double armCurrentAmps         = 0.0;
        public double armAbsEncoderRad       = 0.0;

        // Winch
        public double winchPositionMeters    = 0.0;
        public double winchVelocityMPerSec   = 0.0;
        public double winchAppliedVolts      = 0.0;
        public double winchCurrentAmps       = 0.0;
    }

    // ── Update call ─────────────────────────────────────────────
    /** Called once per loop. Reads hardware → fills inputs. */
    default void updateInputs(ClimberIOInputs inputs) {}

    // ── Motor getters ────────────────────────────────────────────
    // The Subsystem will call these once in its constructor to get
    // the Motor objects and pass them to Mechanism constructors.

    /** Returns the arm motor (or SimMotor in sim). */
    default Motor getArmMotor()   { return new VoidMotor(); }

    /** Returns the winch motor (or SimMotor in sim). */
    default Motor getWinchMotor() { return new VoidMotor(); }
}
