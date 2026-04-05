package frc.robot.subsystems.example_climber;

// ═══════════════════════════════════════════════════════════════════
//  STEP 4 — THE SUBSYSTEM
//
//  Key ideas:
//  • Receives a ClimberIO in the constructor (injected by Superstructure).
//  • Calls io.updateInputs() each loop — this fills inputs AND runs
//    physics simulation when using ClimberIOSim.
//  • Builds the excalib Mechanisms from the motors the IO provides.
//  • All Commands are built here as factory methods.
//  • Position feedback reads from `inputs` (logged), not directly
//    from the motor — this makes AK replay work correctly.
// ═══════════════════════════════════════════════════════════════════

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.mechanisms.linear_extension.LinearExtension;
import monologue.Annotations.Log;
import monologue.Logged;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.example_climber.ClimberConstants.*;

public class Climber extends SubsystemBase implements Logged {

    // ── IO ──────────────────────────────────────────────────────
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    // ── Mechanisms ───────────────────────────────────────────────
    // These are built from the motors the IO gives us.
    // On real robot → TalonFXMotors.  In sim → SimMotors.
    // The mechanisms do NOT know which one they have.
    private final Arm arm;
    private final LinearExtension winch;

    // ── State ────────────────────────────────────────────────────
    @Log.NT private boolean armAtGoal   = false;
    @Log.NT private boolean winchAtGoal = false;

    // ────────────────────────────────────────────────────────────
    public Climber(ClimberIO io) {
        this.io = io;

        // Build mechanisms from the motors the IO provides.
        // Pass null for SimConfig — the IOSim already sets that up
        // in the SimMotor internally via its own mechanism instance.
        arm = new Arm(
                io.getArmMotor(),
                () -> inputs.armPositionRad,   // ← reads from logged inputs (replay-safe)
                new SoftLimit(() -> -2.0, () -> 2.0),
                ARM_GAINS,
                new Mass(() -> ARM_LENGTH_M / 2, () -> 0.0, ARM_MASS_KG)
                // No SimConfig here — the IOSim own sim handles physics
        );

        winch = new LinearExtension(
                io.getWinchMotor(),
                () -> inputs.winchPositionMeters, // ← reads from logged inputs (replay-safe)
                () -> inputs.armPositionRad,       // arm angle for gravity feedforward
                WINCH_GAINS,
                WINCH_CONSTRAINTS,
                WINCH_TOLERANCE_M
                // No SimConfig here
        );
    }

    // ── Periodic ─────────────────────────────────────────────────
    @Override
    public void periodic() {
        // 1. Update inputs (reads hardware OR steps physics in sim)
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        // 2. Log convenience values
        Logger.recordOutput("Climber/ArmAtGoal",   armAtGoal);
        Logger.recordOutput("Climber/WinchAtGoal", winchAtGoal);
    }

    // ── Commands ──────────────────────────────────────────────────
    /**
     * Moves the arm to the stow position (idle / rest).
     */
    public Command stowArmCommand() {
        return arm.goToAngleCommand(
                ARM_STOW_ANGLE_RAD,
                atGoal -> armAtGoal = atGoal,
                ARM_TOLERANCE_RAD,
                this
        ).withName("Climber/StowArm");
    }

    /**
     * Moves the arm to the climb position.
     */
    public Command deployArmCommand() {
        return arm.goToAngleCommand(
                ARM_CLIMB_ANGLE_RAD,
                atGoal -> armAtGoal = atGoal,
                ARM_TOLERANCE_RAD,
                this
        ).withName("Climber/DeployArm");
    }

    /**
     * Extends the winch fully.
     */
    public Command extendWinchCommand() {
        return winch.extendCommand(
                () -> WINCH_EXTEND_LENGTH_M,
                this
        ).withName("Climber/Extend");
    }

    /**
     * Retracts the winch fully (pulls robot up).
     */
    public Command retractWinchCommand() {
        return winch.extendCommand(
                () -> WINCH_RETRACT_LENGTH_M,
                this
        ).withName("Climber/Retract");
    }

    /**
     * Full climb sequence:
     *   1. Deploy arm to climb angle
     *   2. Extend winch
     *   3. Retract winch (lift robot)
     */
    public Command climbSequenceCommand() {
        return Commands.sequence(
                deployArmCommand().until(() -> armAtGoal),
                extendWinchCommand().until(() -> winchAtGoal),
                retractWinchCommand()
        ).withName("Climber/ClimbSequence");
    }

    // ── Logged getters ────────────────────────────────────────────
    @Log.NT public double getArmAngleRad()      { return inputs.armPositionRad; }
    @Log.NT public double getWinchPositionM()   { return inputs.winchPositionMeters; }
    @Log.NT public double getArmCurrentAmps()   { return inputs.armCurrentAmps; }
    @Log.NT public double getWinchCurrentAmps() { return inputs.winchCurrentAmps; }
}
