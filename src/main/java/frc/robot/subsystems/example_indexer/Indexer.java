package frc.robot.subsystems.example_indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import monologue.Annotations.Log;
import monologue.Logged;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.example_indexer.IndexerConstants.*;

public class Indexer extends SubsystemBase implements Logged {

    // ── IO ───────────────────────────────────────────────────────
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    // ── Mechanism ────────────────────────────────────────────────
    // Built from the motor the IO provides.
    private final FlyWheel flyWheel;

    // ── State ────────────────────────────────────────────────────
    @Log.NT private boolean hasNote = false;

    public Indexer(IndexerIO io) {
        this.io = io;
        // Build mechanism with NO SimConfig — simulation is handled inside IOSim.
        flyWheel = new FlyWheel(io.getMotor(), 1000, 0, GAINS);
    }

    // ── Periodic ─────────────────────────────────────────────────
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    // ── Commands ─────────────────────────────────────────────────

    /** Spin up to intaking velocity. */
    public Command intakeCommand() {
        return flyWheel.setDynamicVelocityCommand(() -> INTAKE_VELOCITY, this)
                       .withName("Indexer/Intake");
    }

    /** Spin in reverse to eject a note. */
    public Command ejectCommand() {
        return flyWheel.setDynamicVelocityCommand(() -> EJECT_VELOCITY, this)
                       .withName("Indexer/Eject");
    }

    /** Apply fixed voltage to feed a note into the shooter. */
    public Command feedShotCommand() {
        return Commands.runEnd(
                () -> flyWheel.setVoltage(SHOOT_VOLTAGE),
                () -> flyWheel.setVoltage(0),
                this
        ).withName("Indexer/FeedShot");
    }

    /** Stop the motor. */
    public Command stopCommand() {
        return flyWheel.stopMotorCommand(this).withName("Indexer/Stop");
    }

    // ── Logged getters ────────────────────────────────────────────
    @Log.NT public double getVelocityRotPerSec() { return inputs.velocityRotPerSec; }
    @Log.NT public double getCurrentAmps()        { return inputs.currentAmps; }

    /** Setter injected from a beam-break sensor or similar. */
    public void setHasNote(boolean value) { this.hasNote = value; }
    public boolean hasNote()              { return hasNote; }
}
