package frc.robot.subsystems.example_climber;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.LogTable;

/**
 * Minimal stub to satisfy compilation when generated AutoLogged class is not present.
 * This stub extends the nested `ClimberIO.ClimberIOInputs` so it can be passed to
 * `ClimberIO.updateInputs(...)`. It also implements `LoggableInputs` so it can be
 * passed to `Logger.processInputs(...)`.
 * If AdvantageKit codegen becomes available, this stub can be removed.
 */
public class ClimberIOInputsAutoLogged extends ClimberIO.ClimberIOInputs implements LoggableInputs {
    @Override
    public void toLog(LogTable table) {
        // no-op stub
    }

    @Override
    public void fromLog(LogTable table) {
        // no-op stub
    }
}
