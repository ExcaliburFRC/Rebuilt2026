package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.LogTable;

/**
 * Minimal stub to satisfy compilation when generated AutoLogged class is not present.
 * This stub extends the nested `IntakeIO.IntakeIOInputs` so it can be passed to
 * `IntakeIO.updateInputs(...)`. It also implements `LoggableInputs` so it can be
 * passed to `Logger.processInputs(...)`.
 * If AdvantageKit codegen becomes available, this stub can be removed.
 */
public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs {
	@Override
	public void toLog(LogTable table) {
		// no-op stub
	}

	@Override
	public void fromLog(LogTable table) {
		// no-op stub
	}
}
