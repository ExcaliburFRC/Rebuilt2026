package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.LogTable;

/**
 * Minimal stub to satisfy compilation when generated AutoLogged class is not present.
 * This stub extends the nested `TurretIO.TurretIOInputs` so it can be passed to
 * `TurretIO.updateInputs(...)`. It also implements `LoggableInputs` so it can be
 * passed to `Logger.processInputs(...)`.
 * If AdvantageKit codegen becomes available, this stub can be removed.
 */
public class TurretIOInputsAutoLogged extends TurretIO.TurretIOInputs implements LoggableInputs {
	@Override
	public void toLog(LogTable table) {
		// minimal no-op implementation for compile-time; real generated class writes fields
	}

	@Override
	public void fromLog(LogTable table) {
		// minimal no-op implementation for compile-time; real generated class reads fields
	}
}
