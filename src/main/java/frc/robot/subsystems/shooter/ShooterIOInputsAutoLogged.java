package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.LogTable;

/**
 * Minimal stub to satisfy compilation when generated AutoLogged class is not present.
 * This stub extends the nested `ShooterIO.ShooterIOInputs` so it can be passed to
 * `ShooterIO.updateInputs(...)`. It also implements `LoggableInputs` so it can be
 * passed to `Logger.processInputs(...)`.
 * If AdvantageKit codegen becomes available, this stub can be removed.
 */
public class ShooterIOInputsAutoLogged extends ShooterIO.ShooterIOInputs implements LoggableInputs {
	@Override
	public void toLog(LogTable table) {
		// no-op stub
	}

	@Override
	public void fromLog(LogTable table) {
		// no-op stub
	}
}
