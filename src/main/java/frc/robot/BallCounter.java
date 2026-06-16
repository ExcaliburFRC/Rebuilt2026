package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Logged;

import java.util.function.DoubleSupplier;

public class BallCounter implements Logged {
    public int ballCounterNum = 0;
    public final DoubleSupplier flywheelVelocitySetpoint, flywheelVelocityMeasurement;
    public Trigger hasDippedUnderSetpointTrigger;

    public BallCounter(DoubleSupplier flywheelVelocitySetpoint, DoubleSupplier flywheelVelocityMeasurement) {
        this.flywheelVelocityMeasurement = flywheelVelocityMeasurement;
        this.flywheelVelocitySetpoint = flywheelVelocitySetpoint;

        hasDippedUnderSetpointTrigger = new Trigger(
                () -> (flywheelVelocityMeasurement.getAsDouble() * 1.05 < flywheelVelocitySetpoint.getAsDouble())
        );

        // Trigger increment when velocity dips below setpoint (detects ball shot)
        hasDippedUnderSetpointTrigger.onTrue(new InstantCommand(this::incrementCount));
    }

    private void incrementCount() {
        ballCounterNum++;
    }

    public int getBallCount() {
        return ballCounterNum;
    }

    public void resetCount() {
        ballCounterNum = 0;
    }

}
