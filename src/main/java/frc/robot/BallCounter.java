package frc.robot;

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

                () ->
                        (flywheelVelocityMeasurement.getAsDouble() * 1.05 < flywheelVelocitySetpoint.getAsDouble())
                                

        );

    }

    private void incrementCount() {
        ballCounterNum++;
    }

}
