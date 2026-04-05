package frc.robot.subsystems.transport;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.SimMotor;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;

/**
 * Simulated implementation of TransportIO.
 *
 * <p>Physics delegated to {@link FlyWheel.SimConfig} inside the mechanism.
 * This class is thin — no WPILib sim code.
 */
public class TransportIOSim implements TransportIO {

    private final SimMotor drumMotor = new SimMotor(TransportConstants.DRUM_MOTOR_ID);

    /** Drum flywheel mechanism owns the FlywheelSim internally. */
    private final FlyWheel drumMechanism;

    public TransportIOSim() {
        drumMechanism = new FlyWheel(
                drumMotor,
                100, 0,
                new Gains(),
                new FlyWheel.SimConfig(
                        DCMotor.getFalcon500(1),
                        0.01,    // MOI (kg·m²)
                        10.0     // gear ratio
                )
        );
    }

    @Override
    public Motor getDrumMotor() { return drumMotor; }

    @Override
    public void updateInputs(TransportIOInputs inputs) {
        drumMechanism.simulateStep(0.020);

        inputs.drumMotorVelocityRotPerSec = drumMotor.getMotorVelocity();
        inputs.drumMotorAppliedVolts      = drumMotor.getAppliedVoltage();
        inputs.drumMotorCurrentAmps       = drumMechanism.getSimCurrentAmps();
    }
}
