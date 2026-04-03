package frc.robot.subsystems.transport;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.SimMotor;

/**
 * Simulated implementation of TransportIO.
 */
public class TransportIOSim implements TransportIO {
    private final SimMotor drumMotor = new SimMotor(TransportConstants.DRUM_MOTOR_ID);

    private final FlywheelSim drumSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), 0.01, 10.0),
            DCMotor.getFalcon500(1),
            10.0
    );

    private double drumSimPositionRaw = 0.0;

    @Override
    public Motor getDrumMotor() { return drumMotor; }

    @Override
    public void updateInputs(TransportIOInputs inputs) {
        drumSim.setInputVoltage(drumMotor.getAppliedVoltage());
        drumSim.update(0.020);

        double rotPerSec = drumSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        drumSimPositionRaw += rotPerSec * 0.020;
        drumMotor.setSimulatedState(drumSimPositionRaw, rotPerSec, drumSim.getCurrentDrawAmps());

        inputs.drumMotorVelocityRotPerSec = rotPerSec;
        inputs.drumMotorAppliedVolts = drumMotor.getAppliedVoltage();
        inputs.drumMotorCurrentAmps = drumSim.getCurrentDrawAmps();
        drumMotor.setSimulatedState(0, drumSim.getAngularVelocityRadPerSec(), drumSim.getCurrentDrawAmps());
    }
}
