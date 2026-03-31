package frc.robot.subsystems.transport;

import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.TalonFXMotor;

import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.transport.TransportConstants.*;

/**
 * Real hardware implementation of TransportIO using a TalonFX motor.
 */
public class TransportIOTalonFX implements TransportIO {

    private final TalonFXMotor drumMotor;

    public TransportIOTalonFX() {
        drumMotor = new TalonFXMotor(DRUM_MOTOR_ID, SUBSYSTEMS_CANBUS);
        drumMotor.setCurrentLimit(120, 70);
    }

    @Override
    public Motor getDrumMotor() {
        return drumMotor;
    }

    @Override
    public void updateInputs(TransportIOInputs inputs) {
        inputs.drumMotorVelocityRotPerSec = drumMotor.getMotorVelocity();
        inputs.drumMotorAppliedVolts = drumMotor.getVoltage();
        inputs.drumMotorCurrentAmps = drumMotor.getCurrent();
    }
}
