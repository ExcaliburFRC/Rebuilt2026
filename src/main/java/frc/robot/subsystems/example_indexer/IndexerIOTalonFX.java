package frc.robot.subsystems.example_indexer;

import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.TalonFXMotor;

import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.example_indexer.IndexerConstants.*;

public class IndexerIOTalonFX implements IndexerIO {

    private final TalonFXMotor motor = new TalonFXMotor(MOTOR_ID, SUBSYSTEMS_CANBUS);

    public IndexerIOTalonFX() {
        motor.setVelocityConversionFactor(VELOCITY_CONVERSION);
        motor.setCurrentLimit(40, 30);
    }

    @Override public Motor getMotor() { return motor; }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        TalonFXMotor.refreshAll();
        inputs.velocityRotPerSec = motor.getMotorVelocity();
        inputs.appliedVolts      = motor.getVoltage();
        inputs.currentAmps       = motor.getCurrent();
    }
}
