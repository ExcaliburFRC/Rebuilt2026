package frc.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.CANcoder;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;

import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.turret.TurretConstants.*;

/**
 * Real hardware implementation of TurretIO using a TalonFX motor and a CANcoder.
 */
public class TurretIOTalonFX implements TurretIO {

    private final TalonFXMotor turretMotor;
    private final CANcoder turretEncoder;

    public TurretIOTalonFX() {
        turretMotor = new TalonFXMotor(TURRET_MOTOR_ID, SUBSYSTEMS_CANBUS);
        turretEncoder = new CANcoder(TURRET_ENCODER_ID, SUBSYSTEMS_CANBUS);

        turretEncoder.setPosition(turretEncoder.getAbsolutePosition().getValueAsDouble());

        turretMotor.setCurrentLimit(120, 80);
        turretMotor.setInverted(DirectionState.REVERSE);
        turretMotor.setPositionConversionFactor(MOTOR_POSITION_CONVERSION_FACTOR);
        turretMotor.setVelocityConversionFactor(MOTOR_POSITION_CONVERSION_FACTOR);
        turretMotor.setIdleState(IdleState.BRAKE);

        // Set motor position from encoder (in converted radians)
        double encoderPositionRad = turretEncoder.getPosition().getValueAsDouble() * ENCODER_POSITION_CONVERSION_FACTOR;
        turretMotor.setMotorPosition(encoderPositionRad);
    }

    public CANcoder getTurretEncoder() {
        return turretEncoder;
    }

    @Override
    public Motor getTurretMotor() {
        return turretMotor;
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.encoderAbsolutePositionRotations = turretEncoder.getAbsolutePosition().getValueAsDouble();
        inputs.encoderPositionRotations = turretEncoder.getPosition().getValueAsDouble();
        inputs.motorAppliedVolts = turretMotor.getVoltage();
        inputs.motorCurrentAmps = turretMotor.getCurrent();
    }
}
