package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;

import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

/**
 * Real hardware implementation of ShooterIO using TalonFX motors and a CANcoder.
 */
public class ShooterIOTalonFX implements ShooterIO {

    private final TalonFXMotor hoodMotor;
    private final TalonFXMotor flyWheelMotorTop;
    private final TalonFXMotor flyWheelMotorLow;
    private final TalonFXMotor transportMotor;
    private final CANcoder hoodEncoder;
    private final MotorGroup shooterMotorGroup;

    public ShooterIOTalonFX() {
        hoodMotor = new TalonFXMotor(HOOD_MOTOR_ID, SUBSYSTEMS_CANBUS);
        flyWheelMotorLow = new TalonFXMotor(FLYWHEEL_MOTOR_LOW_ID, SUBSYSTEMS_CANBUS);
        flyWheelMotorTop = new TalonFXMotor(FLYWHEEL_MOTOR_TOP_ID, SUBSYSTEMS_CANBUS);
        transportMotor = new TalonFXMotor(TRANSPORT_MOTOR_ID, SUBSYSTEMS_CANBUS);
        hoodEncoder = new CANcoder(HOOD_ENCODER_ID, SUBSYSTEMS_CANBUS);

        shooterMotorGroup = new MotorGroup(flyWheelMotorLow, flyWheelMotorTop);
        shooterMotorGroup.setIdleState(IdleState.BRAKE);
        shooterMotorGroup.setMotorPosition(0);
        shooterMotorGroup.setVelocityConversionFactor((double) 40 / 48);
        shooterMotorGroup.setPositionConversionFactor((double) 40 / 48);

        flyWheelMotorLow.setInverted(DirectionState.FORWARD);
        flyWheelMotorTop.setInverted(DirectionState.FORWARD);

        flyWheelMotorTop.setCurrentLimit(120, 80);
        flyWheelMotorLow.setCurrentLimit(120, 80);

        hoodEncoder.setPosition(hoodEncoder.getAbsolutePosition().getValueAsDouble());

        hoodMotor.setIdleState(IdleState.BRAKE);
        hoodMotor.setInverted(DirectionState.FORWARD);
        hoodMotor.setPositionConversionFactor(POSITION_CONVERSION_FACTOR * ((double) -0.208 / 1.497) * 1.0231);
        hoodMotor.setMotorPosition(hoodEncoder.getAbsolutePosition().getValueAsDouble() * POSITION_CONVERSION_FACTOR);
        hoodMotor.setIdleState(IdleState.COAST);
    }

    public CANcoder getHoodEncoder() {
        return hoodEncoder;
    }

    public MotorGroup getShooterMotorGroup() {
        return shooterMotorGroup;
    }

    @Override
    public Motor getHoodMotor() {
        return hoodMotor;
    }

    @Override
    public Motor getFlywheelMotorLow() {
        return flyWheelMotorLow;
    }

    @Override
    public Motor getFlywheelMotorTop() {
        return flyWheelMotorTop;
    }

    @Override
    public Motor getTransportMotor() {
        return transportMotor;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.hoodEncoderAbsolutePositionRotations = hoodEncoder.getAbsolutePosition().getValueAsDouble();
        inputs.hoodEncoderPositionRotations = hoodEncoder.getPosition().getValueAsDouble();
        inputs.hoodMotorPositionConverted = hoodMotor.getMotorPosition();
        inputs.hoodMotorAppliedVolts = hoodMotor.getVoltage();
        inputs.hoodMotorCurrentAmps = hoodMotor.getCurrent();
        inputs.flywheelVelocityRotPerSec = flyWheelMotorLow.getMotorVelocity();
        inputs.flywheelTopMotorAppliedVolts = flyWheelMotorTop.getVoltage();
        inputs.flywheelTopMotorCurrentAmps = flyWheelMotorTop.getCurrent();
        inputs.flywheelLowMotorAppliedVolts = flyWheelMotorLow.getVoltage();
        inputs.flywheelLowMotorCurrentAmps = flyWheelMotorLow.getCurrent();
        inputs.transportMotorAppliedVolts = transportMotor.getVoltage();
        inputs.transportMotorCurrentAmps = transportMotor.getCurrent();
    }
}
