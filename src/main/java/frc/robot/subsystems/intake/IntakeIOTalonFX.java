package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;

import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.intake.IntakeConstants.*;

/**
 * Real hardware implementation of IntakeIO using TalonFX motors and a CANcoder.
 */
public class IntakeIOTalonFX implements IntakeIO {

    private final TalonFXMotor fourBarMotor;
    private final TalonFXMotor rollerMotor;
    private final CANcoder angleEncoder;

    public IntakeIOTalonFX() {
        angleEncoder = new CANcoder(ANGLE_ENCODER_ID, SUBSYSTEMS_CANBUS);
        fourBarMotor = new TalonFXMotor(FOUR_BAR_MOTOR_ID, SUBSYSTEMS_CANBUS);
        fourBarMotor.setInverted(DirectionState.REVERSE);
        rollerMotor = new TalonFXMotor(ROLLER_MOTOR_ID, SUBSYSTEMS_CANBUS);
        rollerMotor.setCurrentLimit(80, 80);
    }

    @Override
    public Motor getFourBarMotor() {
        return fourBarMotor;
    }

    @Override
    public Motor getRollerMotor() {
        return rollerMotor;
    }

    public CANcoder getAngleEncoder() {
        return angleEncoder;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.angleEncoderAbsolutePositionRotations =
                angleEncoder.getAbsolutePosition().getValueAsDouble();
        inputs.fourBarMotorPositionRad = fourBarMotor.getMotorPosition();
        inputs.fourBarMotorVelocityRadPerSec = fourBarMotor.getMotorVelocity();
        inputs.fourBarMotorAppliedVolts = fourBarMotor.getVoltage();
        inputs.fourBarMotorCurrentAmps = fourBarMotor.getCurrent();
        inputs.rollerMotorAppliedVolts = rollerMotor.getVoltage();
        inputs.rollerMotorCurrentAmps = rollerMotor.getCurrent();
    }
}
