package frc.robot.subsystems.example_climber;

// ═══════════════════════════════════════════════════════════════════
//  STEP 3b — REAL ROBOT IMPLEMENTATION
//
//  Key idea:
//  • Create TalonFXMotors with the correct CAN IDs and bus.
//  • Apply conversion factors so the motor returns radians / metres.
//  • Read sensors into inputs in updateInputs().
//  • NO mechanism objects here — the Subsystem owns those.
// ═══════════════════════════════════════════════════════════════════

import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.TalonFXMotor;

import static frc.robot.subsystems.example_climber.ClimberConstants.*;
import static frc.robot.Constants.SUBSYSTEMS_CANBUS;

public class ClimberIOTalonFX implements ClimberIO {

    // ── Hardware ────────────────────────────────────────────────
    private final TalonFXMotor armMotor   = new TalonFXMotor(ARM_MOTOR_ID,   SUBSYSTEMS_CANBUS);
    private final TalonFXMotor winchMotor = new TalonFXMotor(WINCH_MOTOR_ID, SUBSYSTEMS_CANBUS);

    // TODO: wire up absolute encoder (e.g. CANcoder) for the arm
    // private final CANcoder armEncoder = new CANcoder(ARM_ENCODER_ID, CANIVORE_BUS);

    public ClimberIOTalonFX() {
        // Convert rotations → radians for the arm
        armMotor.setPositionConversionFactor(ARM_POSITION_CONVERSION);
        armMotor.setVelocityConversionFactor(ARM_POSITION_CONVERSION);
        armMotor.setCurrentLimit(40, 30);

        // Convert rotations → metres for the winch
        winchMotor.setPositionConversionFactor(WINCH_POSITION_CONVERSION);
        winchMotor.setVelocityConversionFactor(WINCH_POSITION_CONVERSION);
        winchMotor.setCurrentLimit(60, 40);

        // Seed motor internal encoder from absolute encoder position
        // armMotor.setMotorPosition(armEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI);
    }

    // ── Motor getters ────────────────────────────────────────────
    @Override public Motor getArmMotor()   { return armMotor; }
    @Override public Motor getWinchMotor() { return winchMotor; }

    // ── Update ───────────────────────────────────────────────────
    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // Batch refresh all Phoenix 6 signals on this bus
        TalonFXMotor.refreshAll();

        // Arm
        inputs.armPositionRad       = armMotor.getMotorPosition();
        inputs.armVelocityRadPerSec = armMotor.getMotorVelocity();
        inputs.armAppliedVolts      = armMotor.getVoltage();
        inputs.armCurrentAmps       = armMotor.getCurrent();
        // inputs.armAbsEncoderRad  = armEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;

        // Winch
        inputs.winchPositionMeters  = winchMotor.getMotorPosition();
        inputs.winchVelocityMPerSec = winchMotor.getMotorVelocity();
        inputs.winchAppliedVolts    = winchMotor.getVoltage();
        inputs.winchCurrentAmps     = winchMotor.getCurrent();
    }
}
