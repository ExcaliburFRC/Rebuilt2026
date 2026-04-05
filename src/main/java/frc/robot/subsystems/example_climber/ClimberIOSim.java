package frc.robot.subsystems.example_climber;

// ═══════════════════════════════════════════════════════════════════
//  STEP 3a — SIMULATION IMPLEMENTATION
//
//  Key idea (new architecture):
//  • Create SimMotors.
//  • Create the excalib Mechanisms with a SimConfig — the mechanism
//    owns the WPILib physics sim internally.
//  • In updateInputs(), call mechanism.simulateStep(0.020) then
//    read back from the motor and mechanism.
//
//  You do NOT need to import FlywheelSim / SingleJointedArmSim /
//  ElevatorSim here — they live inside the mechanism now.
// ═══════════════════════════════════════════════════════════════════

import edu.wpi.first.math.system.plant.DCMotor;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.SimMotor;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.mechanisms.linear_extension.LinearExtension;

import static frc.robot.subsystems.example_climber.ClimberConstants.*;

public class ClimberIOSim implements ClimberIO {

    // ── SimMotors ───────────────────────────────────────────────
    private final SimMotor armMotor   = new SimMotor(ARM_MOTOR_ID);
    private final SimMotor winchMotor = new SimMotor(WINCH_MOTOR_ID);

    // ── Mechanisms (own the physics sims internally) ─────────────
    private final Arm arm;
    private final LinearExtension winch;

    public ClimberIOSim() {
        // Set conversion factors so getMotorPosition() returns radians / metres
        armMotor.setPositionConversionFactor(ARM_POSITION_CONVERSION);
        winchMotor.setPositionConversionFactor(WINCH_POSITION_CONVERSION);

        // ── Arm ────────────────────────────────────────────────
        // Pass SimConfig → Arm creates SingleJointedArmSim internally.
        arm = new Arm(
                armMotor,
                armMotor::getMotorPosition,         // position feedback (radians after conversion)
                ARM_GAINS.kg > 0
                        ? new frc.excalib.control.limits.SoftLimit(
                                () -> -2.0, () -> 2.0)
                        : new frc.excalib.control.limits.SoftLimit(
                                () -> -2.0, () -> 2.0),
                ARM_GAINS,
                new Mass(() -> ARM_LENGTH_M / 2, () -> 0.0, ARM_MASS_KG),
                new Arm.SimConfig(
                        DCMotor.getKrakenX60(1), // motor model (already 1 motor)
                        ARM_GEAR_RATIO,
                        ARM_LENGTH_M,
                        ARM_MASS_KG,
                        ARM_MIN_ANGLE_RAD,
                        ARM_MAX_ANGLE_RAD,
                        true,           // simulate gravity
                        ARM_STOW_ANGLE_RAD
                )
        );

        // ── Winch ───────────────────────────────────────────────
        // Pass SimConfig → LinearExtension creates ElevatorSim internally.
        winch = new LinearExtension(
                winchMotor,
                winchMotor::getMotorPosition,       // position (metres after conversion)
                () -> ARM_CLIMB_ANGLE_RAD,          // arm angle for gravity calc (constant in sim)
                WINCH_GAINS,
                WINCH_CONSTRAINTS,
                WINCH_TOLERANCE_M,
                new LinearExtension.SimConfig(
                        DCMotor.getKrakenX60(1),
                        WINCH_GEAR_RATIO,
                        WINCH_CARRIAGE_MASS_KG,
                        WINCH_DRUM_RADIUS_M,
                        WINCH_MIN_LENGTH_M,
                        WINCH_MAX_LENGTH_M,
                        true,           // simulate gravity
                        WINCH_RETRACT_LENGTH_M
                )
        );
    }

    // ── Motor getters ────────────────────────────────────────────
    @Override public Motor getArmMotor()   { return armMotor; }
    @Override public Motor getWinchMotor() { return winchMotor; }

    // ── Update ───────────────────────────────────────────────────
    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // Step physics — mechanisms update their internal sims and
        // feed position / velocity back into the SimMotors.
        arm.simulateStep(0.020);
        winch.simulateStep(0.020);

        // Arm inputs
        inputs.armPositionRad       = armMotor.getMotorPosition();
        inputs.armVelocityRadPerSec = armMotor.getMotorVelocity();
        inputs.armAppliedVolts      = armMotor.getAppliedVoltage();
        inputs.armCurrentAmps       = arm.getSimCurrentAmps();
        inputs.armAbsEncoderRad     = armMotor.getMotorPosition(); // same source in sim

        // Winch inputs
        inputs.winchPositionMeters  = winchMotor.getMotorPosition();
        inputs.winchVelocityMPerSec = winchMotor.getMotorVelocity();
        inputs.winchAppliedVolts    = winchMotor.getAppliedVoltage();
        inputs.winchCurrentAmps     = winch.getSimCurrentAmps();
    }
}
