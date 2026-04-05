package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.SimMotor;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;

/**
 * Simulated implementation of IntakeIO.
 *
 * <p>Physics are handled internally by the {@link Arm} (four-bar) and {@link FlyWheel}
 * (roller) mechanisms via their {@link Arm.SimConfig} and {@link FlyWheel.SimConfig}.
 * This class is now thin — all it does is create motors, set up mechanisms with
 * sim configs, and call {@code simulateStep()} each loop.
 */
public class IntakeIOSim implements IntakeIO {

    private final SimMotor fourBarMotor = new SimMotor(IntakeConstants.FOUR_BAR_MOTOR_ID);
    private final SimMotor rollerMotor  = new SimMotor(IntakeConstants.ROLLER_MOTOR_ID);

    /** Four-bar arm mechanism owns the SingleJointedArmSim internally. */
    private final Arm fourBarMechanism;

    /** Roller flywheel mechanism owns the FlywheelSim internally. */
    private final FlyWheel rollerMechanism;

    public IntakeIOSim() {
        fourBarMechanism = new Arm(
                fourBarMotor,
                fourBarMotor::getMotorPosition,        // position feedback from SimMotor
                new SoftLimit(
                        () -> IntakeConstants.ARM_MIN_VELOCITY_LIMIT,
                        () -> IntakeConstants.ARM_MAX_VELOCITY_LIMIT
                ),
                new Gains(3, 0, 0, 0.45, 0, 0, 0.82),
                new Mass(() -> 0.175, () -> 0.0, IntakeConstants.ARM_MASS),
                new Arm.SimConfig(
                        DCMotor.getKrakenX60(1),        // single Kraken
                        2.0,                            // gear ratio
                        0.35,                           // arm length (m)
                        3.0,                            // arm mass (kg)
                        IntakeConstants.INTAKE_MIN_ANGLE,
                        IntakeConstants.INTAKE_MAX_ANGLE,
                        true,                           // simulate gravity
                        0.0                             // start angle (rad)
                )
        );

        rollerMechanism = new FlyWheel(
                rollerMotor,
                100, 0,
                new Gains(),
                new FlyWheel.SimConfig(
                        DCMotor.getKrakenX60(1),
                        0.01,    // MOI (kg·m²)
                        2.0      // gear ratio
                )
        );
    }

    @Override
    public Motor getFourBarMotor() { return fourBarMotor; }

    @Override
    public Motor getRollerMotor() { return rollerMotor; }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        fourBarMechanism.simulateStep(0.020);
        rollerMechanism.simulateStep(0.020);

        // Four-bar arm
        inputs.fourBarMotorPositionRad         = fourBarMotor.getMotorPosition();
        inputs.fourBarMotorVelocityRadPerSec   = fourBarMotor.getMotorVelocity();
        inputs.fourBarMotorAppliedVolts        = fourBarMotor.getAppliedVoltage();
        inputs.fourBarMotorCurrentAmps         = fourBarMechanism.getSimCurrentAmps();
        inputs.angleEncoderAbsolutePositionRotations
                = fourBarMotor.getMotorPosition() / (2 * Math.PI);

        // Roller
        inputs.rollerMotorAppliedVolts = rollerMotor.getAppliedVoltage();
        inputs.rollerMotorCurrentAmps  = rollerMechanism.getSimCurrentAmps();
    }
}
