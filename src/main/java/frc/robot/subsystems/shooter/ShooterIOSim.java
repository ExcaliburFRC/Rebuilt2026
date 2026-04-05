package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.SimMotor;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import frc.excalib.mechanisms.turret.Turret;

/**
 * Simulated implementation of ShooterIO.
 *
 * <p>Physics delegated to {@link Turret.SimConfig} (hood) and {@link FlyWheel.SimConfig}
 * (flywheel + transport) inside the mechanisms. No WPILib sim code lives here.
 *
 * <p>The flywheel uses a <em>single</em> shared physics sim driven by the average voltage
 * of the two motors, which is the correct approach for two mechanically-coupled motors.
 */
public class ShooterIOSim implements ShooterIO {

    private final SimMotor hoodMotor        = new SimMotor(ShooterConstants.HOOD_MOTOR_ID);
    private final SimMotor flyWheelMotorLow = new SimMotor(ShooterConstants.FLYWHEEL_MOTOR_LOW_ID);
    private final SimMotor flyWheelMotorTop = new SimMotor(ShooterConstants.FLYWHEEL_MOTOR_TOP_ID);
    private final SimMotor transportMotor   = new SimMotor(ShooterConstants.TRANSPORT_MOTOR_ID);

    /** Hood Turret mechanism owns the SingleJointedArmSim internally. */
    private final Turret hoodMechanism;

    /**
     * Flywheel mechanism: {@link FlyWheel.SimConfig} with 2-Kraken motor model.
     * Driven from flyWheelMotorLow; top motor follows via average voltage in simulateStep.
     */
    private final FlyWheel flywheelMechanism;

    /** Transport belt flywheel mechanism. */
    private final FlyWheel transportMechanism;

    public ShooterIOSim() {
        hoodMotor.setPositionConversionFactor(ShooterConstants.POSITION_CONVERSION_FACTOR);

        hoodMechanism = new Turret(
                hoodMotor,
                new ContinuousSoftLimit(
                        () -> ShooterConstants.HOOD_MIN_ANGLE_LIMIT,
                        () -> ShooterConstants.HOOD_MAX_ANGLE_LIMIT
                ),
                ShooterConstants.HOOD_PID_GAINS,
                ShooterConstants.HOOD_TOLERANCE,
                hoodMotor::getMotorPosition,
                new Turret.SimConfig(
                        DCMotor.getKrakenX60(1),
                        50.0,   // gear ratio
                        edu.wpi.first.wpilibj.simulation.SingleJointedArmSim.estimateMOI(0.2, 2.0),
                        ShooterConstants.HOOD_MIN_ANGLE_LIMIT,
                        ShooterConstants.HOOD_MAX_ANGLE_LIMIT
                )
        );

        flywheelMechanism = new FlyWheel(
                flyWheelMotorLow,
                ShooterConstants.FLYWHEEL_MAX_ACCELERATION,
                ShooterConstants.FLYWHEEL_MAX_JERK,
                ShooterConstants.FLYWHEEL_GAINS,
                new FlyWheel.SimConfig(
                        DCMotor.getKrakenX60(2),  // 2-motor model
                        0.01,                      // MOI (kg·m²)
                        1.0                        // gear ratio (direct drive)
                )
        );

        transportMechanism = new FlyWheel(
                transportMotor,
                100, 0,
                new Gains(),
                new FlyWheel.SimConfig(
                        DCMotor.getKrakenX60(1),
                        0.01,   // MOI (kg·m²)
                        5.0     // gear ratio
                )
        );
    }

    @Override public Motor getHoodMotor()        { return hoodMotor; }
    @Override public Motor getFlywheelMotorLow() { return flyWheelMotorLow; }
    @Override public Motor getFlywheelMotorTop() { return flyWheelMotorTop; }
    @Override public Motor getTransportMotor()   { return transportMotor; }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // For the dual-motor flywheel: both motors drive the same shaft.
        // Feed the average voltage to the primary motor's sim so physics is accurate.
        double avgFlyVoltage = (flyWheelMotorLow.getAppliedVoltage() + flyWheelMotorTop.getAppliedVoltage()) / 2.0;
        // Temporarily override the primary motor's voltage for the physics step.
        // (SimMotor.setMotorVoltage is clamped ±12V and NaN-safe)
        flyWheelMotorLow.setMotorVoltage(avgFlyVoltage);

        hoodMechanism.simulateStep(0.020);
        flywheelMechanism.simulateStep(0.020);
        transportMechanism.simulateStep(0.020);

        // Mirror flywheel state to top motor
        flyWheelMotorTop.setSimulatedState(0, flyWheelMotorLow.getMotorVelocity(), flywheelMechanism.getSimCurrentAmps() / 2.0);

        // Hood
        double hoodAngleRad = hoodMotor.getMotorPosition();
        inputs.hoodEncoderPositionRotations          = hoodAngleRad / ShooterConstants.POSITION_CONVERSION_FACTOR;
        inputs.hoodEncoderAbsolutePositionRotations  = inputs.hoodEncoderPositionRotations;
        inputs.hoodMotorPositionConverted            = hoodAngleRad;
        inputs.hoodMotorAppliedVolts                 = hoodMotor.getAppliedVoltage();
        inputs.hoodMotorCurrentAmps                  = hoodMechanism.getSimCurrentAmps();

        // Flywheel
        inputs.flywheelVelocityRotPerSec    = flyWheelMotorLow.getMotorVelocity();
        inputs.flywheelTopMotorAppliedVolts = flyWheelMotorTop.getAppliedVoltage();
        inputs.flywheelTopMotorCurrentAmps  = flywheelMechanism.getSimCurrentAmps() / 2.0;
        inputs.flywheelLowMotorAppliedVolts = flyWheelMotorLow.getAppliedVoltage();
        inputs.flywheelLowMotorCurrentAmps  = flywheelMechanism.getSimCurrentAmps() / 2.0;

        // Transport
        inputs.transportMotorAppliedVolts = transportMotor.getAppliedVoltage();
        inputs.transportMotorCurrentAmps  = transportMechanism.getSimCurrentAmps();
    }
}
