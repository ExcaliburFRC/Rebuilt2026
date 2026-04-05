package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.SimMotor;
import frc.excalib.mechanisms.turret.Turret;

/**
 * Simulated implementation of TurretIO.
 *
 * <p>Physics are handled internally by the {@link Turret} mechanism via its
 * {@link Turret.SimConfig}. This class is now thin — it just creates the motor,
 * creates the mechanism with sim parameters, and delegates to {@code simulateStep()}.
 */
public class TurretIOSim implements TurretIO {

    private final SimMotor turretMotor = new SimMotor(TurretConstants.TURRET_MOTOR_ID);

    /**
     * The turret mechanism owns the SingleJointedArmSim.
     * Subsystem constructor (Turret.java) will call getTurretMotor() and build
     * the real excalib Turret on top of this motor; we hold a second reference
     * here only so we can call simulateStep() each loop.
     */
    private final Turret simMechanism;

    public TurretIOSim() {
        simMechanism = new Turret(
                turretMotor,
                TurretConstants.TURRET_CONTINUOUS_SOFTLIMIT,
                TurretConstants.TURRET_GAINS,
                TurretConstants.PID_TOLERANCE,
                turretMotor::getMotorPosition,   // reads position from SimMotor
                new Turret.SimConfig(
                        DCMotor.getKrakenX60(1),
                        100.0,                    // gear ratio (motor turns / output turns)
                        SingleJointedArmSimMOI(), // MOI in kg·m²
                        TurretConstants.MIN_LIMIT - 0.5,
                        TurretConstants.MAX_LIMIT + 0.5
                )
        );
        turretMotor.setPositionConversionFactor(TurretConstants.MOTOR_POSITION_CONVERSION_FACTOR);
    }

    @Override
    public Motor getTurretMotor() {
        return turretMotor;
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        simMechanism.simulateStep(0.020);

        double positionRad = turretMotor.getMotorPosition(); // already converted by factor
        inputs.encoderPositionRotations         = positionRad / TurretConstants.ENCODER_POSITION_CONVERSION_FACTOR;
        inputs.encoderAbsolutePositionRotations = inputs.encoderPositionRotations;
        inputs.motorAppliedVolts                = turretMotor.getAppliedVoltage();
        inputs.motorCurrentAmps                 = simMechanism.getSimCurrentAmps();
    }

    /** Estimated MOI using mass=5kg, arm-length=0.5m */
    private static double SingleJointedArmSimMOI() {
        return edu.wpi.first.wpilibj.simulation.SingleJointedArmSim.estimateMOI(0.5, 5.0);
    }
}
