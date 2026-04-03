package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.SimMotor;

/**
 * Simulated implementation of TurretIO using WPILib physics.
 */
public class TurretIOSim implements TurretIO {
    private final SimMotor turretMotor = new SimMotor(TurretConstants.TURRET_MOTOR_ID);

    private final SingleJointedArmSim turretSim = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(DCMotor.getFalcon500(1), SingleJointedArmSim.estimateMOI(0.5, 5), 100.0),
            DCMotor.getFalcon500(1),
            100.0,
            0.5,
            TurretConstants.MIN_LIMIT - 0.5,
            TurretConstants.MAX_LIMIT + 0.5,
            false,
            0.0
    );

    @Override
    public Motor getTurretMotor() {
        return turretMotor;
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        turretSim.setInputVoltage(turretMotor.getAppliedVoltage());
        turretSim.update(0.020);

        double positionRad = turretSim.getAngleRads();
        
        inputs.encoderPositionRotations = positionRad / TurretConstants.ENCODER_POSITION_CONVERSION_FACTOR;
        inputs.encoderAbsolutePositionRotations = inputs.encoderPositionRotations;
        inputs.motorAppliedVolts = turretMotor.getAppliedVoltage();
        inputs.motorCurrentAmps = turretSim.getCurrentDrawAmps();

        turretMotor.setSimulatedState(
                positionRad / TurretConstants.MOTOR_POSITION_CONVERSION_FACTOR,
                turretSim.getVelocityRadPerSec() / TurretConstants.MOTOR_POSITION_CONVERSION_FACTOR,
                turretSim.getCurrentDrawAmps()
        );
    }
}
