package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.SimMotor;

/**
 * Simulated implementation of IntakeIO used in simulation or replay.
 */
public class IntakeIOSim implements IntakeIO {
    private final SimMotor fourBarMotor = new SimMotor(IntakeConstants.FOUR_BAR_MOTOR_ID);
    private final SimMotor rollerMotor = new SimMotor(IntakeConstants.ROLLER_MOTOR_ID);

    private final SingleJointedArmSim fourBarSim = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(DCMotor.getKrakenX60(1), SingleJointedArmSim.estimateMOI(0.35, 3.0), 2.0),
            DCMotor.getKrakenX60(1),
            2.0,
            0.35,
            0.0,
            2.0, // Default max angle
            true,
            0.0
    );
    
    private final FlywheelSim rollerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.01, 2.0),
            DCMotor.getKrakenX60(1),
            2.0
    );

    private double rollerSimPositionRaw = 0.0;

    @Override
    public Motor getFourBarMotor() { return fourBarMotor; }

    @Override
    public Motor getRollerMotor() { return rollerMotor; }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        fourBarSim.setInputVoltage(fourBarMotor.getAppliedVoltage());
        fourBarSim.update(0.020);

        rollerSim.setInputVoltage(rollerMotor.getAppliedVoltage());
        rollerSim.update(0.020);

        double armAngleRad = fourBarSim.getAngleRads();

        inputs.angleEncoderAbsolutePositionRotations = armAngleRad / (2 * Math.PI);
        inputs.fourBarMotorPositionRad = armAngleRad;
        inputs.fourBarMotorVelocityRadPerSec = fourBarSim.getVelocityRadPerSec();
        inputs.fourBarMotorAppliedVolts = fourBarMotor.getAppliedVoltage();
        inputs.fourBarMotorCurrentAmps = fourBarSim.getCurrentDrawAmps();

        double rollerRotPerSec = rollerSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        rollerSimPositionRaw += rollerRotPerSec * 0.020;

        inputs.rollerMotorAppliedVolts = rollerMotor.getAppliedVoltage();
        inputs.rollerMotorCurrentAmps = rollerSim.getCurrentDrawAmps();

        fourBarMotor.setSimulatedState(armAngleRad, fourBarSim.getVelocityRadPerSec(), fourBarSim.getCurrentDrawAmps());
        rollerMotor.setSimulatedState(rollerSimPositionRaw, rollerRotPerSec, rollerSim.getCurrentDrawAmps());
    }
}
