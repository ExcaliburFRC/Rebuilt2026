package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.SimMotor;

/**
 * Simulated implementation of ShooterIO using WPILib physics.
 */
public class ShooterIOSim implements ShooterIO {
    private final SimMotor hoodMotor = new SimMotor(ShooterConstants.HOOD_MOTOR_ID);
    private final SimMotor flyWheelMotorLow = new SimMotor(ShooterConstants.FLYWHEEL_MOTOR_LOW_ID);
    private final SimMotor flyWheelMotorTop = new SimMotor(ShooterConstants.FLYWHEEL_MOTOR_TOP_ID);
    private final SimMotor transportMotor = new SimMotor(ShooterConstants.TRANSPORT_MOTOR_ID);

    private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(DCMotor.getKrakenX60(1), SingleJointedArmSim.estimateMOI(0.2, 2.0), 50.0),
            DCMotor.getKrakenX60(1),
            50.0,
            0.2,
            ShooterConstants.HOOD_MIN_ANGLE_LIMIT,
            ShooterConstants.HOOD_MAX_ANGLE_LIMIT,
            false,
            0.0
    );

    private final FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(2), 0.01, 1.0),
            DCMotor.getKrakenX60(2), 
            1.0
    );

    private final FlywheelSim transportSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.01, 5.0),
            DCMotor.getKrakenX60(1),
            5.0
    );

    private double transportSimPositionRaw = 0.0;

    @Override
    public Motor getHoodMotor() { return hoodMotor; }

    @Override
    public Motor getFlywheelMotorLow() { return flyWheelMotorLow; }

    @Override
    public Motor getFlywheelMotorTop() { return flyWheelMotorTop; }

    @Override
    public Motor getTransportMotor() { return transportMotor; }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        hoodSim.setInputVoltage(hoodMotor.getAppliedVoltage());
        hoodSim.update(0.020);

        flywheelSim.setInputVoltage((flyWheelMotorLow.getAppliedVoltage() + flyWheelMotorTop.getAppliedVoltage()) / 2.0);
        flywheelSim.update(0.020);

        transportSim.setInputVoltage(transportMotor.getAppliedVoltage());
        transportSim.update(0.020);

        double hoodAngleRad = hoodSim.getAngleRads();
        
        inputs.hoodEncoderPositionRotations = hoodAngleRad / ShooterConstants.POSITION_CONVERSION_FACTOR;
        inputs.hoodEncoderAbsolutePositionRotations = inputs.hoodEncoderPositionRotations;
        
        inputs.hoodMotorPositionConverted = hoodAngleRad; 
        inputs.hoodMotorAppliedVolts = hoodMotor.getAppliedVoltage();
        inputs.hoodMotorCurrentAmps = hoodSim.getCurrentDrawAmps();

        double flyVelRotPerSec = flywheelSim.getAngularVelocityRPM() / 60.0;
        inputs.flywheelVelocityRotPerSec = flyVelRotPerSec;
        inputs.flywheelTopMotorAppliedVolts = flyWheelMotorTop.getAppliedVoltage();
        inputs.flywheelTopMotorCurrentAmps = flywheelSim.getCurrentDrawAmps() / 2.0;
        inputs.flywheelLowMotorAppliedVolts = flyWheelMotorLow.getAppliedVoltage();
        inputs.flywheelLowMotorCurrentAmps = flywheelSim.getCurrentDrawAmps() / 2.0;

        double transportRotPerSec = transportSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        transportSimPositionRaw += transportRotPerSec * 0.020;

        inputs.transportMotorAppliedVolts = transportMotor.getAppliedVoltage();
        inputs.transportMotorCurrentAmps = transportSim.getCurrentDrawAmps();

        hoodMotor.setSimulatedState(hoodAngleRad / ShooterConstants.POSITION_CONVERSION_FACTOR, hoodSim.getVelocityRadPerSec() / ShooterConstants.POSITION_CONVERSION_FACTOR, hoodSim.getCurrentDrawAmps());
        flyWheelMotorLow.setSimulatedState(0, flyVelRotPerSec, flywheelSim.getCurrentDrawAmps() / 2.0);
        flyWheelMotorTop.setSimulatedState(0, flyVelRotPerSec, flywheelSim.getCurrentDrawAmps() / 2.0);
        transportMotor.setSimulatedState(transportSimPositionRaw, transportRotPerSec, transportSim.getCurrentDrawAmps());
    }
}
