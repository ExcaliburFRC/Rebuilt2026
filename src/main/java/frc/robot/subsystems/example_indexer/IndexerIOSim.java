package frc.robot.subsystems.example_indexer;

// ═══════════════════════════════════════════════════════════
//  SIMULATION — only 3 meaningful lines of code!
//
//  SimMotor + FlyWheel.SimConfig = zero WPILib sim code here.
// ═══════════════════════════════════════════════════════════

import edu.wpi.first.math.system.plant.DCMotor;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.SimMotor;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;

import static frc.robot.subsystems.example_indexer.IndexerConstants.*;

public class IndexerIOSim implements IndexerIO {

    private final SimMotor motor    = new SimMotor(MOTOR_ID);
    private final FlyWheel flyWheel = new FlyWheel(
            motor, 1000, 0, GAINS,
            new FlyWheel.SimConfig(
                    DCMotor.getKrakenX60(1),  // motor model
                    MOTOR_MOI_KG_M2,           // moment of inertia (kg·m²)
                    GEAR_RATIO                  // gear ratio
            )
    );

    @Override public Motor getMotor() { return motor; }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        flyWheel.simulateStep(0.020);            // one line → physics done

        inputs.velocityRotPerSec = motor.getMotorVelocity();
        inputs.appliedVolts      = motor.getAppliedVoltage();
        inputs.currentAmps       = flyWheel.getSimCurrentAmps();
    }
}
