package frc.robot.subsystems.example_indexer;

import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.VoidMotor;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    class IndexerIOInputs {
        public double velocityRotPerSec = 0.0;
        public double appliedVolts      = 0.0;
        public double currentAmps       = 0.0;
    }

    /** Called once per loop. Reads hardware → fills inputs. */
    default void updateInputs(IndexerIOInputs inputs) {}

    /** Returns the indexer motor. Subsystem uses it to build a FlyWheel mechanism. */
    default Motor getMotor() { return new VoidMotor(); }
}
