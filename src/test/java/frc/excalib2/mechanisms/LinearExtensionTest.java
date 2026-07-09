package frc.excalib2.mechanisms;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.excalib2.control.Gains;
import frc.excalib2.control.MotionConstraints;
import frc.excalib2.device.CANDeviceId;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.*;

class LinearExtensionTest {
    private LinearExtension extension;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
        MechanismConfig config = new MechanismConfig("Test/Elevator", new CANDeviceId(59))
                .gains(new Gains(1, 0, 0), new Gains(1, 0, 0))
                .motion(new MotionConstraints(1.0, 2.0, 0)) // trapezoidal, meters/s
                .rotorToMechanismRatio(10.0)                // rotor rot per meter
                .softLimitsMeters(Meters.of(0.0), Meters.of(1.0))
                .toleranceMeters(Meters.of(0.01))
                .simElevatorModel(DCMotor.getKrakenX60Foc(1), 10.0, 5.0, 0.02, 0.0, 1.0, 0.0);
        extension = new LinearExtension(config);
    }

    @AfterEach
    void teardown() {
        HAL.shutdown();
    }

    @Test
    void startsWithNoGoal() {
        assertFalse(extension.atGoal.getAsBoolean());
    }

    @Test
    void goalIsTrackedInMeters() {
        extension.setLengthMeters(0.5);
        assertEquals(0.5, extension.getGoalMeters(), 1e-9);
    }

    @Test
    void goalClampsToForwardSoftLimit() {
        extension.setLengthMeters(2.0);
        assertEquals(1.0, extension.getGoalMeters(), 1e-9);
    }

    @Test
    void goalClampsToReverseSoftLimit() {
        extension.setLengthMeters(-1.0);
        assertEquals(0.0, extension.getGoalMeters(), 1e-9);
    }

    @Test
    void distanceOverloadMatchesMeters() {
        extension.setLength(Meters.of(0.3));
        assertEquals(0.3, extension.getGoalMeters(), 1e-9);
    }
}
