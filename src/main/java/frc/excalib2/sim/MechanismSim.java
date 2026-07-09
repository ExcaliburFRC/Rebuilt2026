package frc.excalib2.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Physics simulation for one mechanism: a WPILib motor model driven by the TalonFX's
 * simulated output voltage, feeding rotor position/velocity back into
 * {@link TalonFXSimState} each step.
 *
 * <p>Simulation always runs the mechanism in VOLTAGE control mode (see
 * {@code ControlMode}); this class reads the sim output voltage regardless of request type.
 *
 * <p>Attach via {@code MechanismConfig.simModel(...)}; the owning {@code Mechanism} steps
 * it every loop when running in simulation.
 */
public final class MechanismSim {

    /** Minimal physics-model contract (rotations at the mechanism). */
    public interface Model {
        void setInputVoltage(double volts);

        void update(double dtSeconds);

        double positionRotations();

        double velocityRotationsPerSecond();
    }

    /** Factory so configs stay data-only until a mechanism is actually constructed in sim. */
    @FunctionalInterface
    public interface ModelFactory {
        Model create();
    }

    /**
     * Generic rotational load (flywheel, roller, drum, turret).
     *
     * @param gearbox     motor(s) driving the mechanism, e.g. {@code DCMotor.getKrakenX60Foc(2)}
     * @param moiKgMeters2 moment of inertia at the mechanism
     * @param gearing     rotor rotations per mechanism rotation
     */
    public static ModelFactory rotational(DCMotor gearbox, double moiKgMeters2, double gearing) {
        return () -> new Model() {
            private final DCMotorSim sim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(gearbox, moiKgMeters2, gearing), gearbox);

            @Override
            public void setInputVoltage(double volts) {
                sim.setInputVoltage(volts);
            }

            @Override
            public void update(double dt) {
                sim.update(dt);
            }

            @Override
            public double positionRotations() {
                return Units.radiansToRotations(sim.getAngularPositionRad());
            }

            @Override
            public double velocityRotationsPerSecond() {
                return Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
            }
        };
    }

    /**
     * Gravity-loaded single-jointed arm / pivot / hood.
     *
     * @param gearbox       motor(s) driving the joint
     * @param gearing       rotor rotations per mechanism rotation
     * @param moiKgMeters2  moment of inertia about the joint
     * @param armLengthMeters center-of-mass distance used by the WPILib model
     * @param minAngleRad   hard minimum (mechanism radians, 0 = horizontal)
     * @param maxAngleRad   hard maximum
     * @param startAngleRad initial pose
     */
    public static ModelFactory arm(DCMotor gearbox, double gearing, double moiKgMeters2,
                                   double armLengthMeters, double minAngleRad, double maxAngleRad,
                                   double startAngleRad) {
        return () -> new Model() {
            private final SingleJointedArmSim sim = new SingleJointedArmSim(
                    gearbox, gearing, moiKgMeters2, armLengthMeters,
                    minAngleRad, maxAngleRad, true, startAngleRad);

            @Override
            public void setInputVoltage(double volts) {
                sim.setInputVoltage(volts);
            }

            @Override
            public void update(double dt) {
                sim.update(dt);
            }

            @Override
            public double positionRotations() {
                return Units.radiansToRotations(sim.getAngleRads());
            }

            @Override
            public double velocityRotationsPerSecond() {
                return Units.radiansToRotations(sim.getVelocityRadPerSec());
            }
        };
    }

    /**
     * Gravity-loaded linear elevator / extension. The mechanism unit is <b>meters</b>
     * (the device's SensorToMechanismRatio is rotor rotations per meter), so this model
     * reports position/velocity in meters through the rotations-named methods.
     *
     * @param gearbox        motor(s) driving the stage
     * @param gearing        rotor rotations per meter of travel
     * @param carriageMassKg moving mass
     * @param drumRadiusMeters effective drum/pulley radius
     * @param minHeightMeters hard minimum travel
     * @param maxHeightMeters hard maximum travel
     * @param startHeightMeters initial position
     */
    public static ModelFactory elevator(DCMotor gearbox, double gearing, double carriageMassKg,
                                        double drumRadiusMeters, double minHeightMeters,
                                        double maxHeightMeters, double startHeightMeters) {
        return () -> new Model() {
            private final ElevatorSim sim = new ElevatorSim(
                    gearbox, gearing, carriageMassKg, drumRadiusMeters,
                    minHeightMeters, maxHeightMeters, true, startHeightMeters);

            @Override
            public void setInputVoltage(double volts) {
                sim.setInputVoltage(volts);
            }

            @Override
            public void update(double dt) {
                sim.update(dt);
            }

            @Override
            public double positionRotations() {
                return sim.getPositionMeters(); // mechanism unit = meters
            }

            @Override
            public double velocityRotationsPerSecond() {
                return sim.getVelocityMetersPerSecond();
            }
        };
    }

    private final TalonFXSimState simState;
    private final Model model;
    private final double rotorPerMechanism;

    /**
     * @param talon             the simulated device
     * @param model             physics model
     * @param rotorPerMechanism rotor rotations per mechanism rotation (total ratio)
     * @param inverted          whether the motor is configured clockwise-positive
     */
    public MechanismSim(TalonFX talon, Model model, double rotorPerMechanism, boolean inverted) {
        this.simState = talon.getSimState();
        this.model = model;
        this.rotorPerMechanism = rotorPerMechanism;
        simState.Orientation = inverted
                ? ChassisReference.Clockwise_Positive
                : ChassisReference.CounterClockwise_Positive;
    }

    /** Steps the physics one loop: motor voltage → model → rotor position/velocity. */
    public void update(double dtSeconds) {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        model.setInputVoltage(simState.getMotorVoltage());
        model.update(dtSeconds);
        simState.setRawRotorPosition(model.positionRotations() * rotorPerMechanism);
        simState.setRotorVelocity(model.velocityRotationsPerSecond() * rotorPerMechanism);
    }
}
