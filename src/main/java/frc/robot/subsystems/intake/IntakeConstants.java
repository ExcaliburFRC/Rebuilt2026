package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.excalib2.control.CurrentBudget;
import frc.excalib2.control.Gains;
import frc.excalib2.control.MotionConstraints;
import frc.excalib2.sim.MechanismSim;

public class IntakeConstants {

    // ==== IDs ==== //
    public static final int LEFT_FOUR_BAR_MOTOR_ID = 10;
    public static final int RIGHT_FOUR_BAR_MOTOR_ID = 34;
    public static final int ROLLER_MOTOR_ID = 13;
    public static final int ANGLE_ENCODER_ID = 12; // present on the robot; not yet fused (needs offset calibration)

    // ==== Geometry (mechanism radians; 0 = horizontal, matches v1 gravity model) ==== //
    public static final double GEARING = 14.14; // rotor rotations per four-bar rotation (v1: 2π/14.14 rad per rotor rot)
    public static final double INTAKE_MIN_ANGLE_RAD = 0;
    public static final double INTAKE_MAX_ANGLE_RAD = 2.4;
    /** v1 seed position at stow: Math.PI - 0.732601 - 0.159. */
    public static final double START_ANGLE_RAD = Math.PI - 0.732601 - 0.159;
    public static final double INTAKE_ANGLE_TOLERANCE_RAD = 0.003;
    /** PUMP oscillation, from v1 pumpCommand. */
    public static final double PUMP_HIGH_ANGLE_RAD = 1.60;
    public static final double PUMP_LOW_ANGLE_RAD = 0;
    public static final double PUMP_PERIOD_SECONDS = 0.5;

    /**
     * Ported v1 gains (volts): kP 2 V/rad → ×2π V/rotation; kG 0.8 arm-cosine.
     * ⚠ VOLTAGE-mode gains; FOC switch after SysId.
     */
    public static final Gains FOUR_BAR_GAINS = Gains.pid(2 * 2 * Math.PI, 0, 0)
            .withGravity(0.8, GravityTypeValue.Arm_Cosine);
    public static final Gains FOUR_BAR_SIM_GAINS = Gains.pid(30, 0, 0)
            .withGravity(0.3, GravityTypeValue.Arm_Cosine);

    /**
     * v1 limited arm speed two ways: velocity clamp ±1.5 rad/s and a ±1.5 V output clamp.
     * Ported as MotionMagic cruise 1.5 rad/s + the same peak-voltage clamp.
     */
    public static final MotionConstraints FOUR_BAR_MOTION = MotionConstraints.trapezoidal(
            Units.radiansToRotations(1.5), 2.0, 0);
    public static final double FOUR_BAR_PEAK_VOLTS = 1.5;

    public static final CurrentBudget FOUR_BAR_BUDGET = CurrentBudget.of(80, 60);
    public static final CurrentBudget ROLLER_BUDGET = CurrentBudget.of(80, 30);

    public static final MechanismSim.ModelFactory FOUR_BAR_SIM_MODEL = MechanismSim.arm(
            DCMotor.getKrakenX60Foc(2), GEARING, 0.3, 0.3,
            INTAKE_MIN_ANGLE_RAD, INTAKE_MAX_ANGLE_RAD, START_ANGLE_RAD);
    public static final MechanismSim.ModelFactory ROLLER_SIM_MODEL =
            MechanismSim.rotational(DCMotor.getKrakenX60Foc(1), 0.002, 3.0);
}
