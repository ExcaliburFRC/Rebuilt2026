package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.TurretOffsetGetter;

import static edu.wpi.first.units.Units.*;

/**
 * Hand-authored Tuner-style swerve constants, derived from the ExcaLib v1 values
 * (derivations inline). ⚠ Requires on-robot verification before competition use:
 * inversion directions, CANcoder offsets (assumed burned into the devices — v1 read
 * absolute positions directly), and the coupling ratio (unknown, set 0).
 */
public final class SwerveConfig {
    private SwerveConfig() {
    }

    // Teleop limits (carried over from v1 Constants — still the cautious test values)
    public static final double MAX_SPEED_MPS = 2.0;
    public static final double MAX_OMEGA_RAD_PER_SEC = 1.5;

    public static final PIDConstants PP_TRANSLATION_GAINS = new PIDConstants(10.0, 0, 0);
    public static final PIDConstants PP_ROTATION_GAINS = new PIDConstants(5.0, 0, 0);

    // v1 drive conversion: 4 in wheel, gearing 5.27 → circumference 0.31919 m
    private static final double DRIVE_GEAR_RATIO = 5.27;
    private static final double STEER_GEAR_RATIO = 26.09090909090909; // v1: 2π/26.0909 rad per rotor rot
    private static final double WHEEL_RADIUS_INCHES = 2.0;
    private static final double WHEEL_CIRCUMFERENCE_METERS = edu.wpi.first.math.util.Units.inchesToMeters(4) * Math.PI;

    // v1 steer gains: kP 5.2 V/rad → ×2π V/steer-rotation. v1 drive: kP 0, kS 0.2..0.42
    // (per-module in v1; single mid value now), kV 2.008 V/(m/s) × circumference V/(rot/s).
    private static final Slot0Configs STEER_GAINS = new Slot0Configs()
            .withKP(5.2 * 2 * Math.PI).withKD(0);
    private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
            .withKS(0.3).withKV(2.008 * WHEEL_CIRCUMFERENCE_METERS);

    private static final double SPEED_AT_12V_MPS = 12.0 / 2.008; // ≈5.97, from v1 kV

    private static final double TRACK_HALF = 0.69 / 2;

    private static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
            // Hardcoded (not Constants.SwerveConstants.SWERVE_CANBUS): loading that class
            // statically constructs the old lib's CANcoders/Pigeon — avoid initializing it.
            .withCANBusName("SwerveCANivore")
            .withPigeon2Id(2);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FACTORY =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                    .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                    .withCouplingGearRatio(0) // unknown for this drivebase — verify on robot
                    .withWheelRadius(Inches.of(WHEEL_RADIUS_INCHES))
                    .withSteerMotorGains(STEER_GAINS)
                    .withDriveMotorGains(DRIVE_GAINS)
                    .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage) // FOC after SysId
                    .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                    .withSlipCurrent(Amps.of(80)) // v1 drive stator limit
                    .withSpeedAt12Volts(MetersPerSecond.of(SPEED_AT_12V_MPS))
                    .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
                    .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
                    .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                    .withSteerInertia(KilogramSquareMeters.of(0.01))   // sim only
                    .withDriveInertia(KilogramSquareMeters.of(0.025)); // sim only

    // v1 module map: steer / drive / CANcoder — FL 22/20/21, FR 12/10/11, BL 32/30/31, BR 42/40/41.
    // Offsets 0: v1 used raw absolute CANcoder angles (offsets burned into the devices).
    private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> module(
            int steerId, int driveId, int encoderId, double locationX, double locationY) {
        return FACTORY.createModuleConstants(
                steerId, driveId, encoderId, Rotations.of(0),
                Meters.of(locationX), Meters.of(locationY),
                false, false, false);
    }

    /** v1 module map (steer/drive/CANcoder): FL 22/20/21, FR 12/10/11, BL 32/30/31, BR 42/40/41. */
    public static final SwerveDrivetrainConstants DRIVETRAIN = DRIVETRAIN_CONSTANTS;
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            FRONT_LEFT = module(22, 20, 21, TRACK_HALF, TRACK_HALF),
            FRONT_RIGHT = module(12, 10, 11, TRACK_HALF, -TRACK_HALF),
            BACK_LEFT = module(32, 30, 31, -TRACK_HALF, TRACK_HALF),
            BACK_RIGHT = module(42, 40, 41, -TRACK_HALF, -TRACK_HALF);

    /** v1 {@code Swerve.turretToRobot}: turret-frame pose → robot-frame pose. */
    public static Pose2d turretPoseToRobotPose(Pose2d turretPose) {
        Rotation2d robotRotation = turretPose.getRotation()
                .minus(TurretOffsetGetter.instance.getTurretOffset());
        Translation2d robotTranslation = turretPose.getTranslation()
                .plus(new Translation2d(-Constants.PhysicalConstants.TURRET_OFFSET_TRANSLATION.getX(), robotRotation));
        return new Pose2d(robotTranslation, robotRotation);
    }
}
