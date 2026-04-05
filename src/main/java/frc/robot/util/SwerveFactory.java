package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Pose2d;
import frc.excalib.swerve.*;
import frc.robot.Constants;

import java.util.HashMap;
import java.util.Map;

import static frc.robot.Constants.SwerveConstants.*;

/**
 * Factory for constructing the {@link Swerve} drivetrain.
 *
 * <p>All module-level configuration is centralised in a single
 * {@link SwerveModule.SwerveModuleConfig} record, making it easy to see
 * and adjust module parameters in one place.
 */
public class SwerveFactory {

    private static final Map<Integer, CANcoder> encoderCache = new HashMap<>();

    // Shared module config — all four modules use identical gains and conversion factors.
    private static final SwerveModule.SwerveModuleConfig MODULE_CONFIG = new SwerveModule.SwerveModuleConfig(
            ANGLE_PID_GAINS,
            TRANSLATION_PID_GAINS,
            0.01,                   // PID tolerance (rad)
            MAX_VEL,
            VELOCITY_CONVERSION_FACTOR,
            POSITION_CONVERSION_FACTOR,
            ROTATION_VELOCITY_CONVERSION_FACTOR,
            false                   // invert drive wheel
    );

    public static Swerve createSwerve(Pose2d initialPose) {
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            return createSim(initialPose);
        } else {
            return createReal(initialPose);
        }
    }

    // ─── Sim ─────────────────────────────────────────────────────────────────

    private static Swerve createSim(Pose2d initialPose) {
        SwerveModuleIO flIO = new SwerveModuleIOSim(FRONT_LEFT_DRIVE_ID,  FRONT_LEFT_ROTATION_ID);
        SwerveModuleIO frIO = new SwerveModuleIOSim(FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_ROTATION_ID);
        SwerveModuleIO blIO = new SwerveModuleIOSim(BACK_LEFT_DRIVE_ID,   BACK_LEFT_ROTATION_ID);
        SwerveModuleIO brIO = new SwerveModuleIOSim(BACK_RIGHT_DRIVE_ID,  BACK_RIGHT_ROTATION_ID);

        SwerveModule fl = new SwerveModule(flIO, 0, FRONT_LEFT_TRANSLATION,  MODULE_CONFIG);
        SwerveModule fr = new SwerveModule(frIO, 1, FRONT_RIGHT_TRANSLATION, MODULE_CONFIG);
        SwerveModule bl = new SwerveModule(blIO, 2, BACK_LEFT_TRANSLATION,   MODULE_CONFIG);
        SwerveModule br = new SwerveModule(brIO, 3, BACK_RIGHT_TRANSLATION,  MODULE_CONFIG);

        ModulesHolder modules = new ModulesHolder(fl, fr, bl, br);

        GyroIO gyroIO = new GyroIOSim(modules.getSwerveDriveKinematics(), modules::logStates);
        return new Swerve(modules, gyroIO, initialPose);
    }

    // ─── Real ─────────────────────────────────────────────────────────────────

    private static Swerve createReal(Pose2d initialPose) {
        CANcoder flEnc = getOrCreateEncoder(FRONT_LEFT_ABS_ENCODER_ID,  SWERVE_CANBUS);
        CANcoder frEnc = getOrCreateEncoder(FRONT_RIGHT_ABS_ENCODER_ID, SWERVE_CANBUS);
        CANcoder blEnc = getOrCreateEncoder(BACK_LEFT_ABS_ENCODER_ID,   SWERVE_CANBUS);
        CANcoder brEnc = getOrCreateEncoder(BACK_RIGHT_ABS_ENCODER_ID,  SWERVE_CANBUS);

        SwerveModuleIO flIO = new SwerveModuleIOTalonFX(FRONT_LEFT_DRIVE_ID,  FRONT_LEFT_ROTATION_ID,  SWERVE_CANBUS, () -> flEnc.getAbsolutePosition().getValueAsDouble());
        SwerveModuleIO frIO = new SwerveModuleIOTalonFX(FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_ROTATION_ID, SWERVE_CANBUS, () -> frEnc.getAbsolutePosition().getValueAsDouble());
        SwerveModuleIO blIO = new SwerveModuleIOTalonFX(BACK_LEFT_DRIVE_ID,   BACK_LEFT_ROTATION_ID,   SWERVE_CANBUS, () -> blEnc.getAbsolutePosition().getValueAsDouble());
        SwerveModuleIO brIO = new SwerveModuleIOTalonFX(BACK_RIGHT_DRIVE_ID,  BACK_RIGHT_ROTATION_ID,  SWERVE_CANBUS, () -> brEnc.getAbsolutePosition().getValueAsDouble());

        SwerveModule fl = new SwerveModule(flIO, 0, FRONT_LEFT_TRANSLATION,  MODULE_CONFIG);
        SwerveModule fr = new SwerveModule(frIO, 1, FRONT_RIGHT_TRANSLATION, MODULE_CONFIG);
        SwerveModule bl = new SwerveModule(blIO, 2, BACK_LEFT_TRANSLATION,   MODULE_CONFIG);
        SwerveModule br = new SwerveModule(brIO, 3, BACK_RIGHT_TRANSLATION,  MODULE_CONFIG);

        ModulesHolder modules = new ModulesHolder(fl, fr, bl, br);

        // TODO: Wire up real Pigeon2 GyroIO here
        GyroIO gyroIO = new GyroIO() {};
        return new Swerve(modules, gyroIO, initialPose);
    }

    private static CANcoder getOrCreateEncoder(int id, CANBus bus) {
        return encoderCache.computeIfAbsent(id, k -> new CANcoder(id, bus));
    }
}
