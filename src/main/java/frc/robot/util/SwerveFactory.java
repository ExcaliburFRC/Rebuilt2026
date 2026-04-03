package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.excalib.swerve.*;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.CANcoder;

import java.util.HashMap;
import java.util.Map;

import static frc.robot.Constants.SwerveConstants.*;

public class SwerveFactory {

    private static final Map<Integer, CANcoder> encoderCache = new HashMap<>();

    private static CANcoder getOrCreateEncoder(int id, String bus) {
        return encoderCache.computeIfAbsent(id, k -> new CANcoder(id, bus));
    }

    public static Swerve createSwerve(Pose2d initialPose) {
        SwerveModuleIO flIO, frIO, blIO, brIO;
        GyroIO gyroIO;

        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            flIO = new SwerveModuleIOSim(FRONT_LEFT_DRIVE_ID, FRONT_LEFT_ROTATION_ID);
            frIO = new SwerveModuleIOSim(FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_ROTATION_ID);
            blIO = new SwerveModuleIOSim(BACK_LEFT_DRIVE_ID, BACK_LEFT_ROTATION_ID);
            brIO = new SwerveModuleIOSim(BACK_RIGHT_DRIVE_ID, BACK_RIGHT_ROTATION_ID);

            SwerveModule fl = createModule(flIO, 0, FRONT_LEFT_TRANSLATION);
            SwerveModule fr = createModule(frIO, 1, FRONT_RIGHT_TRANSLATION);
            SwerveModule bl = createModule(blIO, 2, BACK_LEFT_TRANSLATION);
            SwerveModule br = createModule(brIO, 3, BACK_RIGHT_TRANSLATION);

            ModulesHolder modules = new ModulesHolder(fl, fr, bl, br);

            // Create simulated gyro that integrates omega from the module states
            gyroIO = new GyroIOSim(
                modules.getSwerveDriveKinematics(),
                modules::logStates
            );

            return new Swerve(modules, gyroIO, initialPose);
        } else {
            CANcoder flEnc = getOrCreateEncoder(FRONT_LEFT_ABS_ENCODER_ID, SWERVE_CANBUS_NAME);
            CANcoder frEnc = getOrCreateEncoder(FRONT_RIGHT_ABS_ENCODER_ID, SWERVE_CANBUS_NAME);
            CANcoder blEnc = getOrCreateEncoder(BACK_LEFT_ABS_ENCODER_ID, SWERVE_CANBUS_NAME);
            CANcoder brEnc = getOrCreateEncoder(BACK_RIGHT_ABS_ENCODER_ID, SWERVE_CANBUS_NAME);

            flIO = new SwerveModuleIOTalonFX(FRONT_LEFT_DRIVE_ID, FRONT_LEFT_ROTATION_ID, SWERVE_CANBUS, () -> flEnc.getAbsolutePosition().getValueAsDouble());
            frIO = new SwerveModuleIOTalonFX(FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_ROTATION_ID, SWERVE_CANBUS, () -> frEnc.getAbsolutePosition().getValueAsDouble());
            blIO = new SwerveModuleIOTalonFX(BACK_LEFT_DRIVE_ID, BACK_LEFT_ROTATION_ID, SWERVE_CANBUS, () -> blEnc.getAbsolutePosition().getValueAsDouble());
            brIO = new SwerveModuleIOTalonFX(BACK_RIGHT_DRIVE_ID, BACK_RIGHT_ROTATION_ID, SWERVE_CANBUS, () -> brEnc.getAbsolutePosition().getValueAsDouble());
            
            // TODO: Wire up real Pigeon2 GyroIO here
            gyroIO = new GyroIO() {}; 

            SwerveModule fl = createModule(flIO, 0, FRONT_LEFT_TRANSLATION);
            SwerveModule fr = createModule(frIO, 1, FRONT_RIGHT_TRANSLATION);
            SwerveModule bl = createModule(blIO, 2, BACK_LEFT_TRANSLATION);
            SwerveModule br = createModule(brIO, 3, BACK_RIGHT_TRANSLATION);

            ModulesHolder modules = new ModulesHolder(fl, fr, bl, br);
            return new Swerve(modules, gyroIO, initialPose);
        }
    }

    private static SwerveModule createModule(SwerveModuleIO io, int index, Translation2d location) {
        return new SwerveModule(
            io, 
            index, 
            Constants.SwerveConstants.ANGLE_PID_GAINS, 
            Constants.SwerveConstants.TRANSLATION_PID_GAINS, 
            0.01, // PID Tolerance
            location, 
            Constants.SwerveConstants.MAX_VEL, 
            Constants.SwerveConstants.VELOCITY_CONVERSION_FACTOR, 
            Constants.SwerveConstants.POSITION_CONVERSION_FACTOR, 
            Constants.SwerveConstants.ROTATION_VELOCITY_CONVERSION_FACTOR, 
            false // Invert
        );
    }
}
