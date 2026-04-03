// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.control.gains.Gains;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    public static final Pose2d INITIAL_POSE = new Pose2d();

    /** Selects the robot operating mode for AdvantageKit IO layer selection. */
    public enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }

    public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

    public static final double PHYSICS_PERIODIC_TIME = 0.02;
    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int PDH_PORT = 1;

    public static final double CONTROLLER_DEADBAND = 0.09;
    public static final com.ctre.phoenix6.CANBus SUBSYSTEMS_CANBUS = new com.ctre.phoenix6.CANBus("");

    public static class SwerveConstants {
        public static final int FRONT_RIGHT_DRIVE_ID = 10;
        public static final int FRONT_LEFT_DRIVE_ID = 20;
        public static final int BACK_LEFT_DRIVE_ID = 30;
        public static final int BACK_RIGHT_DRIVE_ID = 40;

        public static final int FRONT_RIGHT_ROTATION_ID = 12;
        public static final int FRONT_LEFT_ROTATION_ID = 22;
        public static final int BACK_LEFT_ROTATION_ID = 32;
        public static final int BACK_RIGHT_ROTATION_ID = 42;

        public static final int GYRO_ID = 2;
        public static final String SWERVE_CANBUS_NAME = "SwerveCANivore";
        public static final com.ctre.phoenix6.CANBus SWERVE_CANBUS = new com.ctre.phoenix6.CANBus(SWERVE_CANBUS_NAME);

        private static final double PID_TOLERANCE = 0.01;

        public static final double TRACK_WIDTH = 0.69; // m

        public static final Translation2d FRONT_LEFT_TRANSLATION =
                new Translation2d(
                        TRACK_WIDTH / 2, TRACK_WIDTH / 2
                );
        public static final Translation2d FRONT_RIGHT_TRANSLATION =
                new Translation2d(
                        TRACK_WIDTH / 2, -TRACK_WIDTH / 2
                );
        public static final Translation2d BACK_LEFT_TRANSLATION =
                new Translation2d(
                        -TRACK_WIDTH / 2, TRACK_WIDTH / 2
                );
        public static final Translation2d BACK_RIGHT_TRANSLATION =
                new Translation2d(
                        -TRACK_WIDTH / 2, -TRACK_WIDTH / 2
                );

        public static final double WHEEL_RADIUS_METERS = 0.0508;
        public static final double PIGEON_OFFSET = 0;
        public static final double MAX_MODULE_VEL = 4;
        public static final double MAX_VEL = 5;

        public static final double MAX_FRONT_ACC = 1;
        public static final double MAX_SIDE_ACC = 1;
        public static final double MAX_SKID_ACC = 1;
        public static final double MAX_FORWARD_ACC = 1;
        public static final double MAX_OMEGA_RAD_PER_SEC = 11.5;
        public static final double MAX_OMEGA_RAD_PER_SEC_SQUARE = 1;

        public static final PathConstraints MAX_PATH_CONSTRAINTS = new PathConstraints(
                MAX_VEL,
                MAX_SKID_ACC,
                MAX_OMEGA_RAD_PER_SEC,
                MAX_OMEGA_RAD_PER_SEC_SQUARE,
                12.0,
                false
        );

        public static final int FRONT_RIGHT_ABS_ENCODER_ID = 11;
        public static final int FRONT_LEFT_ABS_ENCODER_ID = 21;
        public static final int BACK_LEFT_ABS_ENCODER_ID = 31;
        public static final int BACK_RIGHT_ABS_ENCODER_ID = 41;

        public static final double VELOCITY_CONVERSION_FACTOR = Units.inchesToMeters(4) * Math.PI / 5.27;
        public static final double POSITION_CONVERSION_FACTOR = Units.inchesToMeters(4) * Math.PI / 5.27;
        public static final double ROTATION_VELOCITY_CONVERSION_FACTOR = (2 * Math.PI) / (26.09090909090909);

        public static final com.pathplanner.lib.config.PIDConstants TRANSLATION_PID_PP_CONSTANTS = new com.pathplanner.lib.config.PIDConstants(10.0, 0.0, 0.0);
        public static final com.pathplanner.lib.config.PIDConstants ANGLE_PID_PP_CONSTANTS = new com.pathplanner.lib.config.PIDConstants(5.0, 0.0, 0.0);
        // Gains tuned for simulation with correct unit conversion chain
        // ANGLE: Kp controls how fast modules steer to target angle (rad → V)
        // TRANSLATION: Kp corrects velocity error, Kv provides feedforward (m/s → V)
        //   Kv ≈ 12V / max_speed ≈ 12 / 6.0 ≈ 2.0
        public static final Gains ANGLE_PID_GAINS = new Gains(3.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0);
        public static final Gains TRANSLATION_PID_GAINS = new Gains(0.8, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0);
    }


    public static class FieldConstants {
        // all the units of length are in meters
        public static final AllianceUtils.AlliancePose BLUE_HUB_CENTER_POSE = new
                AllianceUtils.AlliancePose(4.69115, 4.03, 0);
        public static final AllianceUtils.AlliancePose DELIVERY_FAR_POSE = new
                AllianceUtils.AlliancePose(2, 7.41, 0);
        public static final AllianceUtils.AlliancePose DELIVERY_CLOSE_POSE = new
                AllianceUtils.AlliancePose(2, 0.63, 0);
        public static final AllianceUtils.AlliancePose NET_END_RIGHT_POSE = new
                AllianceUtils.AlliancePose(4.62, 3.29, 0);
        public static final AllianceUtils.AlliancePose NET_END_LEFT_POSE = new
                AllianceUtils.AlliancePose(4.62, 4.77, 0);
        public static final Translation3d BLUE_CLIMB_TOWER_POSE_L1 = new
                Translation3d(1.05, 3.74, 0.6858);
        public static final Translation3d BLUE_CLIMB_TOWER_POSE_L2 = new
                Translation3d(1.05, 3.74, 1.143);
        public static final Translation3d BLUE_CLIMB_TOWER_POSE_L3 = new
                Translation3d(1.05, 3.74, 1.6002);
        public static final AllianceUtils.AlliancePose BLUE_OUTPOST_POSE_CENTER = new
                AllianceUtils.AlliancePose(0, 0.63, 0);
        public static final AllianceUtils.AlliancePose BLUE_DOWN_FIELD_TRENCH_POSE = new
                AllianceUtils.AlliancePose(4.62, 0.63, 0);
        public static final AllianceUtils.AlliancePose BLUE_UP_FIELD_TRENCH_POSE = new
                AllianceUtils.AlliancePose(4.62, 7.43, 0);
        public static final AllianceUtils.AlliancePose BLUE_UP_FIELD_PICKUP_FUEL_PLACEMENT = new
                AllianceUtils.AlliancePose(0.39, 6.06, 0);

        public static double CLOSE_TRENCH_TO_BUMP_X = 1.65;
        public static double FAR_TRENCH_TO_BUMP_X = AllianceUtils.FIELD_WIDTH_METERS - CLOSE_TRENCH_TO_BUMP_X;

        public static final double FUEL_DIAMETER = 0.15;

        public static final double FRONT_TRENCH_SIDEX_LINE_DIST_METERS = 3.9624;
        public static final double BACK_TRENCH_SIDEX_LINE_DIST_METERS = 5.1;
        public static final double TRENCH_SIDEY_LINE_DIST_METERS = 1.5;

    }

    public static class PhysicalConstants {
        public static final Translation2d TURRET_OFFSET_TRANSLATION = new Translation2d(-0.215, 0); //todo robot to turret
    }
}