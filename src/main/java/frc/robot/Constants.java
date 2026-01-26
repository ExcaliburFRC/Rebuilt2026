// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.imu.IMU;
import frc.excalib.control.imu.Pigeon;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.swerve.ModulesHolder;
import frc.excalib.swerve.Swerve;
import frc.excalib.swerve.SwerveModule;

public final class Constants {
    public static final Pose2d INITIAL_POSE = new Pose2d();
    public static final double PHYSICS_PERIODIC_TIME = 0.02;
    public static final int PRIMARY_CONTROLLER_PORT = 0;

    public static class SwerveConstants {
        public static final int FRONT_LEFT_DRIVE_ID = 20;
        public static final int FRONT_RIGHT_DRIVE_ID = 10;
        public static final int BACK_RIGHT_DRIVE_ID = 40;
        public static final int BACK_LEFT_DRIVE_ID = 30;

        public static final int FRONT_LEFT_ROTATION_ID = 22;
        public static final int FRONT_RIGHT_ROTATION_ID = 12;
        public static final int BACK_RIGHT_ROTATION_ID = 42;
        public static final int BACK_LEFT_ROTATION_ID = 32;

        public static final int GYRO_ID = 3;
        public static final CANBus SWERVE_CANBUS = new CANBus(""); // todo

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

        public static final double MAX_MODULE_VEL = 1;
        public static final double MAX_FRONT_ACC = 1;
        public static final double MAX_SIDE_ACC = 1;
        public static final double MAX_SKID_ACC = 1;
        public static final double MAX_FORWARD_ACC = 1;
        public static final double MAX_VEL = 2.5;
        public static final double MAX_OMEGA_RAD_PER_SEC = 1.5;
        public static final double MAX_OMEGA_RAD_PER_SEC_SQUARE = 1;

        public static final PathConstraints MAX_PATH_CONSTRAINTS = new PathConstraints(
                MAX_VEL,
                MAX_SKID_ACC,
                MAX_OMEGA_RAD_PER_SEC,
                MAX_OMEGA_RAD_PER_SEC_SQUARE,
                12.0,
                false
        );

        public static final CANcoder FRONT_RIGHT_ABS_ENCODER = new CANcoder(11, SWERVE_CANBUS);
        private static final CANcoder FRONT_LEFT_ABS_ENCODER = new CANcoder(21, SWERVE_CANBUS);


        private static final CANcoder BACK_LEFT_ABS_ENCODER = new CANcoder(31, SWERVE_CANBUS);
        private static final CANcoder BACK_RIGHT_ABS_ENCODER = new CANcoder(41, SWERVE_CANBUS);

        private static final double VELOCITY_CONVERSION_FACTOR = Units.inchesToMeters(4) * Math.PI / 5.27;
        private static final double POSITION_CONVERSION_FACTOR = Units.inchesToMeters(4) * Math.PI / 5.27;
        private static final double ROTATION_VELOCITY_CONVERSION_FACTOR = (2 * Math.PI) / (26.09090909090909);

        public static final PIDConstants TRANSLATION_PID_PP_CONSTANTS = new PIDConstants(10.0, 0.0, 0.0); //TODO
        public static final PIDConstants ANGLE_PID_PP_CONSTANTS = new PIDConstants(5.0, 0.0, 0.0);
        public static final Gains ANGLE_PID_GAINS = new Gains();
        public static final Gains TRANSLATION_PID_GAINS = new Gains();

        private static final IMU GYRO = new Pigeon(GYRO_ID, SWERVE_CANBUS.getName(), new Rotation3d());

        public static Swerve configureSwerve(Pose2d initialPose) {
            return new Swerve(
                    new ModulesHolder(
                            new SwerveModule(
                                    new TalonFXMotor(FRONT_LEFT_DRIVE_ID, SWERVE_CANBUS),
                                    new TalonFXMotor(FRONT_LEFT_ROTATION_ID, SWERVE_CANBUS),
                                    new Gains(7.5, 0, 0, 0.192, 0, 0, 0),
                                    new Gains(0, 0, 0, 0.08709, 2.254, 0, 0),
                                    PID_TOLERANCE,
                                    FRONT_LEFT_TRANSLATION,
                                    () -> FRONT_LEFT_ABS_ENCODER.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                                    MAX_MODULE_VEL,
                                    VELOCITY_CONVERSION_FACTOR,
                                    POSITION_CONVERSION_FACTOR,
                                    ROTATION_VELOCITY_CONVERSION_FACTOR
                            ),
                            new SwerveModule(
                                    new TalonFXMotor(FRONT_RIGHT_DRIVE_ID, SWERVE_CANBUS),
                                    new TalonFXMotor(FRONT_RIGHT_ROTATION_ID, SWERVE_CANBUS),
                                    new Gains(7.5, 0, 0, 0.192, 0, 0, 0),
                                    new Gains(0, 0, 0, 0.08709, 2.254, 0, 0),
                                    PID_TOLERANCE,
                                    FRONT_RIGHT_TRANSLATION,
                                    () -> FRONT_RIGHT_ABS_ENCODER.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                                    MAX_MODULE_VEL,
                                    VELOCITY_CONVERSION_FACTOR,
                                    POSITION_CONVERSION_FACTOR,
                                    ROTATION_VELOCITY_CONVERSION_FACTOR
                            ),
                            new SwerveModule(
                                    new TalonFXMotor(BACK_LEFT_DRIVE_ID, SWERVE_CANBUS),
                                    new TalonFXMotor(BACK_LEFT_ROTATION_ID, SWERVE_CANBUS),
                                    new Gains(7.5, 0, 0, 0.192, 0, 0, 0),
                                    new Gains(0, 0, 0, 0.08709, 2.254, 0, 0),
                                    PID_TOLERANCE,
                                    BACK_LEFT_TRANSLATION,
                                    () -> BACK_LEFT_ABS_ENCODER.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                                    MAX_MODULE_VEL,
                                    VELOCITY_CONVERSION_FACTOR,
                                    POSITION_CONVERSION_FACTOR,
                                    ROTATION_VELOCITY_CONVERSION_FACTOR
                            ),
                            new SwerveModule(
                                    new TalonFXMotor(BACK_RIGHT_DRIVE_ID, SWERVE_CANBUS),
                                    new TalonFXMotor(BACK_RIGHT_ROTATION_ID, SWERVE_CANBUS),
                                    new Gains(7.5, 0, 0, 0.192, 0, 0, 0),
                                    new Gains(0, 0, 0, 0.08709, 2.254, 0, 0),
                                    PID_TOLERANCE,
                                    BACK_RIGHT_TRANSLATION,
                                    () -> BACK_RIGHT_ABS_ENCODER.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                                    MAX_MODULE_VEL,
                                    VELOCITY_CONVERSION_FACTOR,
                                    POSITION_CONVERSION_FACTOR,
                                    ROTATION_VELOCITY_CONVERSION_FACTOR
                            )),
                    GYRO,
                    initialPose
            );
        }

    }

    public static class FieldConstants {
        // all the units of length are in meters

        public static final AllianceUtils.AlliancePose BLUE_HUB_CENTER_POSE = new
                AllianceUtils.AlliancePose(5.06, 4.03, 1.83, 0);
        public static final AllianceUtils.AlliancePose DELIVERY_RIGHT_POSE_DIATANCE = new
                AllianceUtils.AlliancePose(1.988, 6.523,0,0);
        public static final AllianceUtils.AlliancePose DELIVERY_LEFT_POSE_DISTANCE = new
                AllianceUtils.AlliancePose(1.988, 2.172,0,0);
        public static final Translation3d BLUE_CLIMB_TOWER_POSE_L1 = new
                Translation3d(1.148, 4.32, 0.6858);
        public static final Translation3d BLUE_CLIMB_TOWER_POSE_L2 = new
                Translation3d(1.148, 4.32, 1.143);
        public static final Translation3d BLUE_CLIMB_TOWER_POSE_L3 = new
                Translation3d(1.148, 4.32, 1.6002);
        public static final Translation2d BLUE_OUTPOST_POSE_CENTER = new
                Translation2d(0,0.63);
        public static final int SHOOTER_TO_TRENCH_LIMET = 100;
        public static final Translation2d BLUE_DOWN_FIELD_TRENCH_POSE = new
                Translation2d(5.06, 0.63);
        public static final Translation2d BLUE_UP_FIELD_TRENCH_POSE = new
                Translation2d(5.06, 7.43);
        public static final Translation2d BLUE_UP_FIELD_PICKUP_FUEL_PLACEMENT = new
                Translation2d(0.34, 6.509);
        public static final Translation2d BLUE_DOWN_FIELD_PICKUP_FUEL_PLACEMENT = new
                Translation2d(0.34, 4.829);

        public static final double FUEL_DIAMETER = 0.15;
        public static final Translation2d BLUE_SOTER_LIMET_OTASE = new
                Translation2d(3,0);
        public static final Translation2d BLUE_SOTER_LIMET_INER = new
                Translation2d(5,0);
        public static final Translation2d REED_SOTER_LIMET_OTASE = new
                Translation2d(10,0);
        public static final Translation2d REED_SOTER_LIMET_INER = new
                Translation2d(13,0);
        public static final Translation2d[] RIGHT_TRENCH_POSES = {new Translation2d(0,0), new Translation2d(0,0),
                new Translation2d(0,0),new Translation2d(0,0)}; //four poses that the robot might need to go through on his way under the trench
        public static final Translation2d[] LEFT_TRENCH_POSES = {new Translation2d(0,0), new Translation2d(0,0),
                new Translation2d(0,0),new Translation2d(0,0)}; //four poses that the robot might need to go through on his way under the trench
    }

    public static final double DEADBAND_X = 0.07;

}