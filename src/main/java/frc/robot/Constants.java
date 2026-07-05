// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.excalib2.util.AllianceFlip;

public final class Constants {
    public static final Pose2d INITIAL_POSE = new Pose2d();

    public static final double PHYSICS_PERIODIC_TIME = 0.02;
    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int PDH_PORT = 1;

    public static final double CONTROLLER_DEADBAND = 0.2;
    public static final boolean DISABLE_SUBSYSTEMS = false;
    public static final boolean DISABLE_SWERVE = false;

    public static final CANBus SUBSYSTEMS_CANBUS = new CANBus("");

    public static class FieldConstants {
        // all the units of length are in meters
        public static final double FIELD_LENGTH_METERS = 16.54;
        public static final double FIELD_WIDTH_METERS = 8.07;

        public static final AllianceFlip.AllianceTranslation BLUE_HUB_CENTER_POSE = new
                AllianceFlip.AllianceTranslation(4.69115, 4.03);
        public static final AllianceFlip.AllianceTranslation DELIVERY_FAR_POSE = new
                AllianceFlip.AllianceTranslation(2, 7.41);
        public static final AllianceFlip.AllianceTranslation DELIVERY_CLOSE_POSE = new
                AllianceFlip.AllianceTranslation(2, 0.63);
        public static final AllianceFlip.AlliancePose NET_END_RIGHT_POSE = new
                AllianceFlip.AlliancePose(4.62, 3.29, 0);
        public static final AllianceFlip.AlliancePose NET_END_LEFT_POSE = new
                AllianceFlip.AlliancePose(4.62, 4.77, 0);
        public static final Translation3d BLUE_CLIMB_TOWER_POSE_L1 = new
                Translation3d(1.05, 3.74, 0.6858);
        public static final Translation3d BLUE_CLIMB_TOWER_POSE_L2 = new
                Translation3d(1.05, 3.74, 1.143);
        public static final Translation3d BLUE_CLIMB_TOWER_POSE_L3 = new
                Translation3d(1.05, 3.74, 1.6002);
        public static final AllianceFlip.AlliancePose BLUE_OUTPOST_POSE_CENTER = new
                AllianceFlip.AlliancePose(0, 0.63, 0);
        public static final AllianceFlip.AlliancePose BLUE_DOWN_FIELD_TRENCH_POSE = new
                AllianceFlip.AlliancePose(4.62, 0.63, 0);
        public static final AllianceFlip.AlliancePose BLUE_UP_FIELD_TRENCH_POSE = new
                AllianceFlip.AlliancePose(4.62, 7.43, 0);
        public static final AllianceFlip.AlliancePose BLUE_UP_FIELD_PICKUP_FUEL_PLACEMENT = new
                AllianceFlip.AlliancePose(0.39, 6.06, 0);

        public static double CLOSE_TRENCH_TO_BUMP_X = 1.65;
        public static double FAR_TRENCH_TO_BUMP_X = FIELD_WIDTH_METERS - CLOSE_TRENCH_TO_BUMP_X;

        public static final double FUEL_DIAMETER = 0.15;

        public static double FRONT_TRENCH_SIDEX_LINE_DIST_METERS = 3.3;
        public static double BACK_TRENCH_SIDEX_LINE_DIST_METERS = 4.6;
        public static double TRENCH_SIDEY_LINE_DIST_METERS = 1.5;

    }

    public static class PhysicalConstants {
        public static final Translation2d TURRET_OFFSET_TRANSLATION = new Translation2d(-0.215, 0); //todo robot to turret
    }
}