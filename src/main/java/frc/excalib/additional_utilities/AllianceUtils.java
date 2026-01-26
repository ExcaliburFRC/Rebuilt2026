package frc.excalib.additional_utilities;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;

/**
 * the purpose of this class is to supply different utility functions for functionality
 * that depends on the robot alliance.
 *
 * @author Shai Grossman
 */
public class AllianceUtils {
    public static final double FIELD_LENGTH_METERS = 16.54;
    public static final double FIELD_WIDTH_METERS = 8.07;

    /**
     * @return whether the robot is on the blue alliance
     */
    public static boolean isBlueAlliance() {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) return alliance.get().equals(Blue);
        DriverStation.reportError("DS alliance empty!", false);
        return true;
    }

    public static boolean isRedAlliance() {
        return !isBlueAlliance();
    }

    public static int getdirection() {
        if (isBlueAlliance())
            return 1;
        return -1;
    }

    /**
     * Converts a pose to the pose relative to the current driver station alliance.
     *
     * @param bluePose the current blue alliance pose
     * @return the converted pose
     */
    public static Pose3d toAlliancePose(Pose3d bluePose) {
        if (isBlueAlliance()) return bluePose;
        return switchAlliance(bluePose);
    }

    public static Translation3d toAlliancePose(Translation3d bluePose) {
        if (isBlueAlliance()) return bluePose;
        return switchAlliance(bluePose);
    }

    public static Pose3d switchAlliance(Pose3d pose) {
        return new Pose3d(
                FIELD_LENGTH_METERS - pose.getX(),
                FIELD_WIDTH_METERS - pose.getY(),
                pose.getZ(),
                pose.getRotation().minus(new Rotation3d(0, 0, Math.PI))
        );
    }

    public static Translation3d switchAlliance(Translation3d pose) {
        return new Translation3d(
                FIELD_LENGTH_METERS - pose.getX(),
                FIELD_WIDTH_METERS - pose.getY(),
                pose.getZ()
        );
    }

    public static Pose2d mirrorAlliance(Pose2d pose) {
        return new Pose2d(
                FIELD_LENGTH_METERS - pose.getX(),
                pose.getY(),
                new Rotation2d(Math.PI).minus(pose.getRotation())
        );
    }

    public static class AlliancePose {
        private Pose3d pose;

        public AlliancePose(double x, double y, double degrees) {
            this.pose = new Pose3d(new Translation3d(x, y, 0), new Rotation3d(0, 0, degrees));
        }


        public AlliancePose() {
            this.pose = new Pose3d(new Translation3d(), new Rotation3d());
        }

        public AlliancePose(Translation2d translation, Rotation2d rotation) {
            pose = new Pose3d(new Translation3d(translation), new Rotation3d(rotation));
        }

        public AlliancePose(double degrees) {
            this.pose = new Pose3d(new Translation3d(), new Rotation3d(Rotation2d.fromDegrees(degrees)));
        }

        public AlliancePose(Translation3d translation3d, Rotation3d rotation3d) {
            pose = new Pose3d(translation3d, rotation3d);
        }

        public AlliancePose(double x, double y, double z, double yaw, double roll, double pitch) {
            pose = new Pose3d(x, y, z, new Rotation3d(roll, pitch, yaw));
        }

        public AlliancePose(double x, double y, double z, double yaw) {
            pose = new Pose3d(x, y, z, new Rotation3d(0, 0, yaw));
        }

        public Pose3d getAsCurrentAlliance() {
            return toAlliancePose(pose);
        }
    }
}
