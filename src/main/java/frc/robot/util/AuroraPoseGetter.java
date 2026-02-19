package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import monologue.Annotations.Log.NT;
import monologue.Logged;

public class AuroraPoseGetter implements Logged {
    private static final NetworkTable networkTableInstance = NetworkTableInstance.getDefault().getTable("Aurora").getSubTable("robotPose");

    public static Pose3d visionPose;

    public static void periodic() {
        visionPose = new Pose3d(
                new Translation3d(
                        networkTableInstance.getEntry("x").getDouble(0),
                        networkTableInstance.getEntry("y").getDouble(0),
                        networkTableInstance.getEntry("z").getDouble(0)
                ),
                new Rotation3d(
                        networkTableInstance.getEntry("roll").getDouble(0),
                        networkTableInstance.getEntry("pitch").getDouble(0),
                        networkTableInstance.getEntry("yaw").getDouble(0)
                )
        );
    }

    @NT
    public static Pose2d getPose2d() {
        return visionPose.toPose2d();
    }

    public static Pose3d getPose3d() {
        return visionPose;
    }
}
