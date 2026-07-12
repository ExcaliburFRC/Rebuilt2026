package frc.excalib.swerve.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;

import java.util.Optional;

/**
 * Minimal, self-contained Limelight <b>MegaTag2</b> client (NetworkTables only — no
 * dependency on the full {@code LimelightHelpers}).
 *
 * <p>MegaTag2 solves the tag translation given an externally supplied heading, which makes
 * the pose far more robust to tag ambiguity than MegaTag1. Contract: call
 * {@link #setRobotOrientation} every loop with the best current heading of the camera
 * platform, then consume {@link #getEstimate()} and feed it to the pose estimator with
 * {@link #stdDevs} scaling.
 */
public class LimelightMegaTag2 {
    /** One MT2 pose estimate. {@code timestampSeconds} is in the FPGA timebase. */
    public record Estimate(Pose2d pose, double timestampSeconds, int tagCount, double avgTagDistanceMeters) {
    }

    private final DoubleArrayPublisher orientationPublisher;
    private final DoubleArraySubscriber botposeSubscriber;
    private long lastProcessedTimestamp = 0;

    /** @param limelightName NT table name, e.g. {@code "limelight-turret"} */
    public LimelightMegaTag2(String limelightName) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
        botposeSubscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
    }

    /**
     * Publishes the heading MegaTag2 needs. For a turret-mounted camera, pass the
     * <b>camera platform's</b> field heading (robot heading + turret offset).
     */
    public void setRobotOrientation(Rotation2d yaw, double yawRateDegreesPerSecond) {
        orientationPublisher.set(new double[]{yaw.getDegrees(), yawRateDegreesPerSecond, 0, 0, 0, 0});
        NetworkTableInstance.getDefault().flush();
    }

    /** Latest unconsumed MT2 estimate, if a new one with at least one tag exists. */
    public Optional<Estimate> getEstimate() {
        TimestampedDoubleArray sample = botposeSubscriber.getAtomic();
        if (sample.timestamp == lastProcessedTimestamp || sample.value.length < 11) {
            return Optional.empty();
        }
        lastProcessedTimestamp = sample.timestamp;

        double[] data = sample.value;
        int tagCount = (int) data[7];
        if (tagCount < 1) {
            return Optional.empty();
        }
        Pose2d pose = new Pose2d(data[0], data[1], Rotation2d.fromDegrees(data[5]));
        double latencySeconds = data[6] / 1000.0;
        // NT sample time (microseconds, local clock) minus pipeline+capture latency
        double timestampSeconds = sample.timestamp / 1e6 - latencySeconds;
        return Optional.of(new Estimate(pose, timestampSeconds, tagCount, data[9]));
    }

    /**
     * Vision std-devs scaled by distance and tag count: trust falls off with the square of
     * distance and improves with multiple tags. MT2 heading is not trusted at all (the
     * estimator's gyro heading wins).
     */
    public static Matrix<N3, N1> stdDevs(Estimate estimate) {
        double base = 0.4 * (1.0 + estimate.avgTagDistanceMeters() * estimate.avgTagDistanceMeters() / 30.0);
        double xy = estimate.tagCount() >= 2 ? base / 2.0 : base;
        return VecBuilder.fill(xy, xy, Double.MAX_VALUE);
    }
}
