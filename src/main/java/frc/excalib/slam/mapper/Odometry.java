package frc.excalib.slam.mapper;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.function.Supplier;

public class Odometry extends SwerveDrivePoseEstimator {
    private final Supplier<Rotation2d> m_YAW_SUPPLIER;
    private Pose2d m_robotPose;
    private double lastTimeStamp;

    public Odometry(
            SwerveDriveKinematics swerveDrive,
            SwerveModulePosition[] modulesPositions,
            Supplier<Rotation2d> angleSupplier,
            Pose2d initialPose) {
        super(swerveDrive, angleSupplier.get(), modulesPositions, initialPose);
        m_YAW_SUPPLIER = angleSupplier;
        m_robotPose = initialPose;
    }

    public Pose2d getRobotPose() {
        return m_robotPose;
    }

    public void updateOdometry(SwerveModulePosition[] modulesPositions) {
        m_robotPose = super.update(
                m_YAW_SUPPLIER.get(),
                modulesPositions
        );
    }

    public void resetOdometry(SwerveModulePosition[] modulesPositions, Pose2d newInitialPose) {
        super.resetPosition(m_YAW_SUPPLIER.get(), modulesPositions, newInitialPose);
    }
    @Override
    public void addVisionMeasurement(Pose2d visionPose, double timeStamp){
        this.lastTimeStamp = timeStamp;
        super.addVisionMeasurement(visionPose, timeStamp);
    }

    /** Vision fusion with explicit per-measurement standard deviations (x, y, theta). */
    public void addVisionMeasurement(Pose2d visionPose, double timeStamp, Matrix<N3, N1> stdDevs){
        this.lastTimeStamp = timeStamp;
        super.addVisionMeasurement(visionPose, timeStamp, stdDevs);
    }
    public double getLastTimeStamp(){
        return this.lastTimeStamp;
    }
}
