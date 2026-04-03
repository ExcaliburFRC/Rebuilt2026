package frc.excalib.swerve;

import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.excalib.slam.mapper.Odometry;

public class OdometryTest {

    @Test
    public void testStraightOdometryIntegration() {
        // Define a simple square swerve kinematics
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(0.5, 0.5),   // FL
            new Translation2d(0.5, -0.5),  // FR
            new Translation2d(-0.5, 0.5),  // BL
            new Translation2d(-0.5, -0.5)  // BR
        );

        // Initial module positions
        SwerveModulePosition[] positions = new SwerveModulePosition[] {
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0))
        };

        // Initialize Odometry at origin
        Odometry odometry = new Odometry(
            kinematics, 
            positions, 
            () -> new Rotation2d(0.0), 
            new Pose2d()
        );

        System.out.println("TEST: Initial Pose: " + odometry.getRobotPose());

        // Simulate moving each module 1 meter forward
        SwerveModulePosition[] newPositions = new SwerveModulePosition[] {
            new SwerveModulePosition(1.0, new Rotation2d(0.0)),
            new SwerveModulePosition(1.0, new Rotation2d(0.0)),
            new SwerveModulePosition(1.0, new Rotation2d(0.0)),
            new SwerveModulePosition(1.0, new Rotation2d(0.0))
        };

        // Update odometry
        odometry.updateOdometry(newPositions);
        
        Pose2d finalPose = odometry.getRobotPose();
        System.out.println("TEST: Final Pose: " + finalPose);

        // Robot should have moved forward roughly 1 meter in X
        assertTrue(finalPose.getX() > 0.9, "Robot should have moved > 0.9m in X");
        assertTrue(finalPose.getX() < 1.1, "Robot should have moved < 1.1m in X");
    }
}
