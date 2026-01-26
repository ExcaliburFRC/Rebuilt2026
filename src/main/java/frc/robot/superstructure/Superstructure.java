package frc.robot.superstructure;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.swerve.Swerve;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Transport;
import frc.robot.util.ShootingTarget;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.ShooterPhysics;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.subsystems.transport.Constants.SHOOTING_VOLTAGE;
import static frc.robot.util.ShootingTarget.*;

public class Superstructure {
    public final Intake intake;
    public final Shooter shooter;
    public final Transport transport;
    public final Turret turret;
    public final Swerve swerve;

    public final CommandPS5Controller controller;

    public final ShooterPhysics shooterPhysic;
    public Translation3d targetShootingPose = BLUE_HUB_CENTER_POSE.getAsCurrentAlliance().getTranslation();

    public Superstructure(CommandPS5Controller controller, Swerve swerve) {
        intake = new Intake();
        transport = new Transport();

        this.swerve = swerve;

        turret = new Turret(swerve::getApproximatedFuturePose2D);

        shooter = new Shooter(() -> swerve.getPose2D().getTranslation());
        this.controller = controller;

        shooterPhysic = new ShooterPhysics(
                swerve::getApproximatedFuturePose2D,
                () -> swerve.getVelocity().getX(),
                () -> swerve.getRobotRelativeSpeeds().omegaRadiansPerSecond
        );

    }

    private Command shootingCommand() {
        return new ParallelCommandGroup(
                shooter.adjustShooterForShootingCommand(
                        shooterPhysic::getHoodAngleRadSolution,
                        shooterPhysic::getRollerRadPerSecSolution
                ),
                shooter.getFuelCommand(),
                transport.manualCommand(() -> SHOOTING_VOLTAGE)
        );
    }

    public Command shootToHubCommand() {
        return new SequentialCommandGroup(
                turret.setTargetCommand(ShootingTarget.HUB),
                shootingCommand());
    }

    public Command shootForDeliveryCommand() {

        double distanceToDeliveryForRight =
                DELIVERY_RIGHT_POSE_DIATANCE.getAsCurrentAlliance().getTranslation().toTranslation2d().getDistance(swerve.getPose2D().getTranslation());
        double distanceToDeliveryForLeft =
                DELIVERY_LEFT_POSE_DISTANCE.getAsCurrentAlliance().getTranslation().toTranslation2d().getDistance(swerve.getPose2D().getTranslation());

        if (distanceToDeliveryForLeft > distanceToDeliveryForRight) {
            return new InstantCommand(() -> RobotContainer.shootingTarget = LEFT_DELIVERY).andThen(turret.setTargetCommand(LEFT_DELIVERY));

        }
        return new InstantCommand(() -> RobotContainer.shootingTarget = RIGHT_DELIVERY).andThen(turret.setTargetCommand(RIGHT_DELIVERY));
    }

    public Command ultimateShootingCommand() {
        return new ConditionalCommand(
                shootToHubCommand(),
                shootForDeliveryCommand(),
                () -> {
                    boolean condition = swerve.getPose2D().getX() < BLUE_HUB_CENTER_POSE.getAsCurrentAlliance().getX();
                    return AllianceUtils.isBlueAlliance() ? condition : !condition;
                }
        );
    }

    public Command openIntakeCommand() {
        return intake.openIntakeCommand();
    }

    public Command driveUnderTrenchCommand() {
        //closest to right side or left side check
        Translation2d[] TrenchPoses;
        if(swerve.getPose2D().getTranslation().getDistance(RIGHT_TRENCH_POSES[0])
                > swerve.getPose2D().getTranslation().getDistance(LEFT_TRENCH_POSES[0])){
            TrenchPoses = RIGHT_TRENCH_POSES; //todo is it aliasing? and if it is, is it OK?
        }else{
            TrenchPoses = LEFT_TRENCH_POSES; //todo is it aliasing? and if it is, is it OK?
        }

        //finds closest target pose
        Translation2d TargetPose = TrenchPoses[0]; // closest pose to robot by x-value
        int targetInd = 0; //index of target pose in TrenchPoses array
        for(int i = 0; i < TrenchPoses.length; i++){
            if(swerve.getPose2D().getTranslation().getX() < TargetPose.getX()){
                TargetPose = swerve.getPose2D().getTranslation();
            }
            targetInd = i;
        }

        //creates path array that includes only the poses we want and fills in the empty slots in the array.
        Pose2d[] path = new Pose2d[TrenchPoses.length]; //path's type is Pose2d for driveToPoseCommand
        for(int i = 0; i < path.length; i++){
            if(i <= targetInd){
                path[i] = new Pose2d(TargetPose, new Rotation2d()); //TargetPose here is just a default value to fill in the empty spaces of the array.
            }else{
                path[i] = new Pose2d(TrenchPoses[i], new Rotation2d());
            }
        }


        return new SequentialCommandGroup(swerve.driveToPoseCommand(path[0]),
                swerve.driveToPoseCommand(path[1]),
                swerve.driveToPoseCommand(path[2]),
                swerve.driveToPoseCommand(path[3])); //number of drive commands should be equal to length of TrenchPoses


//        double robotYPosition = swerve.getPose2D().getY();
//
//        if (swerve.getPose2D().getTranslation().getDistance(BLUE_DOWN_FIELD_TRENCH_POSE) >
//                swerve.getPose2D().getTranslation().getDistance(BLUE_UP_FIELD_TRENCH_POSE)) {
//            return swerve.driveToPoseCommand(new Pose2d(BLUE_DOWN_FIELD_TRENCH_POSE, new Rotation2d()));
//        } else {
//            return swerve.driveToPoseCommand(new Pose2d(BLUE_UP_FIELD_TRENCH_POSE, new Rotation2d()));
//        }
    }
}
