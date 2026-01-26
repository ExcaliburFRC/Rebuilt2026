package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.additional_utilities.PS5Controller;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Transport;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.turret.ShootingTargets;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.subsystems.transport.Constants.SHOOTING_VOLTAGE;
import static frc.robot.subsystems.turret.ShootingTargets.*;

public class Superstructure {
    public final Intake intake;
    public final Shooter shooter;
    public final Transport transport;
    public final Swerve swerve;
    public final PS5Controller controller;
    public final Turret turret;

    public Superstructure(PS5Controller controller) {
        intake = new Intake();
        transport = new Transport();
        swerve = Constants.SwerveConstants.configureSwerve(Constants.initialPose);
        shooter = new Shooter(()-> swerve.getPose2D().getTranslation());
        turret = new Turret(swerve::getPose2D);
        this.controller = controller;
    }

    private Command shootingCommand(Translation2d target) {
        return new ParallelCommandGroup(
                shooter.adjustShooterForShootingCommand(
                        () -> swerve.getPose2D().getTranslation(),
                        () -> new Pose3d(
                                new Translation3d(target),
                                new Rotation3d()
                        )),

                shooter.getFuelCommand(),
                transport.manualCommand(() -> SHOOTING_VOLTAGE)
        );
    }

    public Command shootToHubCommand() {
        return new SequentialCommandGroup(
                turret.setTargetCommand(ShootingTargets.HUB),
                shootingCommand(FieldConstants.BLUE_HUB_CENTER_POSE.getAsCurrentAlliance().getTranslation()));
    }

    public Command shootForDeliveryCommand() {
        Translation2d deliveryPoseOption;

        double distanceToDeliveryForRight =
                DELIVERY_RIGHT_POSE_DIATANCE.getAsCurrentAlliance().getTranslation().getDistance(swerve.getPose2D().getTranslation());
        double distanceToDeliveryForLeft =
                DELIVERY_LEFT_POSE_DISTANCE.getAsCurrentAlliance().getTranslation().getDistance(swerve.getPose2D().getTranslation());


        if (distanceToDeliveryForLeft > distanceToDeliveryForRight) {
            deliveryPoseOption = DELIVERY_LEFT_POSE_DISTANCE.getAsCurrentAlliance().getTranslation();
            CommandScheduler.getInstance().schedule(turret.setTargetCommand(LEFT_DELIVERY));
        } else {
            deliveryPoseOption = DELIVERY_RIGHT_POSE_DIATANCE.getAsCurrentAlliance().getTranslation();
            CommandScheduler.getInstance().schedule(turret.setTargetCommand(RIGHT_DELIVERY));
        }

        return shootingCommand(deliveryPoseOption);
    }

    public Command ultimateShootingCommand() {
        return new ConditionalCommand(
                shootToHubCommand(),
                shootForDeliveryCommand(),
                () -> {
                    boolean condition = swerve.getPose2D().getX() < BLUE_HUB_CENTER_POSE.getAsCurrentAlliance().getX();
                    return AllianceUtils.isBlueAlliance()? condition: !condition;
                }
        );
    }

    public Command openIntakeCommand(){
        return intake.openIntakeCommand();
    }

    public Command driveToClosesTrenchCommand(){
        double robotYPosition = swerve.getPose2D().getY();

        if (swerve.getPose2D().getTranslation().getDistance(BLUE_DOWN_FIELD_TRENCH_POSE) >
                swerve.getPose2D().getTranslation().getDistance(BLUE_UP_FIELD_TRENCH_POSE)){
            return swerve.driveToPoseCommand(new Pose2d(BLUE_DOWN_FIELD_TRENCH_POSE, new Rotation2d()));
        } else {
            return swerve.driveToPoseCommand(new Pose2d(BLUE_UP_FIELD_TRENCH_POSE, new Rotation2d()));
        }
    }
}
