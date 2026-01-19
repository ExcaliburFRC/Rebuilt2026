package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        shooter = new Shooter();
        transport = new Transport();
        swerve = Constants.SwerveConstants.configureSwerve(Constants.initialPose);
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
}
