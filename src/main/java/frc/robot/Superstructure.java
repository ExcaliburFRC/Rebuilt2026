package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.excalib.additional_utilities.PS5Controller;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Transport;
import frc.robot.Constants.FieldConstants;

import static frc.robot.subsystems.transport.Constants.SHOOTING_VOLTAGE;

public class Superstructure {
    public final Intake intake;
    public final Shooter shooter;
    public final Transport transport;
    public final Swerve swerve;
    public final PS5Controller controller;

    public Superstructure(PS5Controller controller) {
        intake = new Intake();
        shooter = new Shooter();
        transport = new Transport();
        swerve = Constants.SwerveConstants.configureSwerve(new Pose2d());

        this.controller = controller;
    }

    public Command shootToHubCommand() {
        Rotation2d targetAngle = new Rotation2d(
                Math.atan2(FieldConstants.BLUE_HUB_CENTER_POSE.get().getY() - swerve.getPose2D().getY(),
                        swerve.getPose2D().getX() - FieldConstants.BLUE_HUB_CENTER_POSE.get().getX())
        );

        Command shootToHubCommand = new ParallelCommandGroup(
                swerve.turnToAngleCommand(
//                        () -> new Vector2D(
//                                -controller.getLeftY() * MAX_VEL,
//                                -controller.getLeftX() * MAX_VEL // todo: for shooting while in vel
//                        ),
                        () -> new Vector2D(0, 0),
                        () -> targetAngle.minus(Rotation2d.kPi)),

                shooter.adjustShooterForShootingCommand(
                        () -> swerve.getPose2D().getTranslation(),
                        () -> new Pose3d(
                                new Translation3d(Constants.FieldConstants.BLUE_HUB_CENTER_POSE.get().getTranslation()),
                                new Rotation3d()
                        )),

                shooter.getFuelCommand(),
                transport.manualCommand(() -> SHOOTING_VOLTAGE)
        );

        shootToHubCommand.addRequirements(shooter);
        return shootToHubCommand;
    }

    public Command shootForDeliveryCommand(Translation2d current_position){
        Rotation2d targetAngle;
        double distenceToDeliveryForRight =
                FieldConstants.BLUE_RIGHT_BLUE_SIDE_FOR_DELIVERY.get().getTranslation().getDistance(current_position);
        double distenceToDeliveryForLeft =
                FieldConstants.BLUE_LEFT_BLUE_SIDE_FOR_DELIVERY.get().getTranslation().getDistance(current_position);
        if (distenceToDeliveryForLeft > distenceToDeliveryForRight) {
            targetAngle = new Rotation2d(
                    Math.atan2(FieldConstants.BLUE_LEFT_BLUE_SIDE_FOR_DELIVERY.get().getY() - swerve.getPose2D().getY(),
                            swerve.getPose2D().getX() - FieldConstants.BLUE_LEFT_BLUE_SIDE_FOR_DELIVERY.get().getX())
            );
        }else {
            targetAngle = new Rotation2d(
                Math.atan2(FieldConstants.BLUE_RIGHT_BLUE_SIDE_FOR_DELIVERY.get().getY() - swerve.getPose2D().getY(),
                        swerve.getPose2D().getX() - FieldConstants.BLUE_RIGHT_BLUE_SIDE_FOR_DELIVERY.get().getX())
            );
        }

            Command shootForDeliveryCommand = new ParallelCommandGroup(
                swerve.turnToAngleCommand(
                            () -> new Vector2D(0, 0),
                            () -> targetAngle.minus(Rotation2d.kPi)),

                    shooter.adjustShooterForShootingCommand(
                            () -> swerve.getPose2D().getTranslation(),
                            () -> new Pose3d(
                                    new Translation3d(FieldConstants.BLUE_LEFT_BLUE_SIDE_FOR_DELIVERY.get().getTranslation()),
                                    new Rotation3d()
                            )),

                    shooter.getFuelCommand(),
                    transport.manualCommand(() -> SHOOTING_VOLTAGE));

            shootForDeliveryCommand(swerve.getTranslationSetpoint()).addRequirements(shooter);
            return shootForDeliveryCommand(swerve.getTranslationSetpoint());
        }

}
