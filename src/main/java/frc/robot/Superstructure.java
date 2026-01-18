package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.excalib.additional_utilities.PS5Controller;
import frc.excalib.control.math.MathUtils;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Transport;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.turret.ShootingTargets;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.subsystems.transport.Constants.SHOOTING_VOLTAGE;

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
        swerve = Constants.SwerveConstants.configureSwerve(Constants.startingPose);
        turret = new Turret(swerve::getPose2D);


        this.controller = controller;
    }
    private Command shootingCommand(Translation2d target){
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
        turret.turnTurretCommand(()-> ShootingTargets.HUB);
        return shootingCommand(FieldConstants.BLUE_HUB_CENTER_POSE.get().getTranslation());
    }

    public Command shootForDeliveryCommand(Translation2d current_position) {
        Translation2d target;
        double distenceToDeliveryForRight =
                FieldConstants.BLUE_RIGHT_DELIVERY_SIDE.get().getTranslation().getDistance(current_position);
        target = FieldConstants.BLUE_RIGHT_DELIVERY_SIDE.get().getTranslation();
        double distenceToDeliveryForLeft =
                FieldConstants.BLUE_LEFT_DELIVERY_SIDE.get().getTranslation().getDistance(current_position);
        target = FieldConstants.BLUE_RIGHT_DELIVERY_SIDE.get().getTranslation();
        if (distenceToDeliveryForLeft > distenceToDeliveryForRight) {
            turret.turnTurretCommand(()-> ShootingTargets.LEFT_DELIVERY);
        } else {
            turret.turnTurretCommand(()-> ShootingTargets.RIGHT_DELIVERY);
        }
        return shootingCommand(target);
    }
}
