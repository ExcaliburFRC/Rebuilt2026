package frc.robot.superstructure;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.excalib.swerve.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.turret.Turret;
import monologue.Annotations;
import monologue.Annotations.Log;
import monologue.Logged;
import org.opencv.core.Mat;

import java.util.function.Supplier;

import static frc.robot.Constants.FieldConstants.*;

public class Superstructure implements Logged {
    public final Intake intake;
    public final Shooter shooter;
    public final Transport transport;
    public final Turret turret;
    public final Swerve swerve;

    public final InterpolatingDoubleTreeMap distanceTimeOfFlightMap;

    public final CommandPS5Controller controller;

    public Superstructure(CommandPS5Controller controller, Swerve swerve) {
        intake = new Intake();
        transport = new Transport();

        this.swerve = swerve;

        turret = new Turret(() -> getTurretToHubVector().get().getAngle().getRadians());

        shooter = new Shooter(() -> getTurretToHubVector().get().getNorm(), () -> swerve.getPose2D());

        this.controller = controller;


        distanceTimeOfFlightMap = new InterpolatingDoubleTreeMap();
        initDistanceTimeOfFlightMap();
    }

    private void initDistanceTimeOfFlightMap() {
        // TODO
    }

    // turret relative
    public Supplier<Translation2d> getTurretToHubVector() {
        Translation2d fieldToHubTranslation = BLUE_HUB_CENTER_POSE.get().getTranslation();
        Translation2d fieldToRobot = swerve.getPose2D().getTranslation();

        Translation2d robotToHub = (fieldToHubTranslation.minus(fieldToRobot)).rotateBy(swerve.getRotation2D().unaryMinus()); //maybe revese (unary minus) todo
        Translation2d turretToHub = robotToHub.minus(Constants.TURRET_OFFSET_TRANSLATION);

//        ChassisSpeeds robotSpeeds = swerve.getRobotRelativeSpeeds();

//        Translation2d virtualHubOffset = new Translation2d(
//                robotSpeeds.vxMetersPerSecond + Constants.TURRET_OFFSET_TRANSLATION.getY() * robotSpeeds.omegaRadiansPerSecond,
//                robotSpeeds.vyMetersPerSecond + Constants.TURRET_OFFSET_TRANSLATION.getX() * robotSpeeds.omegaRadiansPerSecond
//        ).times(distanceTimeOfFlightMap.get(turretToHub.getNorm()));
//        return turretToHub.minus(virtualHubOffset);  for shooting on the fly

        return () -> turretToHub;
    }

    public Supplier<Translation2d> getTurretToDeliveryVector() {
        Translation2d fieldToDeliveryTranslation;
        if (swerve.getPose2D().getTranslation().getDistance(DELIVERY_LEFT_POSE.get().getTranslation()) >
                swerve.getPose2D().getTranslation().getDistance(DELIVERY_RIGHT_POSE.get().getTranslation())){
            fieldToDeliveryTranslation = DELIVERY_RIGHT_POSE.get().getTranslation();
        } else {
            fieldToDeliveryTranslation = DELIVERY_LEFT_POSE.get().getTranslation();
        }
        Translation2d fieldToRobot = swerve.getPose2D().getTranslation();

        Translation2d robotToDelivery = (fieldToDeliveryTranslation.minus(fieldToRobot)).rotateBy(swerve.getRotation2D().unaryMinus());
        Translation2d turretToDelivery = robotToDelivery.minus(Constants.TURRET_OFFSET_TRANSLATION);

        return () -> turretToDelivery;
    }

    public Command turretTest() {
        return turret.setPositionCommand(() -> getTurretToHubVector().get().getAngle());
    }

    public Command testShotCommand() {
        return new ParallelCommandGroup(
                turret.setPositionCommand(() -> new Rotation2d(Math.PI / 2)),
                //shooter.setHoodAngleCommand(() -> 1),
                shooter.setFlyWheelVelocityCommand(() -> 80),
                new WaitCommand(4).andThen(
                        new ParallelCommandGroup(
                                new InstantCommand(
                                        () -> shooter.transportMechanism.setVoltage(6)
                                ),
                                new InstantCommand(
                                        () -> transport.drumMechanism.setVoltage(-6)
                                )
                        )
                )
        );
    }
    public Command testIntake(double voltage) {
        return intake.rollerManualCommand(voltage);
    }

    @Log.NT
    public double getTurretToHubVectorAngle() {
        return Units.radiansToDegrees(getTurretToHubVector().get().getAngle().getRadians());
    }

    @Log.NT
    public double getTurretToDeliveryVectorAngle() {
        return Units.radiansToDegrees(getTurretToDeliveryVector().get().getAngle().getRadians());
    }

    @Log.NT
    public double getTurretToHubVectorDist() {
        return getTurretToHubVector().get().getNorm();
    }

    @Log.NT
    public Pose2d getpoSE(){
        return new Pose2d(
                swerve.getPose2D().getTranslation(),
                getTurretToHubVector().get().getAngle()
        );
    }


}
