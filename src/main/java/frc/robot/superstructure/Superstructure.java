package frc.robot.superstructure;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.swerve.Swerve;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Transport;
import frc.robot.util.ShootingTarget;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.subsystems.transport.transportConstans.SHOOTING_VOLTAGE;
import static frc.robot.util.ShootingTarget.*;

public class Superstructure {
    public final Intake intake;
    public final Shooter shooter;
    public final Transport transport;
    public final Turret turret;
    public final Swerve swerve;

    public final InterpolatingDoubleTreeMap distanceTimeOfFlightMap;

    public final CommandPS5Controller controller;

    public Translation3d targetShootingPose = BLUE_HUB_CENTER_POSE.getAsCurrentAlliance().getTranslation();

    public Superstructure(CommandPS5Controller controller, Swerve swerve) {
        intake = new Intake();
        transport = new Transport();

        this.swerve = swerve;

        turret = new Turret(() -> getTurretToHubVector().getAngle().getRadians());

        shooter = new Shooter(() -> getTurretToHubVector().getNorm());

        this.controller = controller;


        distanceTimeOfFlightMap = new InterpolatingDoubleTreeMap();
        initDistanceTimeOfFlightMap();
    }

    private void initDistanceTimeOfFlightMap(){
        // TODO
    }


    // turret relative
    public Translation2d getTurretToHubVector() {

        Translation2d fieldToHubTranslation = BLUE_HUB_CENTER_POSE.getAsCurrentAlliance().getTranslation().toTranslation2d();
        Translation2d fieldToRobot = swerve.getPose2D().getTranslation();

        Translation2d robotToHub = (fieldToHubTranslation.minus(fieldToRobot)).rotateBy(swerve.getRotation2D()); //maybe revese (unary minus) todo
        Translation2d turretToHub = robotToHub.minus(Constants.TURRET_OFFSET_TRANSLATION);

        ChassisSpeeds robotSpeeds = swerve.getRobotRelativeSpeeds();

        Translation2d virtualHubOffset = new Translation2d(
                robotSpeeds.vxMetersPerSecond + Constants.TURRET_OFFSET_TRANSLATION.getY() * robotSpeeds.omegaRadiansPerSecond,
                robotSpeeds.vyMetersPerSecond + Constants.TURRET_OFFSET_TRANSLATION.getX() * robotSpeeds.omegaRadiansPerSecond
        ).times(distanceTimeOfFlightMap.get(turretToHub.getNorm()));
//        return turretToHub.minus(virtualHubOffset);  for shooting on the fly

        return turretToHub;
    }


}
