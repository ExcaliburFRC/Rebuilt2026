package frc.robot.superstructure;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.excalib.swerve.Swerve;
import frc.robot.Constants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.turret.Turret;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.PhysicalConstants.TURRET_OFFSET_TRANSLATION;

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

        shooter = new Shooter(() -> getTurretToHubVector().get().getNorm(), swerve::getPose2D);

        this.controller = controller;


        distanceTimeOfFlightMap = new InterpolatingDoubleTreeMap();
        initDistanceTimeOfFlightMap(distanceTimeOfFlightMap);
    }

    private void initDistanceTimeOfFlightMap(InterpolatingTreeMap distanceFlightTimeTable) {
//        distanceFlightTimeTable.put(distance[meters], flight time);
        distanceFlightTimeTable.put(2.99, 0.9);
        distanceFlightTimeTable.put(2.38, 0.7);
        distanceFlightTimeTable.put(3.88, 0.95);
    }

    public Supplier<Translation2d> getTurretToHubVector() {
        Translation2d fieldToHubTranslation = BLUE_HUB_CENTER_POSE.get().getTranslation();
        Translation2d fieldToRobot = swerve.getPose2D().getTranslation();

        Translation2d robotToHub = (fieldToHubTranslation.minus(fieldToRobot)).rotateBy(swerve.getRotation2D().unaryMinus()); //maybe revese (unary minus) todo
        Translation2d turretToHub = robotToHub.minus(TURRET_OFFSET_TRANSLATION);


        ChassisSpeeds robotSpeeds = swerve.getRobotRelativeSpeeds();

        return () -> {
            Translation2d virtualHubOffset = new Translation2d(
                    robotSpeeds.vxMetersPerSecond
                            + TURRET_OFFSET_TRANSLATION.getY()
                            * robotSpeeds.omegaRadiansPerSecond,

                    robotSpeeds.vyMetersPerSecond
                            + TURRET_OFFSET_TRANSLATION.getX()
                            * robotSpeeds.omegaRadiansPerSecond
            ).times(distanceTimeOfFlightMap.get(turretToHub.getNorm()));

            return turretToHub.minus(virtualHubOffset);
        };
//        return () -> turretToHub;
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
        Translation2d turretToDelivery = robotToDelivery.minus(TURRET_OFFSET_TRANSLATION);

        return () -> turretToDelivery;
    }

    public Command autoHoodAndTurretAim() {
        return new RunCommand(
                () -> {
                    new ParallelCommandGroup(
                            turret.setPositionCommand(() -> getTurretToHubVector().get().getAngle()),
                            shooter.setHoodAngleCommand(
                                    () -> shooter.angleDistanceMap.get(turretDistanceFromHub.getAsDouble())
                            )
                    );
                }, shooter
        );
    }

    public Command shotSquenceCommand() {
        return new SequentialCommandGroup(
                shooter.smartFlyWheelVelocity(),
                new WaitUntilCommand(shooter.flyWheelInToleranceTrigger),
                new ParallelCommandGroup(
                        transport.manualCommand(() -> Constants.SHOOTER_TRANSPORT_VOLTAGE),
                        shooter.transportMechanism.manualCommand(() -> Constants.SPINDEXER_TRANSPORT_VOLTAGE)
                )
        );
    }

    public Command intakeRollerActivationCommand(double voltage) {
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
    DoubleSupplier turretDistanceFromHub = () -> getTurretToHubVector().get().getNorm();

    @Log.NT
    public double getSetpointHoodAngle(){
        return shooter.angleDistanceMap.get(getTurretToHubVector().get().getNorm());
    }












    // v / vel = kv
}
