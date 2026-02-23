package frc.robot.superstructure;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.Target;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.PhysicalConstants.*;

public class Superstructure implements Logged {
    public final Intake intake;
    public final Shooter shooter;
    public final Transport transport;
    public final Turret turret;
    public final Swerve swerve;

    public Target currentTarget = Target.IDLE;

    public final InterpolatingDoubleTreeMap distanceTimeOfFlightMap;

    public final CommandPS5Controller controller;

    public Superstructure(CommandPS5Controller controller, Swerve swerve) {
        intake = new Intake();
        transport = new Transport();

        turret = new Turret(() -> getTurretToTargetVector(() -> currentTarget).get().getAngle().getRadians());
        shooter = new Shooter(() -> getTurretToTargetVector(() -> currentTarget).get().getNorm(), swerve::getPose2D);

        this.swerve = swerve;
        this.controller = controller;

        distanceTimeOfFlightMap = new InterpolatingDoubleTreeMap();
        initDistanceTimeOfFlightMap();
    }

    private void initDistanceTimeOfFlightMap() {
//        distanceFlightTimeTable.put(distance[meters], flight time);
        distanceTimeOfFlightMap.put(2.99, 0.9);
        distanceTimeOfFlightMap.put(2.38, 0.7);
        distanceTimeOfFlightMap.put(3.88, 0.95);
    }

    public Supplier<Translation2d> getTurretToTargetVector(Supplier<Target> target) {
        Translation2d fieldToTargetTranslation = target.get().targetTranslation;
        Translation2d fieldToRobot = swerve.getPose2D().getTranslation();

        Translation2d robotToTarget = (fieldToTargetTranslation.minus(fieldToRobot)).rotateBy(swerve.getRotation2D().unaryMinus()); //maybe revese (unary minus) todo
        Translation2d turretToTarget = robotToTarget.minus(TURRET_OFFSET_TRANSLATION);

        ChassisSpeeds robotSpeeds = swerve.getRobotRelativeSpeeds();

        return () -> {
            Translation2d virtualTargetOffset = new Translation2d(
                    robotSpeeds.vxMetersPerSecond
                            + TURRET_OFFSET_TRANSLATION.getY()
                            * robotSpeeds.omegaRadiansPerSecond,

                    robotSpeeds.vyMetersPerSecond
                            + TURRET_OFFSET_TRANSLATION.getX()
                            * robotSpeeds.omegaRadiansPerSecond
            ).times(distanceTimeOfFlightMap.get(turretToTarget.getNorm()));

            return turretToTarget.minus(virtualTargetOffset);
        };
//        return () -> turretToHub;
    }


    public Supplier<Translation2d> getTurretToDeliveryVector() {
        Translation2d fieldToDeliveryTranslation;
        if (swerve.getPose2D().getTranslation().getDistance(DELIVERY_LEFT_POSE.get().getTranslation()) >
                swerve.getPose2D().getTranslation().getDistance(DELIVERY_RIGHT_POSE.get().getTranslation())) {
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
                            turret.setPositionCommand(() -> getTurretToTargetVector(() -> currentTarget).get().getAngle()),
                            shooter.setHoodAngleCommand(
                                    () -> shooter.angleDistanceMap.get(turretDistanceFromHub.getAsDouble())
                            )
                    );
                }, shooter
        );
    }

    public Command shootSquenceCommand() {
        return new SequentialCommandGroup(
                shooter.setAdjustedFlyWheelVelocity(),
                new WaitUntilCommand(shooter.flyWheelToleranceTrigger),
                new ParallelCommandGroup(
                        transport.manualCommand(() -> SHOOTER_TRANSPORT_VOLTAGE),
                        shooter.transportMechanism.manualCommand(() -> SPINDEXER_TRANSPORT_VOLTAGE)
                )
        );
    }

    public Command shootToHubCommand(){
        return new ParallelCommandGroup(
                shooter.shootToHubCommand(),
                turret.targetHubCommand(),
                transport.transportFuelCommand()
        );
    }

    public Command intakeRollerActivationCommand(double voltage) {
        return intake.rollerManualCommand(voltage);
    }

    @Log.NT
    public double getTurretToHubVectorAngle() {
        return Units.radiansToDegrees(getTurretToTargetVector(() -> currentTarget).get().getAngle().getRadians());
    }

    @Log.NT
    public double getTurretToDeliveryVectorAngle() {
        return Units.radiansToDegrees(getTurretToDeliveryVector().get().getAngle().getRadians());
    }

    @Log.NT
    public double getTurretToHubVectorDist() {
        return getTurretToTargetVector(() -> currentTarget).get().getNorm();
    }

    @Log.NT
    DoubleSupplier turretDistanceFromHub = () -> getTurretToTargetVector(() -> currentTarget).get().getNorm();

    @Log.NT
    public double getSetpointHoodAngle() {
        return shooter.angleDistanceMap.get(getTurretToTargetVector(() -> currentTarget).get().getNorm());
    }
}
