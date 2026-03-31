package frc.robot.superstructure;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.slam.mapper.AuroraClient;
import frc.excalib.swerve.Swerve;
import frc.excalib.control.math.MathUtils;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.transport.TransportIO;
import frc.robot.subsystems.transport.TransportIOSim;
import frc.robot.subsystems.transport.TransportIOTalonFX;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.util.Target;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.Supplier;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.PhysicalConstants.*;

public class Superstructure implements Logged {
    public final Intake intake;
    public final Shooter shooter;
    public final Transport transport;
    public final Turret turret;
    public final Swerve swerve;

    public Target currentTarget = Target.HUB;

    public final InterpolatingDoubleTreeMap distanceTimeOfFlightMap;

    private Trigger deliveryTrigger;

    public Superstructure(Swerve swerve) {
        // Select IO implementations based on the configured operating mode
        boolean isReal = Constants.CURRENT_MODE == Constants.Mode.REAL;
        IntakeIO intakeIO       = isReal ? new IntakeIOTalonFX()   : new IntakeIOSim();
        TransportIO transportIO = isReal ? new TransportIOTalonFX() : new TransportIOSim();
        TurretIO turretIO       = isReal ? new TurretIOTalonFX()   : new TurretIOSim();
        ShooterIO shooterIO     = isReal ? new ShooterIOTalonFX()  : new ShooterIOSim();

        intake    = new Intake(intakeIO);
        transport = new Transport(transportIO);

        this.swerve = swerve;

        distanceTimeOfFlightMap = new InterpolatingDoubleTreeMap();
        initDistanceTimeOfFlightMap();

        turret  = new Turret(turretIO, () -> getTurretToTargetVector(() -> currentTarget).get().getAngle().getRadians(), () -> swerve.getRotation2D().getRadians());
        shooter = new Shooter(shooterIO, () -> getTurretToTargetVector(() -> currentTarget).get().getNorm(), swerve::getPose2D);

        turret.setDefaultCommand(turret.defaultCommand());
        shooter.setDefaultCommand(shooter.defaultCommand());

        initDeliveryTrigger();
    }

    private void initDeliveryTrigger() {
        deliveryTrigger = new Trigger(() -> {
            Translation2d deliveryRightPose = DELIVERY_RIGHT_POSE.get().getTranslation();
            Translation2d deliveryLeftPose = DELIVERY_LEFT_POSE.get().getTranslation();
            Translation2d netEndRightPose = NET_END_RIGHT_POSE.get().getTranslation();
            Translation2d netEndLeftPose = NET_END_LEFT_POSE.get().getTranslation();
            Translation2d targetDeliveryPose = deliveryRightPose;
            Translation2d targetNetEndPose = netEndRightPose;
            if (swerve.getPose2D().getTranslation().getDistance(deliveryRightPose) >
                    swerve.getPose2D().getTranslation().getDistance(deliveryLeftPose)) {
                targetDeliveryPose = deliveryLeftPose;
                targetNetEndPose = netEndLeftPose;
            } else {
                targetDeliveryPose = deliveryRightPose;
                targetNetEndPose = netEndRightPose;
            }
            if (Math.abs(MathUtils.getPosesTangentAngle(swerve.getPose2D().getTranslation(), targetDeliveryPose)) >
                    Math.abs(MathUtils.getPosesTangentAngle(swerve.getPose2D().getTranslation(), targetNetEndPose))) {
                return true;
            } else {
                return false;
            }
        });
    }

    private void initDistanceTimeOfFlightMap() {
//        distanceFlightTimeTable.put(distance[meters], flight time);
        distanceTimeOfFlightMap.put(4.3, 1.39);
        distanceTimeOfFlightMap.put(2.99, 1.28);
        distanceTimeOfFlightMap.put(2.36, 1.21);
        distanceTimeOfFlightMap.put(3.6, 1.35);
        distanceTimeOfFlightMap.put(1.95, 1.12);
        distanceTimeOfFlightMap.put(0.0, 0.0);
        distanceTimeOfFlightMap.put(0.0, 0.0);
        distanceTimeOfFlightMap.put(0.0, 0.0);
    }


//        return () -> {
//            Translation2d virtualTargetOffset = new Translation2d(
//                    robotSpeeds.vxMetersPerSecond
//                            + TURRET_OFFSET_TRANSLATION.getY()
//                            * robotSpeeds.omegaRadiansPerSecond,
//
//                    robotSpeeds.vyMetersPerSecond
//                            + TURRET_OFFSET_TRANSLATION.getX()
//                            * robotSpeeds.omegaRadiansPerSecond
//            ).times(distanceTimeOfFlightMap.get(turretToTarget.getNorm()));
//
//
//            Translation2d virtualTurretToTarget = turretToTarget.minus(virtualTargetOffset);
//            return virtualTurretToTarget;
//        };

    public Supplier<Translation2d> getTurretToTargetVector(Supplier<Target> target) {
        return () -> {

            ChassisSpeeds robotSpeeds = swerve.getRobotRelativeSpeeds();


            Pose2d robotPose = swerve.getPose2D();
            Rotation2d robotRot = robotPose.getRotation();


            Translation2d turretField =
                    getTurretOnField().getTranslation();

            Translation2d fieldVector =
                    target.get().getTargetTranslation().minus(turretField);

            Translation2d robotVector =
                    fieldVector.rotateBy(robotRot.unaryMinus());

            Translation2d turretToTarget = robotVector.rotateBy(Rotation2d.fromDegrees(-180));

            Translation2d virtualTargetOffset = new Translation2d(
                    robotSpeeds.vxMetersPerSecond
                            - TURRET_OFFSET_TRANSLATION.getY() * robotSpeeds.omegaRadiansPerSecond,

                    robotSpeeds.vyMetersPerSecond
                            - TURRET_OFFSET_TRANSLATION.getX() * robotSpeeds.omegaRadiansPerSecond
            ).times(distanceTimeOfFlightMap.get(turretToTarget.getNorm()));


            Translation2d virtualTurretToTarget = turretToTarget.minus(virtualTargetOffset);
            return virtualTurretToTarget;
        };
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

    public Command shootToDeliveryCommand() {
        Translation2d deliveryRightPose = DELIVERY_RIGHT_POSE.get().getTranslation();
        Translation2d deliveryLeftPose = DELIVERY_LEFT_POSE.get().getTranslation();
        Translation2d targetDeliveryPose = deliveryRightPose;
        if (swerve.getPose2D().getTranslation().getDistance(deliveryRightPose) >
                swerve.getPose2D().getTranslation().getDistance(deliveryLeftPose)) {
            targetDeliveryPose = deliveryLeftPose;
        } else {
            targetDeliveryPose = deliveryRightPose;
        }
        return new ConditionalCommand(
                turret.targetDeliveryCommand(),
                new ParallelCommandGroup(
                        shooter.shootToDeliveryCommand(),
//              turret.targetDeliveryCommand(),
                        transport.transportFuelCommand()
                ).alongWith(setSuperstructureTarget(Target.DELIVERY)),
                deliveryTrigger
        );
    }

    public Command shootToHubCommand() {
        return new ParallelCommandGroup(
                shooter.shootToHubCommand(),
                turret.targetHubCommand(),
                transport.transportFuelCommand()
        ).alongWith(setSuperstructureTarget(Target.HUB));
    }

    public Command trackHubCommand() {
        return new ParallelCommandGroup(
                shooter.trackHubCommand(),
                turret.targetHubCommand()
        ).alongWith(setSuperstructureTarget(Target.HUB));
    }

    public Command shootFixedCommand(double flywheelVelocity, double hoodAngle) {
        return new ParallelCommandGroup(
                shooter.setFlyWheelDynamicVelocity(() -> flywheelVelocity),
                shooter.setHoodAngleCommand(() -> hoodAngle),
                shooter.manualTransport(),
                transport.transportFuelCommand(),
                turret.targetHubCommand()
        ).alongWith(setSuperstructureTarget(Target.HUB));
    }

    public Command intakeCommand() {
        return intake.intakeCommand();
    }

    public Command ejectCommand() {
        return intake.rollerManualCommand(-7); // Assuming -7 volts is eject based on intakeCommand using 7
    }

    public Command stopIntakeCommand() {
        return intake.closeCommand();
    }

    public Command intakeRollerActivationCommand(double voltage) {
        return intake.rollerManualCommand(voltage);
    }

    public Command setSuperstructureTarget(Target targetToSet) {
        return new InstantCommand(() -> currentTarget = targetToSet);
    }

    @Log.NT
    public double getTurretToHubVectorAngle() {
        return getTurretToTargetVector(() -> Target.HUB).get().getAngle().getDegrees();
    }

    @Log.NT
    public double getTurretToHubVectorDist() {
        return getTurretToTargetVector(() -> Target.HUB).get().getNorm();
    }

    @Log.NT
    public Pose2d getTurretOnField() {
        Pose2d robotPose = swerve.getPose2D();
        Translation2d robotPos = robotPose.getTranslation();
        Rotation2d robotRot = robotPose.getRotation();

        // turret position in field coordinates
        Translation2d turretField =
                robotPos.plus(TURRET_OFFSET_TRANSLATION.rotateBy(robotRot));

        return new Pose2d(turretField, swerve.getRotation2D().minus(turret.turretMechanism.getPosition().unaryMinus()).plus(Rotation2d.kPi));
    }
}
