package frc.robot.superstructure;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.PhysicalConstants.*;

public class Superstructure implements Logged {
    public final Intake intake;
    public final Shooter shooter;
    public final Transport transport;
    public final Turret turret;
    public final Swerve swerve;

    public enum SuperstructureGoal {
        IDLE,
        INTAKE,
        TRACK_HUB,
        SHOOT_HUB,
        SHOOT_DELIVERY,
        SHOOT_FIXED,
        EJECT
    }

    @Log.NT
    private SuperstructureGoal currentGoal = SuperstructureGoal.TRACK_HUB;

    public Target currentTarget = Target.HUB;
    private double manualFlywheelVel = 0;
    private double manualHoodAngle = 0;

    public final InterpolatingDoubleTreeMap distanceTimeOfFlightMap = createFlightTimeMap();

    private static InterpolatingDoubleTreeMap createFlightTimeMap() {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
        map.put(1.95, 1.12);
        map.put(2.36, 1.21);
        map.put(2.99, 1.28);
        map.put(3.6, 1.35);
        map.put(4.3, 1.39);
        return map;
    }

    public Superstructure(Swerve swerve) {
        boolean isReal = Constants.CURRENT_MODE == Constants.Mode.REAL;
        IntakeIO intakeIO       = isReal ? new IntakeIOTalonFX()   : new IntakeIOSim();
        TransportIO transportIO = isReal ? new TransportIOTalonFX() : new TransportIOSim();
        TurretIO turretIO       = isReal ? new TurretIOTalonFX()   : new TurretIOSim();
        ShooterIO shooterIO     = isReal ? new ShooterIOTalonFX()  : new ShooterIOSim();

        intake    = new Intake(intakeIO);
        transport = new Transport(transportIO);
        this.swerve = swerve;

        turret  = new Turret(turretIO, () -> getTurretToTargetVector(currentTarget).getAngle().getRadians(), () -> swerve.getRotation2D().getRadians());
        shooter = new Shooter(shooterIO, () -> getTurretToTargetVector(currentTarget).getNorm(), swerve::getPose2D);

        initDeliveryTrigger();
    }

    public void setGoal(SuperstructureGoal goal) {
        this.currentGoal = goal;
    }

    public void periodic() {
        switch (currentGoal) {
            case IDLE:
                intake.setClosed();
                shooter.setIdle();
                transport.drumMechanism.setVoltage(0);
                break;
            case INTAKE:
                intake.setIntake();
                transport.drumMechanism.setVoltage(0);
                break;
            case TRACK_HUB:
                currentTarget = Target.HUB;
                shooter.setTarget(Target.HUB);
                shooter.setShootingMode(false);
                turret.setTargetAngle(getTurretToTargetVector(Target.HUB).getAngle().getRadians());
                break;
            case SHOOT_HUB:
                currentTarget = Target.HUB;
                shooter.setTarget(Target.HUB);
                shooter.setShootingMode(true);
                turret.setTargetAngle(getTurretToTargetVector(Target.HUB).getAngle().getRadians());
                transport.drumMechanism.setVoltage(-3.0);
                break;
            case SHOOT_DELIVERY:
                currentTarget = Target.DELIVERY;
                shooter.setTarget(Target.DELIVERY);
                shooter.setShootingMode(true);
                turret.setTargetAngle(getTurretToDeliveryVector().getAngle().getRadians());
                transport.drumMechanism.setVoltage(-3.0);
                break;
            case SHOOT_FIXED:
                shooter.setTarget(Target.HUB); // Or some manual target
                shooter.setManualSetpoints(manualFlywheelVel, manualHoodAngle);
                shooter.setShootingMode(true);
                transport.drumMechanism.setVoltage(-3.0);
                break;
            case EJECT:
                intake.setRollerVoltage(-7);
                transport.drumMechanism.setVoltage(-7);
                break;
        }
    }

    private Pose2d getClosestDeliveryPose() {
        Pose2d right = DELIVERY_RIGHT_POSE.get();
        Pose2d left = DELIVERY_LEFT_POSE.get();
        Translation2d current = swerve.getPose2D().getTranslation();
        return current.getDistance(right.getTranslation()) < current.getDistance(left.getTranslation()) ? right : left;
    }

    private Pose2d getClosestNetEndPose() {
        Pose2d right = NET_END_RIGHT_POSE.get();
        Pose2d left = NET_END_LEFT_POSE.get();
        Translation2d current = swerve.getPose2D().getTranslation();
        return current.getDistance(right.getTranslation()) < current.getDistance(left.getTranslation()) ? right : left;
    }

    private void initDeliveryTrigger() {
        // Triggers can be moved to RobotContainer if needed for button bindings
    }

    public Translation2d getTurretToTargetVector(Target target) {
        ChassisSpeeds robotSpeeds = swerve.getRobotRelativeSpeeds();
        Pose2d robotPose = swerve.getPose2D();
        Rotation2d robotRot = robotPose.getRotation();

        Translation2d turretField = getTurretOnField().getTranslation();

        Translation2d fieldVector = target.getTargetTranslation().minus(turretField);
        Translation2d robotVector = fieldVector.rotateBy(robotRot.unaryMinus());
        Translation2d turretToTarget = robotVector.rotateBy(Rotation2d.fromDegrees(-180));

        Translation2d virtualTargetOffset = new Translation2d(
                robotSpeeds.vxMetersPerSecond - TURRET_OFFSET_TRANSLATION.getY() * robotSpeeds.omegaRadiansPerSecond,
                robotSpeeds.vyMetersPerSecond - TURRET_OFFSET_TRANSLATION.getX() * robotSpeeds.omegaRadiansPerSecond
        ).times(distanceTimeOfFlightMap.get(turretToTarget.getNorm()));

        return turretToTarget.minus(virtualTargetOffset);
    }

    public Translation2d getTurretToDeliveryVector() {
        Translation2d fieldToDeliveryTranslation = getClosestDeliveryPose().getTranslation();
        Translation2d fieldToRobot = swerve.getPose2D().getTranslation();

        Translation2d robotToDelivery = fieldToDeliveryTranslation.minus(fieldToRobot).rotateBy(swerve.getRotation2D().unaryMinus());
        return robotToDelivery.minus(TURRET_OFFSET_TRANSLATION);
    }

    public Command shootToDeliveryCommand() {
        return Commands.startEnd(() -> setGoal(SuperstructureGoal.SHOOT_DELIVERY), () -> setGoal(SuperstructureGoal.IDLE));
    }

    public Command shootToHubCommand() {
        return Commands.startEnd(() -> setGoal(SuperstructureGoal.SHOOT_HUB), () -> setGoal(SuperstructureGoal.IDLE));
    }

    public Command trackHubCommand() {
        return Commands.startEnd(() -> setGoal(SuperstructureGoal.TRACK_HUB), () -> setGoal(SuperstructureGoal.IDLE));
    }

    public Command shootFixedCommand(double flywheelVelocity, double hoodAngle) {
        return Commands.startEnd(() -> {
            this.manualFlywheelVel = flywheelVelocity;
            this.manualHoodAngle = hoodAngle;
            setGoal(SuperstructureGoal.SHOOT_FIXED);
        }, () -> setGoal(SuperstructureGoal.IDLE));
    }

    public Command intakeCommand() {
        return Commands.startEnd(() -> setGoal(SuperstructureGoal.INTAKE), () -> setGoal(SuperstructureGoal.IDLE));
    }

    public Command ejectCommand() {
        return Commands.startEnd(() -> setGoal(SuperstructureGoal.EJECT), () -> setGoal(SuperstructureGoal.IDLE));
    }

    public Command stopIntakeCommand() {
        return Commands.runOnce(() -> setGoal(SuperstructureGoal.IDLE));
    }

    @Log.NT
    public double getTurretToHubVectorAngle() {
        return getTurretToTargetVector(Target.HUB).getAngle().getDegrees();
    }

    @Log.NT
    public double getTurretToHubVectorDist() {
        return getTurretToTargetVector(Target.HUB).getNorm();
    }

    @Log.NT
    public Pose2d getTurretOnField() {
        Pose2d robotPose = swerve.getPose2D();
        Translation2d robotPos = robotPose.getTranslation();
        Rotation2d robotRot = robotPose.getRotation();

        Translation2d turretField = robotPos.plus(TURRET_OFFSET_TRANSLATION.rotateBy(robotRot));
        return new Pose2d(turretField, swerve.getRotation2D().minus(turret.turretMechanism.getPosition().unaryMinus()).plus(Rotation2d.kPi));
    }
}
