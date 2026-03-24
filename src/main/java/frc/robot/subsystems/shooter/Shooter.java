package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.additional_utilities.Color;
import frc.excalib.additional_utilities.LEDs;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.EMAFilter;
import frc.excalib.control.math.periodics.PeriodicScheduler;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.mechanisms.Mechanism;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import frc.excalib.mechanisms.turret.Turret;
import monologue.Logged;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.excalib.additional_utilities.AllianceUtils.FIELD_LENGTH_METERS;
import static frc.excalib.additional_utilities.AllianceUtils.FIELD_WIDTH_METERS;
import static frc.robot.Constants.DISABLE_SUBSYSTEMS;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.PhysicalConstants.TURRET_OFFSET_TRANSLATION;
import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.shooter.ShooterStates.IDLE;
import static frc.robot.subsystems.shooter.ShooterStates.LOOK_HUB;
import static monologue.Annotations.Log.*;

public class Shooter extends SubsystemBase implements Logged {

    private final TalonFXMotor hoodMotor, flyWheelMotorTop, flyWheelMotorLow, turretMotor;
    private final MotorGroup shooterMotorGroup;
    private final CANcoder hoodEncoder, turretEncoder;
    private final PIDController angleController;

    public final Turret turretMechanism;
    public final DoubleSupplier turretAngleSupplier;

    public final DoubleSupplier turretRelativeAngleToTarget;

    private ShooterStates currentState;

    private final FlyWheel flyWheelMechanism;
    private final Mechanism hoodMechanism;

    private DoubleSupplier hoodAngleSupplier;
    private final SoftLimit hoodSoftLimit;

    private final Supplier<Pose2d> robotPositionSupplier;

    private final DoubleSupplier turretRelativeDistanceFromTarget;

    public Supplier<Translation2d> turretToHubVector;

    private DoubleSupplier flywheelVelocitySetpoint;
    private DoubleSupplier hoodAngleSetpoint;

    private final EMAFilter flywheelVelocityFilter;

    private final InterpolatingDoubleTreeMap angleDistanceMap;
    private final InterpolatingDoubleTreeMap velocityDistanceMap;
    private final InterpolatingDoubleTreeMap distanceTimeOfFlightMap;

    private final Trigger volatileTrenchHoodTrigger;

    private final Trigger activateLedsTrigger;
    private final Trigger flyWheelReadyTrigger;
    private final Trigger hoodAdjustedTrigger;
    public final Trigger isTurretAligned;

    public final Trigger shooterReady;

    private Supplier<ChassisSpeeds> swerveSpeeds;

    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> swerveSpeeds) {
        hoodMotor = new TalonFXMotor(HOOD_MOTOR_ID, SUBSYSTEMS_CANBUS);
        flyWheelMotorLow = new TalonFXMotor(FLYWHEEL_MOTOR_LOW_ID, SUBSYSTEMS_CANBUS);
        flyWheelMotorTop = new TalonFXMotor(FLYWHEEL_MOTOR_TOP_ID, SUBSYSTEMS_CANBUS);
        hoodEncoder = new CANcoder(HOOD_ENCODER_ID, SUBSYSTEMS_CANBUS);

        this.swerveSpeeds = swerveSpeeds;
        shooterMotorGroup = new MotorGroup(flyWheelMotorLow, flyWheelMotorTop);
        shooterMotorGroup.setIdleState(IdleState.BRAKE);
        shooterMotorGroup.setMotorPosition(0);
        shooterMotorGroup.setVelocityConversionFactor((double) 40 / 48);
        shooterMotorGroup.setPositionConversionFactor((double) 40 / 48);

        flyWheelMotorLow.setInverted(DirectionState.FORWARD);
        flyWheelMotorTop.setInverted(DirectionState.FORWARD);

        currentState = LOOK_HUB;

        flyWheelMotorTop.setCurrentLimit(120, 80);
        flyWheelMotorLow.setCurrentLimit(120, 80);

        turretMotor = new TalonFXMotor(TURRET_MOTOR_ID, SUBSYSTEMS_CANBUS);
        turretEncoder = new CANcoder(TURRET_ENCODER_ID, SUBSYSTEMS_CANBUS);
        turretEncoder.setPosition(turretEncoder.getAbsolutePosition().getValueAsDouble());
        turretAngleSupplier = () -> turretEncoder.getPosition().getValueAsDouble() * ENCODER_POSITION_CONVERSION_FACTOR;
        turretMotor.setMotorPosition(turretEncoder.getPosition().getValueAsDouble());

        turretMotor.setCurrentLimit(120, 80);
        this.turretRelativeAngleToTarget = () -> getTurretToTargetVector().get().getAngle().getRadians();
        turretMotor.setInverted(DirectionState.REVERSE);

        turretMotor.setIdleState(IdleState.BRAKE);
        turretMotor.setMotorPosition(turretAngleSupplier.getAsDouble());
        turretMotor.setPositionConversionFactor(MOTOR_POSITION_CONVERSION_FACTOR);
        turretMotor.setVelocityConversionFactor(MOTOR_POSITION_CONVERSION_FACTOR);

        turretMechanism = new frc.excalib.mechanisms.turret.Turret(
                turretMotor,
                TURRET_CONTINUOUS_SOFTLIMIT,
                TURRET_GAINS,
                PID_TOLERANCE,
                turretMotor::getMotorPosition,
                new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 100)
        );

        isTurretAligned = new Trigger(
                () -> Math.abs(
                        turretMechanism.getPosition().getRadians() -
                                SOFT_LIMIT.limit(
                                        TURRET_CONTINUOUS_SOFTLIMIT.getSetpoint(
                                                turretAngleSupplier.getAsDouble(),
                                                turretRelativeAngleToTarget.getAsDouble()))) < PID_TOLERANCE

        );

        hoodEncoder.setPosition(hoodEncoder.getAbsolutePosition().getValueAsDouble());

        distanceTimeOfFlightMap = new InterpolatingDoubleTreeMap();

        turretToHubVector = getTurretToTargetVector();

        angleController = new PIDController(HOOD_GAINS.kp, HOOD_GAINS.ki, HOOD_GAINS.kd);
        angleController.setTolerance(0.01);

        hoodMotor.setIdleState(IdleState.BRAKE);
        hoodMotor.setInverted(DirectionState.FORWARD);
        hoodAngleSupplier = () -> (hoodEncoder.getPosition().getValueAsDouble() * POSITION_CONVERSION_FACTOR);

        hoodMotor.setPositionConversionFactor(POSITION_CONVERSION_FACTOR * ((double) -0.208 / 1.497) * 1.0231);
        hoodMotor.setMotorPosition(hoodAngleSupplier.getAsDouble());

        flywheelVelocitySetpoint = () -> 0;
        hoodAngleSetpoint = () -> 0;
        hoodMotor.setIdleState(IdleState.COAST);

        flyWheelMechanism = new FlyWheel(shooterMotorGroup, FLYWHEEL_MAX_ACCELERATION, FLYWHEEL_MAX_JERK, FLYWHEEL_GAINS);

        flywheelVelocityFilter = new EMAFilter(
                flyWheelMechanism::getVelocity,
                0.05,
                PeriodicScheduler.PERIOD.MILLISECONDS_20);


        PeriodicScheduler.PERIOD.MILLISECONDS_20.add(flywheelVelocityFilter);

        robotPositionSupplier = poseSupplier;

        angleDistanceMap = new InterpolatingDoubleTreeMap();
        initAngleMap();

        velocityDistanceMap = new InterpolatingDoubleTreeMap();
        initVelocityMap();

        flyWheelReadyTrigger = new Trigger(() -> Math.abs(flywheelVelocityFilter.getValue() - flywheelVelocitySetpoint.getAsDouble()) < FLYWHEEL_TOLERANCE);

        hoodAdjustedTrigger = new Trigger(() -> Math.abs(hoodAngleSupplier.getAsDouble() - hoodAngleSetpoint.getAsDouble()) < HOOD_TOLERANCE);

        activateLedsTrigger = new Trigger(() -> flyWheelMechanism.getVelocity() > 3);
        activateLedsTrigger.onTrue(LEDs.getInstance().setPattern(LEDs.LEDPattern.BLINKING, Color.Colors.ORANGE.color).andThen(LEDs.getInstance().restoreLEDs()));

        volatileTrenchHoodTrigger = new Trigger(() -> {
            Pose2d pose = poseSupplier.get();
            if (AllianceUtils.isBlueAlliance()) {
                return (pose.getX() > FRONT_TRENCH_SIDEX_LINE_DIST_METERS && pose.getY() < TRENCH_SIDEY_LINE_DIST_METERS) && (pose.getX() < BACK_TRENCH_SIDEX_LINE_DIST_METERS);
            } else {
                return (pose.getX() < FIELD_LENGTH_METERS - FRONT_TRENCH_SIDEX_LINE_DIST_METERS && pose.getY() > FIELD_WIDTH_METERS - TRENCH_SIDEY_LINE_DIST_METERS) && (pose.getX() > FIELD_LENGTH_METERS - BACK_TRENCH_SIDEX_LINE_DIST_METERS);
            }
        });

        hoodMechanism = new Mechanism(hoodMotor);

        hoodSoftLimit = new SoftLimit(() -> HOOD_MIN_ANGLE_LIMIT, () -> {
            if (volatileTrenchHoodTrigger.getAsBoolean()) {
                return HOOD_MAX_ANGLE_LIMIT_IN_TRENCH;
            }
            return HOOD_MAX_ANGLE_LIMIT;
        });

        initDistanceTimeOfFlightMap();

        this.turretRelativeDistanceFromTarget = () -> getTurretToTargetVector().get().getNorm();

        shooterReady = isTurretAligned
                .and(flyWheelReadyTrigger)
                .and(hoodAdjustedTrigger);

        setDefaultCommand(defaultCommand().unless(() -> DISABLE_SUBSYSTEMS));
    }

    private void initDistanceTimeOfFlightMap() {
        distanceTimeOfFlightMap.put(0.0, 0.0); //v0
        distanceTimeOfFlightMap.put(1.626, 0.746); //v0
        distanceTimeOfFlightMap.put(2.04, 1.038); //v0
        distanceTimeOfFlightMap.put(2.277, 1.116); //v0
        distanceTimeOfFlightMap.put(2.321, 1.1433); //v0
        distanceTimeOfFlightMap.put(2.595, 1.178); //v0
        distanceTimeOfFlightMap.put(2.672, 1.166); //v0
        distanceTimeOfFlightMap.put(2.87, 1.21); //v0
        distanceTimeOfFlightMap.put(3.038, 1.144); //v0
        distanceTimeOfFlightMap.put(3.419, 1.452); //v0
        distanceTimeOfFlightMap.put(3.5, 1.165); //v0
    }

    public void initAngleMap() {
        angleDistanceMap.put(0.0, 0.0);
        angleDistanceMap.put(2.041, 0.0);
        angleDistanceMap.put(2.359, 0.0);
        angleDistanceMap.put(2.538, 0.0);
        angleDistanceMap.put(2.816, 0.05);
        angleDistanceMap.put(2.995, 0.1);
        angleDistanceMap.put(3.293, 0.15);
        angleDistanceMap.put(3.53, 0.175);
        angleDistanceMap.put(3.695, 0.2);
        angleDistanceMap.put(3.98, 0.3);
        angleDistanceMap.put(4.215, 0.3);
        angleDistanceMap.put(4.42, 0.3);
        angleDistanceMap.put(4.66, 0.3);
        angleDistanceMap.put(4.83, 0.3);
        angleDistanceMap.put(5.303, 0.4);
        angleDistanceMap.put(5.49, 0.4);
        angleDistanceMap.put(5.798, 0.46);
        angleDistanceMap.put(6.0, 0.5);
        angleDistanceMap.put(6.334, 0.52);
        angleDistanceMap.put(6.738, 0.56);
        angleDistanceMap.put(7.658, 0.56);
    }

    public void initVelocityMap() {
//          velocityDistanceMapTable.put(distance[meters], flywheel velocity);
        velocityDistanceMap.put(0.0, 0.0);
        velocityDistanceMap.put(2.041, 33.0);
        velocityDistanceMap.put(2.359, 36.0);
        velocityDistanceMap.put(2.538, 37.5);
        velocityDistanceMap.put(2.816, 37.5);
        velocityDistanceMap.put(2.995, 38.0);
        velocityDistanceMap.put(3.293, 38.0);
        velocityDistanceMap.put(3.53, 38.5);
        velocityDistanceMap.put(3.695, 39.0);
        velocityDistanceMap.put(3.98, 39.5);
        velocityDistanceMap.put(4.215, 40.0);
        velocityDistanceMap.put(4.42, 41.0);
        velocityDistanceMap.put(4.66, 43.0);
        velocityDistanceMap.put(4.82, 44.5);
        velocityDistanceMap.put(5.303, 44.75);
        velocityDistanceMap.put(5.49, 45.0);
        velocityDistanceMap.put(5.798, 46.0);
        velocityDistanceMap.put(6.0, 47.1);
        velocityDistanceMap.put(6.334, 49.0);
        velocityDistanceMap.put(6.738, 49.5);
        velocityDistanceMap.put(7.65, 53.0);
    }

    public Command setStateCommand(ShooterStates stateToSet) {
        return new InstantCommand(() -> this.currentState = stateToSet);
    }

    public Command setTurretPositionCommand(Supplier<Rotation2d> position) {
        return turretMechanism.setPositionCommand(position);
    }


    public Command defaultCommand() {
        Command defaultCommand = new ConditionalCommand(
                new ParallelCommandGroup(
                        setHoodAngleCommand(() -> 0),
                        flyWheelMechanism.setDynamicVelocityCommand(() -> {
                            flywheelVelocitySetpoint = () -> 0;
                            return 0;
                        }),
                        setTurretPositionCommand(Rotation2d::new)
                ).until(() -> !this.currentState.equals(IDLE)),
                new ParallelCommandGroup(
                        setAdjustedHoodAngleCommand(),
                        adjustFlyWheelVelocityCommand(),
                        setAdjustedTurretAngle()
                ).until(() -> this.currentState.equals(IDLE)),
                () -> this.currentState.equals(IDLE));
        defaultCommand.addRequirements(this);
        return defaultCommand;

    }

    public Command setAdjustedTurretAngle() {
        return setTurretPositionCommand(() -> getTurretToTargetVector().get().getAngle());
    }


    public Supplier<Translation2d> getTurretToTargetVector() {
        return () -> {

            ChassisSpeeds robotSpeeds = swerveSpeeds.get();

            Pose2d robotPose = robotPositionSupplier.get();
            Rotation2d robotRot = robotPose.getRotation();

            Translation2d turretField =
                    getTurretOnField().getTranslation();

            Translation2d fieldVector =
                    currentState.targetTranslation.get().minus(turretField);


            Translation2d turretToTarget = fieldVector.rotateBy(robotRot.unaryMinus());

//            return turretToTarget;
            Translation2d virtualTargetOffset = new Translation2d(
                    robotSpeeds.vxMetersPerSecond
                            - TURRET_OFFSET_TRANSLATION.getY() * robotSpeeds.omegaRadiansPerSecond,

                    robotSpeeds.vyMetersPerSecond
                            + TURRET_OFFSET_TRANSLATION.getX() * robotSpeeds.omegaRadiansPerSecond
            ).times(distanceTimeOfFlightMap.get(turretToTarget.getNorm()));


            Translation2d virtualTurretToTarget = turretToTarget.minus(virtualTargetOffset);
            return virtualTurretToTarget;
        };
    }


    @NT
    public Pose2d getTurretOnField() {
        Pose2d robotPose = robotPositionSupplier.get();
        Translation2d robotTranslation = robotPose.getTranslation();
        Rotation2d robotRot = robotPose.getRotation();

        // turret position in field coordinates
        Translation2d turretField =
                robotTranslation.plus(TURRET_OFFSET_TRANSLATION.rotateBy(robotRot));

        return new Pose2d(turretField, robotPositionSupplier.get().getRotation().minus(turretMechanism.getPosition().unaryMinus()));
    }

    public Command setHoodAngleCommand(DoubleSupplier angleSetpoint) {
        return new RunCommand(() -> hoodMechanism.setVoltage(getControlledOutputForAngle(() -> hoodSoftLimit.limit(angleSetpoint.getAsDouble()))));
    }


    public Command adjustFlyWheelVelocityCommand() {
        return flyWheelMechanism.setDynamicVelocityCommand(
                () -> {
                    double distance = turretRelativeDistanceFromTarget.getAsDouble();
                    double velocity = currentState.isShooting ? velocityDistanceMap.get(distance) : 0;
                    flywheelVelocitySetpoint = () -> velocity;
                    return velocity;
                }
        );
    }

    public Command setAdjustedHoodAngleCommand() {
        return new RunCommand(() -> {
            double distance = turretRelativeDistanceFromTarget.getAsDouble();
            hoodAngleSetpoint = () -> hoodSoftLimit.limit(angleDistanceMap.get(distance));
            hoodMechanism.setVoltage(getControlledOutputForAngle(() -> hoodSoftLimit.limit(angleDistanceMap.get(distance))));
        });
    }


    public double getControlledOutputForAngle(DoubleSupplier angleSetpoint) {
        double pid = angleController.calculate(hoodMotor.getMotorPosition(), angleSetpoint.getAsDouble());
        if (pid > 0) {
            return pid + Math.signum(pid) * 0.375; //ks positive
        }
        return pid + Math.signum(pid) * -0.25;
    }


    @NT
    public double getFlyWheelVelocitySetpoint() {
        return flywheelVelocitySetpoint.getAsDouble();
    }

    @NT
    public double getFlyWheelVelocity() {
        return flywheelVelocityFilter.getValue();
    }

    @NT
    public double getHoodAngleSetpoint() {
        return hoodAngleSetpoint.getAsDouble();
    }

    @NT
    public double getHoodAngleSupplier() {
        return hoodAngleSupplier.getAsDouble();
    }

    @NT
    public boolean flyWheelReadyTrigger() {
        return flyWheelReadyTrigger.getAsBoolean();
    }

    @NT
    public boolean hoodAdjustedTrigger() {
        return hoodAdjustedTrigger.getAsBoolean();
    }

    @NT
    public double getLimitedHoodAngle() {
        return hoodSoftLimit.limit(hoodAngleSetpoint.getAsDouble());
    }

    @NT
    public boolean volatileTrenchHoodTrigger() {
        return volatileTrenchHoodTrigger.getAsBoolean();
    }

    @NT
    public String getCurrentShooterState() {
        return currentState.name();
    }

    public Trigger isShooterReady() {
        return shooterReady;
    }

    @NT
    public Pose2d getHubOnFieldAfterCalc() {
        Pose2d turretOnField = getTurretOnField();
        return new Pose2d(turretOnField.getTranslation().plus(turretToHubVector.get().rotateBy(robotPositionSupplier.get().getRotation())), new Rotation2d());
    }

    @NT
    public double getDistanceFromHubTarget() {
        return getTurretToTargetVector().get().getNorm();
    }

    public Command setFlyWheelVelocity(DoubleSupplier rps) {
        return flyWheelMechanism.setDynamicVelocityCommand(rps);
    }

}