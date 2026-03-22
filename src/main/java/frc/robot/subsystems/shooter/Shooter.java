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
import monologue.Annotations;
import monologue.Logged;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.excalib.additional_utilities.AllianceUtils.FIELD_LENGTH_METERS;
import static frc.excalib.additional_utilities.AllianceUtils.FIELD_WIDTH_METERS;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.PhysicalConstants.TURRET_OFFSET_TRANSLATION;
import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.shooter.ShooterStates.IDLE;
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

        currentState = IDLE;

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

        setDefaultCommand(defaultCommand());
    }

    public Command setStateCommand(ShooterStates stateToSet) {
        return new InstantCommand(() -> this.currentState = stateToSet, this);
    }

    public Command setTurretPositionCommand(Supplier<Rotation2d> position) {
        return turretMechanism.setPositionCommand(position, this);
    }

    private void initDistanceTimeOfFlightMap() {
        //        distanceFlightTimeTable.put(distance[meters], flight time);
        //        distanceTimeOfFlightMap.put(4.3, 1.39);
        //        distanceTimeOfFlightMap.put(2.99, 1.28);
        //        distanceTimeOfFlightMap.put(2.36, 1.21);
        //        distanceTimeOfFlightMap.put(3.6, 1.35);
        //        distanceTimeOfFlightMap.put(1.95, 1.12);
        //        distanceTimeOfFlightMap.put(0.0, 0.0);
        //        distanceTimeOfFlightMap.put(0.0, 0.0);
        distanceTimeOfFlightMap.put(5.036, 1.3);
        distanceTimeOfFlightMap.put(3.93, 1.2);
        distanceTimeOfFlightMap.put(2.8, 0.98);
    }

    public Command defaultCommand() {
        return new ConditionalCommand(
                new ParallelCommandGroup(
                        setHoodAngleCommand(() -> 0),
                        setFlyWheelDynamicVelocityCommand(() -> 0),
                        setTurretPositionCommand(Rotation2d::new)
                ),
                new ParallelCommandGroup(
                        setAdjustedHoodAngle(),
                        setAdjustedFlyWheelVelocity(),
                        setAdjustedTurretAngle()
                ),
                () -> this.currentState.equals(IDLE)
        );
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


    @Annotations.Log.NT
    public Pose2d getTurretOnField() {
        Pose2d robotPose = robotPositionSupplier.get();
        Translation2d robotTranslation = robotPose.getTranslation();
        Rotation2d robotRot = robotPose.getRotation();

        // turret position in field coordinates
        Translation2d turretField =
                robotTranslation.plus(TURRET_OFFSET_TRANSLATION.rotateBy(robotRot));

        return new Pose2d(turretField, robotPositionSupplier.get().getRotation().minus(turretMechanism.getPosition().unaryMinus()));
    }


    public void initAngleMap() {
//        angleDistanceMapTable.put(distance[meters], hood angle);
//        angleDistanceMap.put(1.947, 0.0);
//        angleDistanceMap.put(2.58, 0.08);
//        angleDistanceMap.put(3.96, 0.225);
//        angleDistanceMap.put(5.07, 0.4);
//        angleDistanceMap.put(3.48, 0.18);
//        angleDistanceMap.put(3.19, 0.13);
//        angleDistanceMap.put(4.0, 0.22);
//        angleDistanceMap.put(4.81, 0.32);
//        angleDistanceMap.put(5.7, 0.6);


        angleDistanceMap.put(3.36, 0.2);
        angleDistanceMap.put(2.52, 0.15);
        angleDistanceMap.put(2.57, 0.25);
        angleDistanceMap.put(2.08, 0.05);
        angleDistanceMap.put(2.98, 0.25);
        angleDistanceMap.put(3.6, 0.25);
        angleDistanceMap.put(4.8, 0.37);


    }

    public void initVelocityMap() {
//          velocityDistanceMapTable.put(distance[meters], flywheel velocity);
//        velocityDistanceMap.put(1.948, 35.0);
//        velocityDistanceMap.put(2.58, 37.3);
//        velocityDistanceMap.put(3.96, 41.0);
//        velocityDistanceMap.put(5.07, 44.0);
//        velocityDistanceMap.put(3.48, 39.5);
//        velocityDistanceMap.put(3.19, 39.0);
//        velocityDistanceMap.put(4.0, 43.0);
//        velocityDistanceMap.put(4.81, 44.0);
//        velocityDistanceMap.put(5.7, 49.0);

        velocityDistanceMap.put(3.36, 36.0);
        velocityDistanceMap.put(2.52, 34.0);
        velocityDistanceMap.put(3.57, 38.0);
        velocityDistanceMap.put(2.08, 33.0);
        velocityDistanceMap.put(2.98, 34.0);
        velocityDistanceMap.put(3.6, 37.0);
        velocityDistanceMap.put(4.8, 45.0);
    }

    public Command setHoodAngleCommand(DoubleSupplier angleSetpoint) {
        return new RunCommand(() -> hoodMechanism.setVoltage(getControlledOutputForAngle(() -> hoodSoftLimit.limit(angleSetpoint.getAsDouble()))), this);
    }

    public void setFlyWheelDynamicVelocity(DoubleSupplier vel, SubsystemBase... req) {
        flywheelVelocitySetpoint = vel;
        flyWheelMechanism.setDynamicVelocityCommand(vel, req).alongWith(new PrintCommand("" + flywheelVelocitySetpoint.getAsDouble()));
    }

    public Command setFlyWheelDynamicVelocityCommand(DoubleSupplier vel, SubsystemBase... req) {
        return new RunCommand(() -> setFlyWheelDynamicVelocity(vel, req));
    }

    public Command setAdjustedFlyWheelVelocity() {
        return new RunCommand(() -> {
            double distance = turretRelativeDistanceFromTarget.getAsDouble();
            double velocity = velocityDistanceMap.get(distance);
            flywheelVelocitySetpoint = () -> velocity;
            setFlyWheelDynamicVelocity(flywheelVelocitySetpoint);
        });
    }

    public Command setAdjustedHoodAngle() {
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

    public Trigger isShooterReady(){
        return shooterReady;
    }
}