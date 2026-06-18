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
import frc.robot.util.TurretOffsetGetter;
import frc.robot.BallCounter;
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
import static frc.robot.subsystems.shooter.TargetHeight.HIGH;
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

    private final BallCounter ballCounter;

    private final InterpolatingDoubleTreeMap highAngleDistanceMap;
    private final InterpolatingDoubleTreeMap highVelocityDistanceMap;
    private final InterpolatingDoubleTreeMap highDistanceTimeOfFlightMap;

    private final InterpolatingDoubleTreeMap lowAngleDistanceMap;
    private final InterpolatingDoubleTreeMap lowVelocityDistanceMap;
    private final InterpolatingDoubleTreeMap lowDistanceTimeOfFlightMap;

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

        turretMotor.setIdleState(IdleState.COAST);
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

        highDistanceTimeOfFlightMap = new InterpolatingDoubleTreeMap();

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

        ballCounter = new BallCounter(
                () -> flywheelVelocitySetpoint.getAsDouble(),
                flywheelVelocityFilter::getValue
        );

        PeriodicScheduler.PERIOD.MILLISECONDS_20.add(flywheelVelocityFilter);

        robotPositionSupplier = poseSupplier;

        highAngleDistanceMap = new InterpolatingDoubleTreeMap();
        initAngleMap();

        highVelocityDistanceMap = new InterpolatingDoubleTreeMap();
        initVelocityMap();

        lowAngleDistanceMap = new InterpolatingDoubleTreeMap();
        lowDistanceTimeOfFlightMap = new InterpolatingDoubleTreeMap();
        lowVelocityDistanceMap = new InterpolatingDoubleTreeMap();

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

        initLowMaps();
        this.turretRelativeDistanceFromTarget = () -> getTurretToTargetVector().get().getNorm();

        shooterReady = isTurretAligned
                .and(flyWheelReadyTrigger)
                .and(hoodAdjustedTrigger);

//                .and(()-> getTurretToTargetVector().get().getNorm() < 3.5);


        TurretOffsetGetter.instance.setTurretOffsetSupplier(turretMechanism::getPosition);
        TurretOffsetGetter.instance.setTurretRotationalVelSup(turretMechanism::logVelocity);
        TurretOffsetGetter.instance.setRobotRotationalVel(() -> swerveSpeeds.get().omegaRadiansPerSecond);

        setDefaultCommand(defaultCommand().unless(() -> DISABLE_SUBSYSTEMS));
    }

    private void initDistanceTimeOfFlightMap() {
        // Populate empirical time-of-flight map for HIGH goal (fix A1/B1)
        // The commented values were empirical measurements — restore them so the lead model runs.
        highDistanceTimeOfFlightMap.put(0.0, 0.0);
        highDistanceTimeOfFlightMap.put(1.626, 0.746);
        highDistanceTimeOfFlightMap.put(2.04, 1.038);
        highDistanceTimeOfFlightMap.put(2.277, 1.116);
        highDistanceTimeOfFlightMap.put(2.321, 1.1433);
        highDistanceTimeOfFlightMap.put(2.595, 1.178);
        highDistanceTimeOfFlightMap.put(2.672, 1.166);
        highDistanceTimeOfFlightMap.put(2.87, 1.21);
        highDistanceTimeOfFlightMap.put(3.038, 1.144);
        highDistanceTimeOfFlightMap.put(3.419, 1.452);
        highDistanceTimeOfFlightMap.put(3.5, 1.165);
    }

    public void initAngleMap() {
        // Populate a basic HIGH hood-angle map so the hood moves with distance (fix A3)
        // These values are initial entries — replace with tuned calibration values when available.
        highAngleDistanceMap.put(1.77, 0.90);
        highAngleDistanceMap.put(1.89, 0.92);
        highAngleDistanceMap.put(2.08, 0.94);
        highAngleDistanceMap.put(2.23, 0.96);
        highAngleDistanceMap.put(2.45, 0.98);
        highAngleDistanceMap.put(2.74, 1.00);
        highAngleDistanceMap.put(2.83, 1.02);
        highAngleDistanceMap.put(3.00, 1.04);
    }

    public void initVelocityMap() {
//          velocityDistanceMapTable.put(distance[meters], flywheel velocity);
        highVelocityDistanceMap.put(1.77, 28.0);
        highVelocityDistanceMap.put(1.89, 29.0);
        highVelocityDistanceMap.put(2.08, 29.5);
        highVelocityDistanceMap.put(2.23, 30.0);
        highVelocityDistanceMap.put(2.45, 31.0);
        highVelocityDistanceMap.put(2.74, 31.0);
        highVelocityDistanceMap.put(2.83, 33.5);
        highVelocityDistanceMap.put(3.0, 34.0
        );
    }

    private void initLowMaps() {
        lowAngleDistanceMap.put(2.0, 1.0);
        lowAngleDistanceMap.put(3.0, 1.0);
        lowAngleDistanceMap.put(4.0, 0.925);
        lowAngleDistanceMap.put(5.0, 0.874);
        lowAngleDistanceMap.put(6.0, 0.841);
        lowAngleDistanceMap.put(7.0, 0.819);
        lowAngleDistanceMap.put(8.0, 0.802);
        lowAngleDistanceMap.put(9.0, 0.789);
        lowAngleDistanceMap.put(10.0, 0.779);

        lowVelocityDistanceMap.put(2.0, 13.25247);
        lowVelocityDistanceMap.put(3.0, 15.852848);
        lowVelocityDistanceMap.put(4.0, 18.51587);
        lowVelocityDistanceMap.put(5.0, 20.92826);
        lowVelocityDistanceMap.put(6.0, 23.09001);
        lowVelocityDistanceMap.put(7.0, 25.063791);
        lowVelocityDistanceMap.put(8.0, 26.91224);
        lowVelocityDistanceMap.put(9.0, 28.635382);
        lowVelocityDistanceMap.put(10.0, 30.26452);

        lowDistanceTimeOfFlightMap.put(2.0, 0.64);
        lowDistanceTimeOfFlightMap.put(3.0, 0.72);
        lowDistanceTimeOfFlightMap.put(4.0, 0.84);
        lowDistanceTimeOfFlightMap.put(5.0, 0.96);
        lowDistanceTimeOfFlightMap.put(6.0, 1.06);
        lowDistanceTimeOfFlightMap.put(7.0, 1.15);
        lowDistanceTimeOfFlightMap.put(8.0, 1.24);
        lowDistanceTimeOfFlightMap.put(9.0, 1.32);
        lowDistanceTimeOfFlightMap.put(10.0, 1.39);
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

            // Use a small fixed-point iteration to converge the (range <-> time-of-flight <-> lead) loop
            // instead of a single lookup. This implements B1.
            Translation2d aim = turretToTarget;
            for (int i = 0; i < 3; i++) {
                double tof = getInterpolatingTimeOfFlightMap().get(aim.getNorm());
                Translation2d virtualTargetOffset = new Translation2d(
                        robotSpeeds.vxMetersPerSecond - TURRET_OFFSET_TRANSLATION.getY() * robotSpeeds.omegaRadiansPerSecond,
                        robotSpeeds.vyMetersPerSecond + TURRET_OFFSET_TRANSLATION.getX() * robotSpeeds.omegaRadiansPerSecond
                ).times(tof);
                aim = turretToTarget.minus(virtualTargetOffset);
            }

            return aim;
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
                    double velocity = currentState.isShooting ? getInterpolatingVelocityMap().get(distance) : 0;
                    flywheelVelocitySetpoint = () -> velocity;
                    return velocity;
                }
        );
    }

    public double getWantedVelocity(){
        double distance = turretRelativeDistanceFromTarget.getAsDouble();
        double velocity = currentState.isShooting ? getInterpolatingVelocityMap().get(distance) : 0;
        return velocity;
    }

    public Command setAdjustedHoodAngleCommand() {
        return new RunCommand(() -> {
            double distance = turretRelativeDistanceFromTarget.getAsDouble();
            hoodAngleSetpoint = () -> hoodSoftLimit.limit(getInterpolatingAngleMap().get(distance));
            hoodMechanism.setVoltage(getControlledOutputForAngle(() -> hoodSoftLimit.limit(getInterpolatingAngleMap().get(distance))));
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

    public InterpolatingDoubleTreeMap getInterpolatingTimeOfFlightMap() {
        return currentState.targetHeight.equals(HIGH) ? highDistanceTimeOfFlightMap : lowDistanceTimeOfFlightMap;
    }

    public InterpolatingDoubleTreeMap getInterpolatingVelocityMap() {
        return currentState.targetHeight.equals(HIGH) ? highVelocityDistanceMap : lowVelocityDistanceMap;
    }

    public InterpolatingDoubleTreeMap getInterpolatingAngleMap() {
        return currentState.targetHeight.equals(HIGH) ? highAngleDistanceMap : lowAngleDistanceMap;
    }

    @NT
    public boolean isTurretAligned() {
        return isTurretAligned.getAsBoolean();
    }

    public Command yoavHatesThisCommandCommand(DoubleSupplier hoodAngleSupplier, DoubleSupplier flywheelVelocitySetpoint) {
        Command command = new ParallelCommandGroup(
                setFlyWheelVelocity(flywheelVelocitySetpoint),
                setHoodAngleCommand(hoodAngleSupplier),
                setAdjustedTurretAngle()
        );
        command.addRequirements(this);
        return command;
    }

    @NT
    public int getBallCount() {
        return ballCounter.getBallCount();
    }

    public void resetBallCount() {
        ballCounter.resetCount();
    }
}