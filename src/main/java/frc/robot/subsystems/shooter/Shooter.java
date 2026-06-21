package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.lib.FieldZones;
import frc.robot.lib.StateMachineSubsystem;
import frc.robot.util.BallCounter;
import frc.robot.util.TurretOffsetGetter;
import monologue.Annotations.Log;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.DISABLE_SUBSYSTEMS;
import static frc.robot.Constants.PhysicalConstants.TURRET_OFFSET_TRANSLATION;
import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.shooter.ShooterStates.IDLE;
import static frc.robot.subsystems.shooter.ShooterStates.LOOK_HUB;

/**
 * Turret-mounted shooter: turret aiming with shot-lead compensation, a velocity-controlled flywheel,
 * and a profiled hood. Setpoints come from distance-keyed interpolation tables selected by the active
 * {@link ShooterStates} (HIGH vs LOW goal), and the {@link #shooterReady} trigger composes the three
 * mechanism-ready conditions.
 */
public class Shooter extends StateMachineSubsystem<ShooterStates> {

    private final TalonFXMotor hoodMotor, flyWheelMotorTop, flyWheelMotorLow, turretMotor;
    private final MotorGroup shooterMotorGroup;
    private final CANcoder hoodEncoder, turretEncoder;
    private final PIDController angleController;

    private final Turret turretMechanism;
    private final FlyWheel flyWheelMechanism;
    private final Mechanism hoodMechanism;

    private final DoubleSupplier turretAngleSupplier;
    private final DoubleSupplier turretRelativeAngleToTarget;
    private final DoubleSupplier hoodAngleSupplier;
    private final DoubleSupplier turretRelativeDistanceFromTarget;

    private final SoftLimit hoodSoftLimit;
    private final FieldZones hoodTrenchZones;

    private final Supplier<Pose2d> robotPositionSupplier;
    private final Supplier<ChassisSpeeds> swerveSpeeds;

    /** Single source of truth for the commanded setpoints, written each loop by the active command. */
    private double flywheelVelocitySetpointRps = 0;
    private double hoodAngleSetpointRad = 0;

    private final EMAFilter flywheelVelocityFilter;
    private final BallCounter ballCounter;

    private final InterpolatingDoubleTreeMap highAngleDistanceMap;
    private final InterpolatingDoubleTreeMap highVelocityDistanceMap;
    private final InterpolatingDoubleTreeMap highDistanceTimeOfFlightMap;
    private final InterpolatingDoubleTreeMap lowAngleDistanceMap;
    private final InterpolatingDoubleTreeMap lowVelocityDistanceMap;
    private final InterpolatingDoubleTreeMap lowDistanceTimeOfFlightMap;

    private final Trigger flyWheelReadyTrigger;
    private final Trigger hoodAdjustedTrigger;
    private final Trigger isTurretAligned;
    private final Trigger activateLedsTrigger;
    private final Trigger shooterReady;

    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> swerveSpeeds) {
        super(LOOK_HUB);
        this.robotPositionSupplier = poseSupplier;
        this.swerveSpeeds = swerveSpeeds;

        hoodMotor = new TalonFXMotor(HOOD_MOTOR_ID, SUBSYSTEMS_CANBUS);
        hoodMotor.setCurrentLimit(40, 30);
        flyWheelMotorLow = new TalonFXMotor(FLYWHEEL_MOTOR_LOW_ID, SUBSYSTEMS_CANBUS);
        flyWheelMotorTop = new TalonFXMotor(FLYWHEEL_MOTOR_TOP_ID, SUBSYSTEMS_CANBUS);
        hoodEncoder = new CANcoder(HOOD_ENCODER_ID, SUBSYSTEMS_CANBUS);

        shooterMotorGroup = new MotorGroup(flyWheelMotorLow, flyWheelMotorTop);
        shooterMotorGroup.setIdleState(IdleState.BRAKE);
        shooterMotorGroup.setMotorPosition(0);
        shooterMotorGroup.setVelocityConversionFactor(FLYWHEEL_GROUP_CONVERSION_FACTOR);
        shooterMotorGroup.setPositionConversionFactor(FLYWHEEL_GROUP_CONVERSION_FACTOR);
        flyWheelMotorLow.setInverted(DirectionState.FORWARD);
        flyWheelMotorTop.setInverted(DirectionState.FORWARD);
        flyWheelMotorTop.setCurrentLimit(120, 80);
        flyWheelMotorLow.setCurrentLimit(120, 80);

        turretMotor = new TalonFXMotor(TURRET_MOTOR_ID, SUBSYSTEMS_CANBUS);
        turretEncoder = new CANcoder(TURRET_ENCODER_ID, SUBSYSTEMS_CANBUS);
        turretEncoder.setPosition(turretEncoder.getAbsolutePosition().getValueAsDouble());
        turretAngleSupplier = () -> turretEncoder.getPosition().getValueAsDouble() * ENCODER_POSITION_CONVERSION_FACTOR;
        turretMotor.setMotorPosition(turretEncoder.getPosition().getValueAsDouble());
        turretMotor.setCurrentLimit(120, 80);
        turretRelativeAngleToTarget = () -> getTurretToTargetVector().get().getAngle().getRadians();
        turretMotor.setInverted(DirectionState.REVERSE);
        turretMotor.setIdleState(IdleState.COAST);
        turretMotor.setMotorPosition(turretAngleSupplier.getAsDouble());
        turretMotor.setPositionConversionFactor(MOTOR_POSITION_CONVERSION_FACTOR);
        turretMotor.setVelocityConversionFactor(MOTOR_POSITION_CONVERSION_FACTOR);

        turretMechanism = new Turret(
                turretMotor,
                TURRET_CONTINUOUS_SOFTLIMIT,
                TURRET_GAINS,
                PID_TOLERANCE,
                turretMotor::getMotorPosition,
                new TrapezoidProfile.Constraints(TURRET_PROFILE_MAX_VELOCITY, TURRET_PROFILE_MAX_ACCELERATION)
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

        angleController = new PIDController(HOOD_GAINS.kp, HOOD_GAINS.ki, HOOD_GAINS.kd);
        angleController.setTolerance(HOOD_PID_TOLERANCE);

        hoodMotor.setIdleState(IdleState.BRAKE);
        hoodMotor.setInverted(DirectionState.FORWARD);
        hoodAngleSupplier = () -> (hoodEncoder.getPosition().getValueAsDouble() * POSITION_CONVERSION_FACTOR);
        hoodMotor.setPositionConversionFactor(HOOD_MOTOR_POSITION_CONVERSION_FACTOR);
        hoodMotor.setMotorPosition(hoodAngleSupplier.getAsDouble());
        hoodMotor.setIdleState(IdleState.COAST);
        hoodMechanism = new Mechanism(hoodMotor);

        flyWheelMechanism = new FlyWheel(shooterMotorGroup, FLYWHEEL_MAX_ACCELERATION, FLYWHEEL_MAX_JERK, FLYWHEEL_GAINS);

        flywheelVelocityFilter = new EMAFilter(
                flyWheelMechanism::getVelocity,
                FLYWHEEL_EMA_ALPHA,
                PeriodicScheduler.PERIOD.MILLISECONDS_20);
        PeriodicScheduler.PERIOD.MILLISECONDS_20.add(flywheelVelocityFilter);

        ballCounter = new BallCounter(
                () -> flywheelVelocitySetpointRps,
                flywheelVelocityFilter::getValue
        );

        highAngleDistanceMap = new InterpolatingDoubleTreeMap();
        highVelocityDistanceMap = new InterpolatingDoubleTreeMap();
        highDistanceTimeOfFlightMap = new InterpolatingDoubleTreeMap();
        lowAngleDistanceMap = new InterpolatingDoubleTreeMap();
        lowVelocityDistanceMap = new InterpolatingDoubleTreeMap();
        lowDistanceTimeOfFlightMap = new InterpolatingDoubleTreeMap();
        initAngleMap();
        initVelocityMap();
        initDistanceTimeOfFlightMap();
        initLowMaps();

        flyWheelReadyTrigger = new Trigger(
                () -> Math.abs(flywheelVelocityFilter.getValue() - flywheelVelocitySetpointRps) < FLYWHEEL_TOLERANCE);
        hoodAdjustedTrigger = new Trigger(
                () -> Math.abs(hoodAngleSupplier.getAsDouble() - hoodAngleSetpointRad) < HOOD_TOLERANCE);

        activateLedsTrigger = new Trigger(() -> flyWheelMechanism.getVelocity() > LED_ACTIVATION_VELOCITY);
        activateLedsTrigger.onTrue(LEDs.getInstance().setPattern(LEDs.LEDPattern.BLINKING, Color.Colors.ORANGE.color)
                .andThen(LEDs.getInstance().restoreLEDs()));

        // Hood must stay low while physically inside the trench; trench geometry is owned by FieldZones.
        hoodTrenchZones = new FieldZones(poseSupplier);
        hoodSoftLimit = new SoftLimit(() -> HOOD_MIN_ANGLE_LIMIT, () ->
                hoodTrenchZones.inTrench().getAsBoolean() ? HOOD_MAX_ANGLE_LIMIT_IN_TRENCH : HOOD_MAX_ANGLE_LIMIT);

        turretRelativeDistanceFromTarget = () -> getTurretToTargetVector().get().getNorm();

        shooterReady = isTurretAligned
                .and(flyWheelReadyTrigger)
                .and(hoodAdjustedTrigger);

        TurretOffsetGetter.instance.setTurretOffsetSupplier(turretMechanism::getPosition);
        TurretOffsetGetter.instance.setTurretRotationalVelSup(turretMechanism::logVelocity);
        TurretOffsetGetter.instance.setRobotRotationalVel(() -> swerveSpeeds.get().omegaRadiansPerSecond);

        setDefaultCommand(defaultCommand().unless(() -> DISABLE_SUBSYSTEMS));
    }

    private void initDistanceTimeOfFlightMap() {
        // Populate empirical time-of-flight map for HIGH goal (fix A1/B1)
        // The commented values were empirical measurements — restore them so the lead model runs.
        highDistanceTimeOfFlightMap.put(0.0, 0.0); //currently not used because of testing, DONT TOUCH!
    }

    private void initAngleMap() {
        highAngleDistanceMap.put(0.00, 0.0);
    }

    private void initVelocityMap() {
//          velocityDistanceMapTable.put(distance[meters], flywheel velocity);
        highVelocityDistanceMap.put(1.77, 28.0);
        highVelocityDistanceMap.put(1.89, 29.0);
        highVelocityDistanceMap.put(2.08, 29.5);
        highVelocityDistanceMap.put(2.23, 30.0);
        highVelocityDistanceMap.put(2.45, 31.0);
        highVelocityDistanceMap.put(2.74, 31.0);
        highVelocityDistanceMap.put(2.83, 33.5);
        highVelocityDistanceMap.put(3.0, 34.0);
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

    public Command defaultCommand() {
        Command defaultCommand = new ConditionalCommand(
                new ParallelCommandGroup(
                        setHoodAngleCommand(() -> 0),
                        flyWheelMechanism.setDynamicVelocityCommand(() -> {
                            flywheelVelocitySetpointRps = 0;
                            return 0;
                        }),
                        setTurretPositionCommand(Rotation2d::new)
                ).until(() -> state() != IDLE),
                new ParallelCommandGroup(
                        setAdjustedHoodAngleCommand(),
                        adjustFlyWheelVelocityCommand(),
                        setAdjustedTurretAngle()
                ).until(() -> state() == IDLE),
                () -> state() == IDLE);
        defaultCommand.addRequirements(this);
        return defaultCommand;
    }

    // ===== Manual override =====

    /** Drive all three mechanisms from external setpoints (turret still auto-aims). */
    public Command manualShoot(DoubleSupplier hoodAngle, DoubleSupplier flywheelVelocityRps) {
        Command command = new ParallelCommandGroup(
                setFlyWheelVelocity(flywheelVelocityRps),
                setHoodAngleCommand(hoodAngle),
                setAdjustedTurretAngle()
        );
        command.addRequirements(this);
        return command;
    }

    public Command coastCommand() {
        return turretMechanism.coastCommand(this);
    }

    public Command setFlyWheelVelocity(DoubleSupplier rps) {
        return flyWheelMechanism.setDynamicVelocityCommand(rps);
    }

    // ===== Turret =====

    private Command setTurretPositionCommand(Supplier<Rotation2d> position) {
        return turretMechanism.setPositionCommand(position);
    }

    public Command setAdjustedTurretAngle() {
        return setTurretPositionCommand(() -> getTurretToTargetVector().get().getAngle());
    }

    private Supplier<Translation2d> getTurretToTargetVector() {
        return () -> {
            ChassisSpeeds robotSpeeds = swerveSpeeds.get();

            Pose2d robotPose = robotPositionSupplier.get();
            Rotation2d robotRot = robotPose.getRotation();

            Translation2d turretField = getTurretOnField().getTranslation();
            Translation2d fieldVector = state().target().get().minus(turretField);
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

    @Log.NT
    public Pose2d getTurretOnField() {
        Pose2d robotPose = robotPositionSupplier.get();
        Translation2d robotTranslation = robotPose.getTranslation();
        Rotation2d robotRot = robotPose.getRotation();

        // turret position in field coordinates
        Translation2d turretField = robotTranslation.plus(TURRET_OFFSET_TRANSLATION.rotateBy(robotRot));

        return new Pose2d(turretField, robotPositionSupplier.get().getRotation().minus(turretMechanism.getPosition().unaryMinus()));
    }

    // ===== Hood =====

    private Command setHoodAngleCommand(DoubleSupplier angleSetpoint) {
        return new RunCommand(() -> hoodMechanism.setVoltage(getControlledOutputForAngle(() -> hoodSoftLimit.limit(angleSetpoint.getAsDouble()))));
    }

    private Command setAdjustedHoodAngleCommand() {
        return new RunCommand(() -> {
            double distance = turretRelativeDistanceFromTarget.getAsDouble();
            double target = hoodSoftLimit.limit(getInterpolatingAngleMap().get(distance));
            hoodAngleSetpointRad = target;
            hoodMechanism.setVoltage(getControlledOutputForAngle(() -> target));
        });
    }

    private double getControlledOutputForAngle(DoubleSupplier angleSetpoint) {
        double pid = angleController.calculate(hoodMotor.getMotorPosition(), angleSetpoint.getAsDouble());
        if (pid > 0) {
            return pid + Math.signum(pid) * HOOD_KS_RAISING;
        }
        return pid + Math.signum(pid) * HOOD_KS_LOWERING;
    }

    // ===== Flywheel =====

    private Command adjustFlyWheelVelocityCommand() {
        return flyWheelMechanism.setDynamicVelocityCommand(
                () -> {
                    double distance = turretRelativeDistanceFromTarget.getAsDouble();
                    double velocity = state().isShooting() ? getWantedVelocityForDistance(distance) : 0;
                    flywheelVelocitySetpointRps = velocity;
                    return velocity;
                }
        );
    }

    private double getWantedVelocityForDistance(double distance) {
        double nominalVelocity = getInterpolatingVelocityMap().get(distance);
        Translation2d turretToTarget = getTurretToTargetVector().get();
        double range = turretToTarget.getNorm();
        if (range < 1e-9) {
            return nominalVelocity;
        }

        Translation2d unitToTarget = turretToTarget.div(range);
        ChassisSpeeds robotSpeeds = swerveSpeeds.get();
        Translation2d robotVelocity = new Translation2d(
                robotSpeeds.vxMetersPerSecond - TURRET_OFFSET_TRANSLATION.getY() * robotSpeeds.omegaRadiansPerSecond,
                robotSpeeds.vyMetersPerSecond + TURRET_OFFSET_TRANSLATION.getX() * robotSpeeds.omegaRadiansPerSecond
        );

        double radialVelocity = robotVelocity.getX() * unitToTarget.getX() + robotVelocity.getY() * unitToTarget.getY();
        return nominalVelocity - radialVelocity;
    }

    // ===== Interpolation table selection (HIGH vs LOW goal) =====

    private InterpolatingDoubleTreeMap getInterpolatingTimeOfFlightMap() {
        return state().isHighGoal() ? highDistanceTimeOfFlightMap : lowDistanceTimeOfFlightMap;
    }

    private InterpolatingDoubleTreeMap getInterpolatingVelocityMap() {
        return state().isHighGoal() ? highVelocityDistanceMap : lowVelocityDistanceMap;
    }

    private InterpolatingDoubleTreeMap getInterpolatingAngleMap() {
        return state().isHighGoal() ? highAngleDistanceMap : lowAngleDistanceMap;
    }

    // ===== Readiness =====

    public Trigger isShooterReady() {
        return shooterReady;
    }

    // ===== Ball counting =====

    @Log.NT
    public int getBallCount() {
        return ballCounter.getBallCount();
    }

    public void resetBallCount() {
        ballCounter.resetCount();
    }

    // ===== Telemetry =====

    @Log.NT
    public double getFlyWheelVelocitySetpoint() {
        return flywheelVelocitySetpointRps;
    }

    @Log.NT
    public double getFlyWheelVelocity() {
        return flywheelVelocityFilter.getValue();
    }

    @Log.NT
    public double getHoodAngleSetpoint() {
        return hoodAngleSetpointRad;
    }

    @Log.NT
    public double getHoodAngleSupplier() {
        return hoodAngleSupplier.getAsDouble();
    }

    @Log.NT
    public boolean flyWheelReadyTrigger() {
        return flyWheelReadyTrigger.getAsBoolean();
    }

    @Log.NT
    public boolean hoodAdjustedTrigger() {
        return hoodAdjustedTrigger.getAsBoolean();
    }

    @Log.NT
    public double getLimitedHoodAngle() {
        return hoodSoftLimit.limit(hoodAngleSetpointRad);
    }

    @Log.NT
    public boolean volatileTrenchHoodTrigger() {
        return hoodTrenchZones.inTrench().getAsBoolean();
    }

    @Log.NT
    public String getCurrentShooterState() {
        return currentStateName();
    }

    @Log.NT
    public Pose2d getHubOnFieldAfterCalc() {
        Pose2d turretOnField = getTurretOnField();
        return new Pose2d(turretOnField.getTranslation().plus(getTurretToTargetVector().get().rotateBy(robotPositionSupplier.get().getRotation())), new Rotation2d());
    }

    @Log.NT
    public double getDistanceFromHubTarget() {
        return getTurretToTargetVector().get().getNorm();
    }

    @Log.NT
    public boolean isTurretAligned() {
        return isTurretAligned.getAsBoolean();
    }
}
