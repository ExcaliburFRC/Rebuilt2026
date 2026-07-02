package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.additional_utilities.Color;
import frc.excalib.additional_utilities.LEDs;
import frc.excalib2.control.ControlMode;
import frc.excalib2.device.CANDeviceId;
import frc.excalib2.mechanisms.MechanismConfig;
import frc.excalib2.mechanisms.PositionalMechanism;
import frc.excalib2.mechanisms.VelocityMechanism;
import frc.excalib2.statemachine.StateMachine;
import frc.robot.util.BallCounter;
import frc.robot.util.TurretOffsetGetter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Radians;
import static frc.excalib.additional_utilities.AllianceUtils.FIELD_LENGTH_METERS;
import static frc.excalib.additional_utilities.AllianceUtils.FIELD_WIDTH_METERS;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.PhysicalConstants.TURRET_OFFSET_TRANSLATION;
import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.shooter.ShooterStates.IDLE;
import static frc.robot.subsystems.shooter.TargetHeight.HIGH;

/**
 * Shooter on ExcaLib v2: turret + hood as {@link PositionalMechanism}s (turret with
 * continuous-wrap goals inside its ±limits, hood with a trench-aware dynamic max) and the
 * two-Kraken flywheel as a {@link VelocityMechanism}, coordinated by a state machine over
 * {@link ShooterStates}.
 *
 * <p>The proven competition logic is preserved unchanged: interpolation maps
 * (distance → hood angle / flywheel velocity / time of flight, HIGH and LOW goals),
 * the 3-iteration shot-lead fixed point, moving-shot radial-velocity compensation, and
 * the flywheel-dip {@link BallCounter}.
 */
public class Shooter extends SubsystemBase {
    private static final double EMA_ALPHA = 0.05; // v1 flywheel velocity filter

    // ── Mechanisms ───────────────────────────────────────────────────────────

    private final VelocityMechanism flywheel = new VelocityMechanism(
            MechanismConfig.of("Shooter/Flywheel",
                            new CANDeviceId(FLYWHEEL_MOTOR_LOW_ID, SUBSYSTEMS_CANBUS.getName()))
                    .follower(new CANDeviceId(FLYWHEEL_MOTOR_TOP_ID, SUBSYSTEMS_CANBUS.getName()), false)
                    .controlMode(ControlMode.VOLTAGE) // ported v1 volts gains; FOC after SysId
                    .rotorToMechanismRatio(FLYWHEEL_ROTOR_PER_MECHANISM)
                    .gains(FLYWHEEL_GAINS, FLYWHEEL_SIM_GAINS)
                    .currentBudget(FLYWHEEL_BUDGET)
                    .tolerance(Radians.of(FLYWHEEL_TOLERANCE_RPS)) // rot/s tolerance
                    .simModel(FLYWHEEL_SIM_MODEL));

    private final PositionalMechanism hood = new PositionalMechanism(
            MechanismConfig.of("Shooter/Hood",
                            new CANDeviceId(HOOD_MOTOR_ID, SUBSYSTEMS_CANBUS.getName()))
                    .controlMode(ControlMode.VOLTAGE)
                    .rotorToMechanismRatio(HOOD_ROTOR_PER_MECHANISM)
                    .gains(HOOD_GAINS, HOOD_SIM_GAINS)
                    .softLimits(Radians.of(HOOD_MIN_ANGLE_RAD), Radians.of(HOOD_MAX_ANGLE_RAD))
                    .currentBudget(HOOD_BUDGET)
                    .tolerance(Radians.of(HOOD_TOLERANCE_RAD))
                    .neutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Coast)
                    .simModel(HOOD_SIM_MODEL));

    private final PositionalMechanism turret = new PositionalMechanism(
            MechanismConfig.of("Shooter/Turret",
                            new CANDeviceId(TURRET_MOTOR_ID, SUBSYSTEMS_CANBUS.getName()))
                    .controlMode(ControlMode.VOLTAGE)
                    .inverted(true) // v1: REVERSE
                    .rotorToMechanismRatio(TURRET_ROTOR_PER_MECHANISM)
                    .gains(TURRET_GAINS, TURRET_SIM_GAINS)
                    .motion(TURRET_MOTION)
                    .softLimits(Radians.of(TURRET_MIN_ANGLE_RAD), Radians.of(TURRET_MAX_ANGLE_RAD))
                    .continuousWrap()
                    .currentBudget(TURRET_BUDGET)
                    .tolerance(Radians.of(TURRET_TOLERANCE_RAD))
                    .neutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Coast)
                    .simModel(TURRET_SIM_MODEL));

    // ── Targeting state ──────────────────────────────────────────────────────

    private final Supplier<Pose2d> robotPositionSupplier;
    private final Supplier<ChassisSpeeds> swerveSpeeds;
    private final StateMachine<ShooterStates> machine;

    private final InterpolatingDoubleTreeMap highAngleDistanceMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap highVelocityDistanceMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap highDistanceTimeOfFlightMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap lowAngleDistanceMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap lowVelocityDistanceMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap lowDistanceTimeOfFlightMap = new InterpolatingDoubleTreeMap();

    private double flywheelVelocitySetpoint = 0;
    private double hoodAngleSetpoint = 0;
    private double filteredFlywheelVelocity = 0;

    private final BallCounter ballCounter;

    // ── Triggers (v1 semantics preserved) ────────────────────────────────────

    public final Trigger isTurretAligned;
    private final Trigger flyWheelReadyTrigger;
    private final Trigger hoodAdjustedTrigger;
    private final Trigger volatileTrenchHoodTrigger;
    public final Trigger shooterReady;

    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> swerveSpeeds) {
        this.robotPositionSupplier = poseSupplier;
        this.swerveSpeeds = swerveSpeeds;

        // Seed mechanism positions from the absolute encoders (v1 boot behavior; the
        // CANcoders are not yet fused as device feedback — planned follow-up).
        CANcoder hoodEncoder = new CANcoder(HOOD_ENCODER_ID, SUBSYSTEMS_CANBUS);
        CANcoder turretEncoder = new CANcoder(TURRET_ENCODER_ID, SUBSYSTEMS_CANBUS);
        hood.resetPosition(Radians.of(
                hoodEncoder.getAbsolutePosition().getValueAsDouble() * HOOD_RAD_PER_ENCODER_ROTATION));
        turret.resetPosition(Radians.of(
                turretEncoder.getAbsolutePosition().getValueAsDouble() * TURRET_RAD_PER_ENCODER_ROTATION));

        initAngleMap();
        initVelocityMap();
        initDistanceTimeOfFlightMap();
        initLowMaps();

        machine = StateMachine.builder("Shooter", ShooterStates.LOOK_HUB)
                .whileIn(IDLE, idleCommand())
                .whileIn(ShooterStates.SHOOT_HUB, trackCommand())
                .whileIn(ShooterStates.LOOK_HUB, trackCommand())
                .whileIn(ShooterStates.SHOOT_FAR_DELIVERY, trackCommand())
                .whileIn(ShooterStates.LOOK_FAR_DELIVERY, trackCommand())
                .whileIn(ShooterStates.SHOOT_CLOSE_DELIVERY, trackCommand())
                .whileIn(ShooterStates.LOOK_CLOSE_DELIVERY, trackCommand())
                .transitionFromAny(IDLE)
                .transitionFromAny(ShooterStates.SHOOT_HUB)
                .transitionFromAny(ShooterStates.LOOK_HUB)
                .transitionFromAny(ShooterStates.SHOOT_FAR_DELIVERY)
                .transitionFromAny(ShooterStates.LOOK_FAR_DELIVERY)
                .transitionFromAny(ShooterStates.SHOOT_CLOSE_DELIVERY)
                .transitionFromAny(ShooterStates.LOOK_CLOSE_DELIVERY)
                .build();

        isTurretAligned = turret.atGoal;
        flyWheelReadyTrigger = new Trigger(
                () -> Math.abs(filteredFlywheelVelocity - flywheelVelocitySetpoint) < FLYWHEEL_TOLERANCE_RPS);
        hoodAdjustedTrigger = new Trigger(
                () -> Math.abs(hood.getPosition().in(Radians) - hoodAngleSetpoint) < HOOD_TOLERANCE_RAD);
        shooterReady = isTurretAligned.and(flyWheelReadyTrigger).and(hoodAdjustedTrigger);

        volatileTrenchHoodTrigger = new Trigger(() -> {
            Pose2d pose = poseSupplier.get();
            if (AllianceUtils.isBlueAlliance()) {
                return pose.getX() > FRONT_TRENCH_SIDEX_LINE_DIST_METERS
                        && pose.getY() < TRENCH_SIDEY_LINE_DIST_METERS
                        && pose.getX() < BACK_TRENCH_SIDEX_LINE_DIST_METERS;
            }
            return pose.getX() < FIELD_LENGTH_METERS - FRONT_TRENCH_SIDEX_LINE_DIST_METERS
                    && pose.getY() > FIELD_WIDTH_METERS - TRENCH_SIDEY_LINE_DIST_METERS
                    && pose.getX() > FIELD_LENGTH_METERS - BACK_TRENCH_SIDEX_LINE_DIST_METERS;
        });

        ballCounter = new BallCounter(
                () -> flywheelVelocitySetpoint,
                () -> filteredFlywheelVelocity);

        // v1 LED behavior: blink orange while the flywheel spins
        Trigger activateLeds = new Trigger(() -> flywheel.getVelocityRotationsPerSecond() > 3);
        activateLeds.onTrue(LEDs.getInstance()
                .setPattern(LEDs.LEDPattern.BLINKING, Color.Colors.ORANGE.color)
                .andThen(LEDs.getInstance().restoreLEDs()));

        // Vision pose conversion still needs the turret angle (consumed by swerve)
        TurretOffsetGetter.instance.setTurretOffsetSupplier(this::getTurretRotation);
        TurretOffsetGetter.instance.setTurretRotationalVelSup(
                () -> turret.getVelocityRotationsPerSecond() * 2 * Math.PI);
        TurretOffsetGetter.instance.setRobotRotationalVel(
                () -> swerveSpeeds.get().omegaRadiansPerSecond);
    }

    // ── Interpolation maps (tuning data — DO NOT reformat; edit values only) ─

    private void initDistanceTimeOfFlightMap() {
        highDistanceTimeOfFlightMap.put(0.0, 0.0);
        highDistanceTimeOfFlightMap.put(4.01, 1.2);
        highDistanceTimeOfFlightMap.put(4.72, 1.21);
        highDistanceTimeOfFlightMap.put(5.49, 2.02);
    }

    public void initAngleMap() {
        highAngleDistanceMap.put(0.00, 0.0);
        highAngleDistanceMap.put(2.00, 0.0);
        highAngleDistanceMap.put(4.01, 0.3);
        highAngleDistanceMap.put(4.72, 0.5);
        highAngleDistanceMap.put(5.49, 0.5);
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
        highVelocityDistanceMap.put(3.0, 34.0);
        highVelocityDistanceMap.put(4.01, 38.0);
        highVelocityDistanceMap.put(4.72, 38.0);
        highVelocityDistanceMap.put(5.49, 43.0);
    }

    private void initLowMaps() {
        // Flatter than the high-goal map on purpose -> lower apex.
        // angle decreases slightly with distance to hold apex roughly constant.
        lowAngleDistanceMap.put(2.0, 0.7);
        lowAngleDistanceMap.put(3.0, 0.7);
        lowAngleDistanceMap.put(4.0, 0.6);
        lowAngleDistanceMap.put(5.0, 0.5);
        lowAngleDistanceMap.put(6.0, 0.5);

        // Comparable to high-goal speeds (flatter shot needs the horizontal reach).
        // This is the easy empirical knob: short -> raise, long -> lower.
        lowVelocityDistanceMap.put(2.0, 22.0);
        lowVelocityDistanceMap.put(3.0, 26.0);
        lowVelocityDistanceMap.put(4.0, 31.0);
        lowVelocityDistanceMap.put(5.0, 36.0);
        lowVelocityDistanceMap.put(6.0, 41.0);

        // Flat shot -> shorter hang time than a high arc. MEASURE these on-robot.
        lowDistanceTimeOfFlightMap.put(2.0, 0.30);
        lowDistanceTimeOfFlightMap.put(3.0, 0.42);
        lowDistanceTimeOfFlightMap.put(4.0, 0.54);
        lowDistanceTimeOfFlightMap.put(5.0, 0.66);
        lowDistanceTimeOfFlightMap.put(6.0, 0.78);
    }

    // ── State behavior ───────────────────────────────────────────────────────

    private Command idleCommand() {
        return run(() -> {
            setHoodAngle(0);
            setFlywheelVelocity(0);
            turret.setGoal(Radians.of(0));
        }).withName("Shooter Idle");
    }

    private Command trackCommand() {
        return run(() -> {
            double distance = getTurretToTargetVector().get().getNorm();
            ShooterStates state = machine.getCurrentState();
            setHoodAngle(getInterpolatingAngleMap().get(distance));
            setFlywheelVelocity(state.isShooting ? getWantedVelocityForDistance(distance) : 0);
            turret.setGoal(Radians.of(getTurretToTargetVector().get().getAngle().getRadians()));
        }).withName("Shooter Track");
    }

    /** Hood goal with the trench-aware dynamic maximum (v1 volatile trench limit). */
    private void setHoodAngle(double angleRadians) {
        double max = volatileTrenchHoodTrigger.getAsBoolean()
                ? HOOD_MAX_ANGLE_IN_TRENCH_RAD : HOOD_MAX_ANGLE_RAD;
        hoodAngleSetpoint = Math.max(HOOD_MIN_ANGLE_RAD, Math.min(angleRadians, max));
        hood.setGoal(Radians.of(hoodAngleSetpoint));
    }

    private void setFlywheelVelocity(double rps) {
        flywheelVelocitySetpoint = rps;
        flywheel.setVelocityRotationsPerSecond(rps);
    }

    // ── Targeting math (ported verbatim from v1) ─────────────────────────────

    public Supplier<Translation2d> getTurretToTargetVector() {
        return () -> {
            ChassisSpeeds robotSpeeds = swerveSpeeds.get();
            Pose2d robotPose = robotPositionSupplier.get();
            Rotation2d robotRot = robotPose.getRotation();

            Translation2d turretField = getTurretOnField().getTranslation();
            Translation2d fieldVector =
                    machine.getCurrentState().targetTranslation.get().minus(turretField);
            Translation2d turretToTarget = fieldVector.rotateBy(robotRot.unaryMinus());

            // Fixed-point iteration converging the (range <-> time-of-flight <-> lead) loop
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
                robotSpeeds.vyMetersPerSecond + TURRET_OFFSET_TRANSLATION.getX() * robotSpeeds.omegaRadiansPerSecond);
        double radialVelocity = robotVelocity.getX() * unitToTarget.getX() + robotVelocity.getY() * unitToTarget.getY();
        return nominalVelocity - radialVelocity;
    }

    public Pose2d getTurretOnField() {
        Pose2d robotPose = robotPositionSupplier.get();
        Translation2d turretField = robotPose.getTranslation()
                .plus(TURRET_OFFSET_TRANSLATION.rotateBy(robotPose.getRotation()));
        return new Pose2d(turretField,
                robotPose.getRotation().minus(getTurretRotation().unaryMinus()));
    }

    public Rotation2d getTurretRotation() {
        return new Rotation2d(turret.getPosition().in(Radians));
    }

    public InterpolatingDoubleTreeMap getInterpolatingTimeOfFlightMap() {
        return machine.getCurrentState().targetHeight.equals(HIGH)
                ? highDistanceTimeOfFlightMap : lowDistanceTimeOfFlightMap;
    }

    public InterpolatingDoubleTreeMap getInterpolatingVelocityMap() {
        return machine.getCurrentState().targetHeight.equals(HIGH)
                ? highVelocityDistanceMap : lowVelocityDistanceMap;
    }

    public InterpolatingDoubleTreeMap getInterpolatingAngleMap() {
        return machine.getCurrentState().targetHeight.equals(HIGH)
                ? highAngleDistanceMap : lowAngleDistanceMap;
    }

    // ── Public API ───────────────────────────────────────────────────────────

    public Command setStateCommand(ShooterStates state) {
        return machine.requestCommand(state);
    }

    public Trigger isShooterReady() {
        return shooterReady;
    }

    /** Manual overrides (used by Superstructure's manual shooting mode). */
    public Command manualShootingCommand(DoubleSupplier hoodAngleRad, DoubleSupplier flywheelRps) {
        return run(() -> {
            setHoodAngle(hoodAngleRad.getAsDouble());
            setFlywheelVelocity(flywheelRps.getAsDouble());
            turret.setGoal(Radians.of(getTurretToTargetVector().get().getAngle().getRadians()));
        }).withName("Shooter Manual");
    }

    public Command coastTurretCommand() {
        return turret.coastCommand(this);
    }

    public int getBallCount() {
        return ballCounter.getBallCount();
    }

    public void resetBallCount() {
        ballCounter.resetCount();
    }

    @Override
    public void periodic() {
        flywheel.periodic();
        hood.periodic();
        turret.periodic();
        machine.periodic();

        filteredFlywheelVelocity += EMA_ALPHA * (flywheel.getVelocityRotationsPerSecond() - filteredFlywheelVelocity);

        DogLog.log("Shooter/FlywheelVelocityFiltered", filteredFlywheelVelocity);
        DogLog.log("Shooter/FlywheelSetpoint", flywheelVelocitySetpoint);
        DogLog.log("Shooter/HoodSetpointRad", hoodAngleSetpoint);
        DogLog.log("Shooter/Ready", shooterReady.getAsBoolean());
        DogLog.log("Shooter/TurretAligned", isTurretAligned.getAsBoolean());
        DogLog.log("Shooter/TurretOnField", getTurretOnField());
        DogLog.log("Shooter/DistanceToTarget", getTurretToTargetVector().get().getNorm());
        DogLog.log("Shooter/BallCount", ballCounter.getBallCount());
        DogLog.log("Shooter/InTrench", volatileTrenchHoodTrigger.getAsBoolean());
    }
}
