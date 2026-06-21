package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
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
import frc.robot.util.BallCounter;
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

    // ===== Physics-based shot solver (replaces the old lookup tables) =====
    private final BallisticSolver ballisticSolver = new BallisticSolver();
    /** Latest solved shot, refreshed every periodic() loop. */
    private BallisticSolver.ShotSolution shotSolution = BallisticSolver.INVALID;
    /** Latest lead-compensated turret-relative aim vector (horizontal, robot frame). */
    private Translation2d cachedAimVector = new Translation2d();
    /** Latest hood encoder-value setpoint and flywheel setpoint derived from the solve. */
    private double solvedHoodValue = 0.0;
    private double solvedFlywheelSetpoint = 0.0;

    private final Trigger volatileTrenchHoodTrigger;

    private final Trigger activateLedsTrigger;
    private final Trigger flyWheelReadyTrigger;
    private final Trigger hoodAdjustedTrigger;
    public final Trigger isTurretAligned;

    public final Trigger shooterReady;

    private Supplier<ChassisSpeeds> swerveSpeeds;

    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> swerveSpeeds) {
        hoodMotor = new TalonFXMotor(HOOD_MOTOR_ID, SUBSYSTEMS_CANBUS);
        hoodMotor.setCurrentLimit(40, 30);
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


    /**
     * Returns the lead-compensated, turret-relative aim vector (horizontal, robot frame).
     * The heavy ballistic solve runs once per loop in {@link #periodic()}; this just hands
     * back the cached result so the many call sites stay cheap.
     */
    public Supplier<Translation2d> getTurretToTargetVector() {
        return () -> cachedAimVector;
    }

    @Override
    public void periodic() {
        if (DISABLE_SUBSYSTEMS) return;
        solveCurrentShot();
    }

    /**
     * The full physics shot solve for the current robot pose / velocity / target.
     *
     * <ol>
     *   <li>Compute the un-led turret-relative vector to the target.</li>
     *   <li>Fixed-point loop: solve the ballistic trajectory for the current aim to get a
     *       time-of-flight, then shift the aim point by the robot's velocity * time-of-flight
     *       (shoot-while-moving / shot lead).</li>
     *   <li>Cache the aim vector, plus the hood value and flywheel setpoint derived from the
     *       solved launch angle and exit speed.</li>
     * </ol>
     */
    private void solveCurrentShot() {
        Translation2d turretToTarget = computeTurretToTargetNoLead();

        ChassisSpeeds robotSpeeds = swerveSpeeds.get();
        Translation2d robotVelocity = new Translation2d(
                robotSpeeds.vxMetersPerSecond - TURRET_OFFSET_TRANSLATION.getY() * robotSpeeds.omegaRadiansPerSecond,
                robotSpeeds.vyMetersPerSecond + TURRET_OFFSET_TRANSLATION.getX() * robotSpeeds.omegaRadiansPerSecond
        );

        double heightDelta = heightDeltaForGoal();
        double descentAngle = descentAngleForGoal();

        Translation2d aim = turretToTarget;
        BallisticSolver.ShotSolution solution = BallisticSolver.INVALID;
        for (int i = 0; i < BallisticConstants.LEAD_ITERATIONS; i++) {
            solution = ballisticSolver.solve(aim.getNorm(), heightDelta, descentAngle);
            double tof = solution.valid() ? solution.timeOfFlight() : 0.0;
            aim = turretToTarget.minus(robotVelocity.times(tof));
        }

        cachedAimVector = aim;
        shotSolution = solution;

        if (solution.valid()) {
            solvedHoodValue = BallisticConstants.hoodValueForLaunchAngle(solution.launchAngleRad());
            solvedFlywheelSetpoint = BallisticConstants.flywheelSetpointForBallSpeed(solution.ballSpeedMps());
        }
        // If the solve was invalid (target unreachable), keep the previous setpoints so the
        // mechanisms hold their last good aim rather than snapping to zero.
    }

    /** Turret-relative (robot-frame) vector to the current target, with no shot-lead applied. */
    private Translation2d computeTurretToTargetNoLead() {
        Pose2d robotPose = robotPositionSupplier.get();
        Rotation2d robotRot = robotPose.getRotation();
        Translation2d turretField = getTurretOnField().getTranslation();
        Translation2d fieldVector = currentState.targetTranslation.get().minus(turretField);
        return fieldVector.rotateBy(robotRot.unaryMinus());
    }

    /** Target height minus ball exit height for the current goal (meters). */
    private double heightDeltaForGoal() {
        double goalHeight = currentState.targetHeight.equals(HIGH)
                ? BallisticConstants.HIGH_GOAL_HEIGHT_M
                : BallisticConstants.LOW_GOAL_HEIGHT_M;
        return goalHeight - BallisticConstants.BALL_EXIT_HEIGHT_M;
    }

    /** Desired descent angle into the current goal (radians, downward from horizontal). */
    private double descentAngleForGoal() {
        return currentState.targetHeight.equals(HIGH)
                ? BallisticConstants.HIGH_GOAL_DESCENT_ANGLE_RAD
                : BallisticConstants.LOW_GOAL_DESCENT_ANGLE_RAD;
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
                    // The shot lead (robot velocity) is already baked into the solved exit speed
                    // because solveCurrentShot() aims at the lead-shifted virtual target.
                    double velocity = currentState.isShooting ? solvedFlywheelSetpoint : 0;
                    flywheelVelocitySetpoint = () -> velocity;
                    return velocity;
                }
        );
    }

    public double getWantedVelocity(){
        return currentState.isShooting ? solvedFlywheelSetpoint : 0;
    }

    public Command setAdjustedHoodAngleCommand() {
        return new RunCommand(() -> {
            hoodAngleSetpoint = () -> hoodSoftLimit.limit(solvedHoodValue);
            hoodMechanism.setVoltage(getControlledOutputForAngle(() -> hoodSoftLimit.limit(solvedHoodValue)));
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

    @NT
    public boolean isTurretAligned() {
        return isTurretAligned.getAsBoolean();
    }

    // ===== Ballistic solver telemetry (handy while tuning BallisticConstants) =====

    @NT
    public boolean shotSolutionValid() {
        return shotSolution.valid();
    }

    @NT
    public double solvedLaunchAngleDeg() {
        return Math.toDegrees(shotSolution.launchAngleRad());
    }

    @NT
    public double solvedBallSpeedMps() {
        return shotSolution.ballSpeedMps();
    }

    @NT
    public double solvedTimeOfFlight() {
        return shotSolution.timeOfFlight();
    }

    @NT
    public double solvedHoodValue() {
        return solvedHoodValue;
    }

    @NT
    public double solvedFlywheelSetpoint() {
        return solvedFlywheelSetpoint;
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