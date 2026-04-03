package frc.robot.subsystems.shooter;

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
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.mechanisms.Mechanism;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import frc.excalib.mechanisms.turret.Turret;
import monologue.Logged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.excalib.additional_utilities.AllianceUtils.FIELD_LENGTH_METERS;
import static frc.excalib.additional_utilities.AllianceUtils.FIELD_WIDTH_METERS;
import static frc.robot.Constants.DISABLE_SUBSYSTEMS;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.Target.*;

public class Shooter extends SubsystemBase implements Logged {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

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

    private double flywheelVelocitySetpoint = 0.0;
    private double hoodAngleSetpoint = 0.0;

    private final EMAFilter flywheelVelocityFilter;

    private static final InterpolatingDoubleTreeMap ANGLE_DISTANCE_MAP = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap VELOCITY_DISTANCE_MAP = new InterpolatingDoubleTreeMap();

    static {
        ANGLE_DISTANCE_MAP.put(1.947, 0.0);
        ANGLE_DISTANCE_MAP.put(2.58, 0.08);
        ANGLE_DISTANCE_MAP.put(3.19, 0.13);
        ANGLE_DISTANCE_MAP.put(3.48, 0.18);
        ANGLE_DISTANCE_MAP.put(3.96, 0.225);
        ANGLE_DISTANCE_MAP.put(4.0, 0.22);
        ANGLE_DISTANCE_MAP.put(4.81, 0.32);
        ANGLE_DISTANCE_MAP.put(5.07, 0.4);

        VELOCITY_DISTANCE_MAP.put(1.948, 35.0);
        VELOCITY_DISTANCE_MAP.put(2.58, 37.3);
        VELOCITY_DISTANCE_MAP.put(3.19, 39.0);
        VELOCITY_DISTANCE_MAP.put(3.48, 39.5);
        VELOCITY_DISTANCE_MAP.put(3.96, 41.0);
        VELOCITY_DISTANCE_MAP.put(4.0, 43.0);
        VELOCITY_DISTANCE_MAP.put(4.81, 44.0);
        VELOCITY_DISTANCE_MAP.put(5.07, 44.0);
    }

    private final Trigger volatileTrenchHoodTrigger;

    private Target shooterTarget = HUB;
    private boolean shootingMode = false;

    public final Trigger shooterReady;

    private Supplier<ChassisSpeeds> swerveSpeeds;

    public Shooter(ShooterIO io, DoubleSupplier turretRelativeDistanceFromTarget, Supplier<Pose2d> poseSupplier) {
        this.io = io;
        this.turretRelativeDistanceFromTarget = turretRelativeDistanceFromTarget;

        Motor hoodMotor = io.getHoodMotor();
        Motor flywheelMotorLow = io.getFlywheelMotorLow();
        Motor flywheelMotorTop = io.getFlywheelMotorTop();
        Motor transportMotor = io.getTransportMotor();

        MotorGroup shooterMotorGroup = new MotorGroup(flywheelMotorLow, flywheelMotorTop);

        // hoodAngleSupplier reads from logged inputs for proper replay
        hoodAngleSupplier = () -> inputs.hoodEncoderPositionRotations * POSITION_CONVERSION_FACTOR;

        turretToHubVector = getTurretToTargetVector();

        angleController = new PIDController(HOOD_GAINS.kp, HOOD_GAINS.ki, HOOD_GAINS.kd);
        angleController.setTolerance(0.01);

        flywheelVelocitySetpoint = 0;
        hoodAngleSetpoint = 0;

        flyWheelMechanism = new FlyWheel(shooterMotorGroup, FLYWHEEL_MAX_ACCELERATION, FLYWHEEL_MAX_JERK, FLYWHEEL_GAINS);
        transportMechanism = new Mechanism(transportMotor);
        hoodMechanism = new Mechanism(hoodMotor);

        flywheelVelocityFilter = new EMAFilter(
                () -> inputs.flywheelVelocityRotPerSec,
                0.05,
                PeriodicScheduler.PERIOD.MILLISECONDS_20);


        PeriodicScheduler.PERIOD.MILLISECONDS_20.add(flywheelVelocityFilter);

        robotPositionSupplier = poseSupplier;

        flyWheelReadyTrigger = new Trigger(
                () -> Math.abs(flywheelVelocityFilter.getValue() - flywheelVelocitySetpoint) < FLYWHEEL_TOLERANCE);

        hoodAdjustedTrigger = new Trigger(
                () -> Math.abs(hoodAngleSupplier.getAsDouble() - hoodAngleSetpoint) < HOOD_TOLERANCE);

        activateLedsTrigger = new Trigger(() -> flyWheelMechanism.getVelocity() > 3);
        activateLedsTrigger.onTrue(LEDs.getInstance().setPattern(LEDs.LEDPattern.BLINKING, Color.Colors.ORANGE.color).andThen(LEDs.getInstance().restoreLEDs()));

        volatileTrenchHoodTrigger = new Trigger(
                () -> {
                    Pose2d pose = poseSupplier.get();
                    if (AllianceUtils.isBlueAlliance()) {
                        return isWithinBounds(pose.getX(), pose.getY(), 
                                FRONT_TRENCH_SIDEX_LINE_DIST_METERS, BACK_TRENCH_SIDEX_LINE_DIST_METERS, 
                                -Double.MAX_VALUE, TRENCH_SIDEY_LINE_DIST_METERS);
                    } else {
                        return isWithinBounds(pose.getX(), pose.getY(), 
                                FIELD_LENGTH_METERS - BACK_TRENCH_SIDEX_LINE_DIST_METERS, 
                                FIELD_LENGTH_METERS - FRONT_TRENCH_SIDEX_LINE_DIST_METERS, 
                                FIELD_WIDTH_METERS - TRENCH_SIDEY_LINE_DIST_METERS, Double.MAX_VALUE);
                    }
                }
        );

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

        setDefaultCommand(defaultCommand().unless(() -> DISABLE_SUBSYSTEMS));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if (shooterTarget == HUB) {
            double distance = turretRelativeDistanceFromTarget.getAsDouble();
            flywheelVelocitySetpoint = VELOCITY_DISTANCE_MAP.get(distance);
            hoodAngleSetpoint = hoodSoftLimit.limit(ANGLE_DISTANCE_MAP.get(distance));
        } else if (shooterTarget == DELIVERY) {
            // Add delivery specific setpoints if needed
        }

        if (shootingMode && shooterTarget != IDLE) {
            flyWheelMechanism.setDynamicVelocity(flywheelVelocitySetpoint);
            hoodMechanism.setVoltage(getPIDForAngle(() -> hoodAngleSetpoint));
            transportMechanism.setVoltage(TRANSPORT_VOLTAGE);
        } else {
            flyWheelMechanism.setVoltage(0);
            hoodMechanism.setVoltage(0);
            transportMechanism.setVoltage(0);
        }
    }

    public void setTarget(Target target) {
        this.shooterTarget = target;
    }

    public void setShootingMode(boolean shooting) {
        this.shootingMode = shooting;
    }

    public void setManualSetpoints(double velocity, double angle) {
        this.flywheelVelocitySetpoint = velocity;
        this.hoodAngleSetpoint = angle;
    }

    public void setIdle() {
        this.shooterTarget = IDLE;
        this.shootingMode = false;
    }


    private static boolean isWithinBounds(double x, double y, double minX, double maxX, double minY, double maxY) {
        return x > minX && x < maxX && y > minY && y < maxY;
    }

    public Command setHoodAngleCommand(DoubleSupplier angleSetpoint) {
        return new RunCommand(() -> hoodMechanism.setVoltage(getControlledOutputForAngle(() -> hoodSoftLimit.limit(angleSetpoint.getAsDouble()))));
    }


    public Command setAdjustedFlyWheelVelocity() {
        return Commands.run(
                () -> {
                    double distance = turretRelativeDistanceFromTarget.getAsDouble();
                    double velocity = VELOCITY_DISTANCE_MAP.get(distance);
                    flywheelVelocitySetpoint = velocity;
                    flyWheelMechanism.setDynamicVelocity(velocity);
                }
        );
    }

    public Command setAdjustedHoodAngle() {
        return Commands.run(
                () -> {
                    double distance = turretRelativeDistanceFromTarget.getAsDouble();
                    hoodAngleSetpoint = hoodSoftLimit.limit(ANGLE_DISTANCE_MAP.get(distance));
                    hoodMechanism.setVoltage(
                            getPIDForAngle(
                                    () -> hoodAngleSetpoint
                            )
                    );
                }
        );
    }

    public Command setAdjustedTransportBehavior() {
//        return Commands.either(
//                transportMechanism.manualCommand(() -> TRANSPORT_VOLTAGE),
//                transportMechanism.manualCommand(() -> 0),
//                () -> shootingMode
//        );
        return transportMechanism.manualCommand(() -> TRANSPORT_VOLTAGE);
    }

    public Command defaultCommand() {
        Command c = Commands.either(
                idleCommand(),
                Commands.parallel(
                        setAdjustedFlyWheelVelocity(),
                        setAdjustedHoodAngle(),
                        setAdjustedTransportBehavior()
                ),
                () -> shooterTarget.equals(IDLE)
        );
        c.addRequirements(this);
        return c;
    }

    public double getPIDForAngle(DoubleSupplier angleSetpoint) {
        return angleController.calculate(inputs.hoodMotorPositionConverted, angleSetpoint.getAsDouble());
    }


    public Command setTargetCommand(Target targetToSet) {
        return Commands.runOnce(() -> shooterTarget = targetToSet, this);
    }

    public Command turnOnShootingCommand() {
        return Commands.runOnce(() -> shootingMode = true);
    }

    public Command turnOffShootingCommand() {
        return Commands.runOnce(() -> shootingMode = false);
    }

    public Command shootToHubCommand() {
        return Commands.startEnd(
                () -> {
                    shooterTarget = HUB;
                    shootingMode = true;
                },
                () -> {
                    shooterTarget = IDLE;
                    shootingMode = false;
                }
        );
    }

    public Command trackHubCommand() {
        return Commands.startEnd(
                () -> CommandScheduler.getInstance().schedule(setTargetCommand(HUB)),
                () -> CommandScheduler.getInstance().schedule(setTargetCommand(IDLE))
        );
    }

    public Command shootToDeliveryCommand() {
        return Commands.startEnd(
                () -> CommandScheduler.getInstance().schedule(setTargetCommand(DELIVERY).andThen(turnOnShootingCommand())),
                () -> CommandScheduler.getInstance().schedule(setTargetCommand(IDLE).andThen(turnOffShootingCommand()))
        );
    }

    public Command idleCommand() {
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

    @AutoLogOutput(key = "Shooter/FlyWheelVelocitySetpoint")
    public double getFlyWheelVelocitySetpoint() {
        return flywheelVelocitySetpoint < 0.01 ? 0 : flywheelVelocitySetpoint;
    }

    @AutoLogOutput(key = "Shooter/FlyWheelVelocity")
    public double getFlyWheelVelocity() {
        return flywheelVelocityFilter.getValue();
    }

    @AutoLogOutput(key = "Shooter/HoodAngleSetpoint")
    public double getHoodAngleSetpoint() {
        return hoodAngleSetpoint;
    }

    @AutoLogOutput(key = "Shooter/HoodAngle")
    public double getHoodAngleSupplier() {
        return hoodAngleSupplier.getAsDouble();
    }

    @AutoLogOutput(key = "Shooter/FlyWheelReady")
    public boolean flyWheelReadyTrigger() {
        return flyWheelReadyTrigger.getAsBoolean();
    }

    @AutoLogOutput(key = "Shooter/HoodAdjusted")
    public boolean hoodAdjustedTrigger() {
        return hoodAdjustedTrigger.getAsBoolean();
    }

}
