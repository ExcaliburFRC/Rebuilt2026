package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.mechanisms.Mechanism;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import frc.robot.util.Target;
import monologue.Logged;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.excalib.additional_utilities.AllianceUtils.FIELD_LENGTH_METERS;
import static frc.excalib.additional_utilities.AllianceUtils.FIELD_WIDTH_METERS;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.Target.*;
import static monologue.Annotations.Log.*;

public class Shooter extends SubsystemBase implements Logged {

    private final TalonFXMotor hoodMotor, flyWheelMotorTop, flyWheelMotorLow;
    private final MotorGroup shooterMotorGroup;
    private final CANcoder hoodEncoder;
    private final PIDController angleController;

    private final FlyWheel flyWheelMechanism;
    private final Mechanism hoodMechanism;

    private DoubleSupplier hoodAngleSupplier;
    private final SoftLimit hoodSoftLimit;

    private final Supplier<Pose2d> robotPositionSupplier;

    private final DoubleSupplier turretRelativeDistanceFromTarget;

    private DoubleSupplier flywheelVelocitySetpoint;
    private DoubleSupplier hoodAngleSetpoint;

    private final EMAFilter flywheelVelocityFilter;
    private final EMAFilter kvFilter;

    private final InterpolatingDoubleTreeMap angleDistanceMap;
    private final InterpolatingDoubleTreeMap velocityDistanceMap;

    private final Trigger volatileTrenchHoodTrigger;
    private Target shooterTarget = HUB;
    private BooleanSupplier shootingMode = () -> true;

    private Trigger activateLedsTrigger;
    private Trigger flyWheelReadyTrigger;
    private Trigger hoodAdjustedTrigger;
    private BooleanSupplier transportNeededSupplier;


    public Shooter(DoubleSupplier turretRelativeDistanceFromTarget, Supplier<Pose2d> poseSupplier) {
        hoodMotor = new TalonFXMotor(HOOD_MOTOR_ID, SUBSYSTEMS_CANBUS);
        flyWheelMotorLow = new TalonFXMotor(FLYWHEEL_MOTOR_LOW_ID, SUBSYSTEMS_CANBUS);
        flyWheelMotorTop = new TalonFXMotor(FLYWHEEL_MOTOR_TOP_ID, SUBSYSTEMS_CANBUS);
        hoodEncoder = new CANcoder(HOOD_ENCODER_ID, SUBSYSTEMS_CANBUS);

        shooterMotorGroup = new MotorGroup(flyWheelMotorLow, flyWheelMotorTop);
        shooterMotorGroup.setIdleState(IdleState.BRAKE);
        shooterMotorGroup.setMotorPosition(0);
        shooterMotorGroup.setVelocityConversionFactor((double) 40 / 48);
        shooterMotorGroup.setPositionConversionFactor((double) 40 / 48);

        flyWheelMotorLow.setInverted(DirectionState.FORWARD);
        flyWheelMotorTop.setInverted(DirectionState.FORWARD);

        flyWheelMotorTop.setCurrentLimit(120, 80);
        flyWheelMotorLow.setCurrentLimit(120, 80);

        hoodEncoder.setPosition(hoodEncoder.getAbsolutePosition().getValueAsDouble());
        this.turretRelativeDistanceFromTarget = turretRelativeDistanceFromTarget;

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

        transportNeededSupplier = () -> false;

        flywheelVelocityFilter = new EMAFilter(
                flyWheelMechanism::getVelocity,
                0.05,
                PeriodicScheduler.PERIOD.MILLISECONDS_20);

        kvFilter = new EMAFilter(
                this::getKV,
                0.025
                ,
                PeriodicScheduler.PERIOD.MILLISECONDS_20
        );

        PeriodicScheduler.PERIOD.MILLISECONDS_20.add(flywheelVelocityFilter);
        PeriodicScheduler.PERIOD.MILLISECONDS_20.add(kvFilter);

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

//        setDefaultCommand(defaultCommand());
    }


    public void initAngleMap() {
//        angleDistanceMapTable.put(distance[meters], hood angle);
        angleDistanceMap.put(1.947, 0.0);
        angleDistanceMap.put(2.58, 0.08);
        angleDistanceMap.put(3.96, 0.225);
        angleDistanceMap.put(5.07, 0.4);
        angleDistanceMap.put(3.48, 0.18);
        angleDistanceMap.put(3.19, 0.13);
        angleDistanceMap.put(4.0, 0.22);
        angleDistanceMap.put(4.81, 0.32);

    }

    public void initVelocityMap() {
//          velocityDistanceMapTable.put(distance[meters], flywheel velocity);
        velocityDistanceMap.put(1.948, 35.0);
        velocityDistanceMap.put(2.58, 37.3);
        velocityDistanceMap.put(3.96, 41.0);
        velocityDistanceMap.put(5.07, 44.0);
        velocityDistanceMap.put(3.48, 39.5);
        velocityDistanceMap.put(3.19, 39.0);
        velocityDistanceMap.put(4.0, 43.0);
        velocityDistanceMap.put(4.81, 44.0);
    }

    public Command setHoodAngleCommand(DoubleSupplier angleSetpoint) {
        return new RunCommand(() -> hoodMechanism.setVoltage(getControlledOutputForAngle(() -> hoodSoftLimit.limit(angleSetpoint.getAsDouble()))), this);
    }

    public Command setFlyWheelDynamicVelocity(DoubleSupplier vel, SubsystemBase... req) {
        flywheelVelocitySetpoint = vel;
        return flyWheelMechanism.setDynamicVelocityCommand(vel, req);
    }

    public Command setAdjustedFlyWheelVelocity() {
//        return new RunCommand(() -> {
//            double distance = turretRelativeDistanceFromTarget.getAsDouble();
//            double velocity = velocityDistanceMap.get(distance);
//            flywheelVelocitySetpoint = () -> velocity;
//            if (!shootingMode.getAsBoolean()) {
//                flyWheelMechanism.setVoltage(0);
//            } else {
//                setFlyWheelDynamicVelocity(() -> velocity).schedule();
//            }
//        });

        return new ConditionalCommand(
                setFlyWheelDynamicVelocity(
                        () -> velocityDistanceMap.get(
                                turretRelativeDistanceFromTarget.getAsDouble()
                        )
                ),
                new RunCommand(()->flyWheelMechanism.setVoltage(0)),
                shootingMode
        );
    }

    public Command setAdjustedHoodAngle() {
        return new RunCommand(() -> {
            double distance = turretRelativeDistanceFromTarget.getAsDouble();
            hoodAngleSetpoint = () -> hoodSoftLimit.limit(angleDistanceMap.get(distance));
            hoodMechanism.setVoltage(getControlledOutputForAngle(() -> hoodSoftLimit.limit(angleDistanceMap.get(distance))));
        });
    }

    public Command setAdjustedTransportBehavior() {
        return new InstantCommand(() -> transportNeededSupplier = () -> true);
//        return new RunCommand(
//                () -> {
//                    if (shootingMode.getAsBoolean()) {
//                        if (flyWheelReadyTrigger()) {
//                            transportMechanism.setVoltage(-12);
//                        } else {
//                            transportMechanism.setVoltage(0);
//                        }
//                    } else {
//                        transportMechanism.setVoltage(0);
//                    }
//                }
//        );
    }

    public Command defaultCommand() {
        Command c = new ConditionalCommand(idleCommand(), new ParallelCommandGroup(setAdjustedFlyWheelVelocity(), setAdjustedTransportBehavior(), setAdjustedHoodAngle()), () -> shooterTarget.equals(IDLE));
        c.addRequirements(this);
        return c;
    }

    public double getControlledOutputForAngle(DoubleSupplier angleSetpoint) {
        double pid = angleController.calculate(hoodMotor.getMotorPosition(), angleSetpoint.getAsDouble());
        if (pid > 0) {
            return pid + Math.signum(pid) * 0.375; //ks positive
        }
        return pid + Math.signum(pid) * -0.25;
    }


    public Command setTargetCommand(Target targetToSet) {
        return new InstantCommand(() -> shooterTarget = targetToSet, this);
    }

    public Command turnOnShootingCommand() {
        return new InstantCommand(() -> shootingMode = () -> true);
    }

    public Command turnOffShootingCommand() {
        return new InstantCommand(() -> shootingMode = () -> false);
    }

    public Command shootToHubCommand() {
        return new InstantCommand(() -> {
            shooterTarget = HUB;
            shootingMode = () -> true;
        });
    }

    public Command trackHubCommand() {
        return new InstantCommand(() -> CommandScheduler.getInstance().schedule(setTargetCommand(HUB).alongWith(turnOffShootingCommand())));
    }

    public Command shootToDeliveryCommand() {
        return new StartEndCommand(() -> CommandScheduler.getInstance().schedule(setTargetCommand(DELIVERY).andThen(turnOnShootingCommand())), () -> CommandScheduler.getInstance().schedule(setTargetCommand(IDLE).andThen(turnOffShootingCommand())));
    }

    public Command idleCommand() {
        return new RunCommand(() -> {
            flyWheelMechanism.setVoltage(0);
            hoodMechanism.setVoltage(0);
            transportNeededSupplier = () -> false;
        }, this).alongWith(turnOffShootingCommand());
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
    public String getShooterTarget() {
        return shooterTarget.name();
    }

    @NT
    public boolean isShootingModeOn() {
        return shootingMode.getAsBoolean();
    }

    @NT
    public BooleanSupplier isTransportNeeded() {
        return transportNeededSupplier;
    }

    public BooleanSupplier shouldTransport() {
        return () -> isTransportNeeded().getAsBoolean() && flyWheelReadyTrigger.getAsBoolean() && hoodAdjustedTrigger();
    }

    @NT
    public double getLimitedHoodAngle() {
        return hoodSoftLimit.limit(hoodAngleSetpoint.getAsDouble());
    }

    @NT
    public double getKV() {
        double voltage = flyWheelMechanism.logVoltage();
        double velocity = flyWheelMechanism.getVelocity();
        return velocity == 0 ? 0 : voltage / velocity;
    }

    @NT
    public double getMeasurementSetpointRatio() {
        return flywheelVelocitySetpoint.getAsDouble() / flyWheelMechanism.getVelocity();
    }

}