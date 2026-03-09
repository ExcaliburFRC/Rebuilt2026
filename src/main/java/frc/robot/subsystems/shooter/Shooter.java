package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
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

    private final TalonFXMotor hoodMotor, flyWheelMotorTop, flyWheelMotorLow, transportMotor;
    private final MotorGroup shooterMotorGroup;
    private final CANcoder hoodEncoder;
    private final PIDController angleController;

    private final FlyWheel flyWheelMechanism;
    private final Mechanism hoodMechanism;
    private final Mechanism transportMechanism;

    private DoubleSupplier hoodAngleSupplier;
    private final SoftLimit hoodSoftLimit;

    private final Supplier<Pose2d> robotPositionSupplier;

    private final DoubleSupplier turretRelativeDistanceFromTarget;

    private DoubleSupplier flywheelVelocitySetpoint;
    private DoubleSupplier hoodAngleSetpoint;

    private final EMAFilter flywheelVelocityFilter;

    private final InterpolatingDoubleTreeMap angleDistanceMap;
    private final InterpolatingDoubleTreeMap velocityDistanceMap;

    private final Trigger volitileTrenchHoodTrigger;

    private Target shooterTarget = IDLE;
    private BooleanSupplier shootingMode = () -> false;

    private Trigger activateLedsTrigger;


    public Shooter(DoubleSupplier turretRelativeDistanceFromTarget, Supplier<Pose2d> poseSupplier) {
        hoodMotor = new TalonFXMotor(HOOD_MOTOR_ID, SUBSYSTEMS_CANBUS);
        flyWheelMotorLow = new TalonFXMotor(FLYWHEEL_MOTOR_LOW_ID, SUBSYSTEMS_CANBUS);
        flyWheelMotorTop = new TalonFXMotor(FLYWHEEL_MOTOR_TOP_ID, SUBSYSTEMS_CANBUS);
        transportMotor = new TalonFXMotor(TRANSPORT_MOTOR_ID, SUBSYSTEMS_CANBUS);
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

        angleController = new PIDController(HOOD_PID_GAINS.kp, HOOD_PID_GAINS.ki, HOOD_PID_GAINS.kd);
        angleController.setTolerance(0.01);

        hoodMotor.setIdleState(IdleState.BRAKE);
        hoodMotor.setInverted(DirectionState.FORWARD);
        hoodAngleSupplier = () -> (hoodEncoder.getPosition().getValueAsDouble() * POSITION_CONVERSION_FACTOR);

        hoodMotor.setPositionConversionFactor(POSITION_CONVERSION_FACTOR * ((double) -0.208 / 1.497) * 1.0231);
        hoodMotor.setMotorPosition(hoodAngleSupplier.getAsDouble());

        flywheelVelocitySetpoint = () -> 0;
        hoodAngleSetpoint = () -> 0;
        hoodMotor.setIdleState(IdleState.COAST);

        flyWheelMechanism = new FlyWheel(shooterMotorGroup, FLY_WHEEL_MAX_ACCELERATION, FLY_WHEEL_MAX_JERK, FLYWHEEL_GAINS);

        transportMechanism = new Mechanism(transportMotor);

        flywheelVelocityFilter = new EMAFilter(
                flyWheelMechanism::getVelocity,
                0.05,
                PeriodicScheduler.PERIOD.MILLISECONDS_20
        );

        PeriodicScheduler.PERIOD.MILLISECONDS_20.add(flywheelVelocityFilter);

        hoodMechanism = new Mechanism(hoodMotor);

        robotPositionSupplier = poseSupplier;

        angleDistanceMap = new InterpolatingDoubleTreeMap();
        initAngleMap();

        velocityDistanceMap = new InterpolatingDoubleTreeMap();
        initVelocityMap();


        activateLedsTrigger = new Trigger(()-> flyWheelMechanism.getVelocity() > 3);
        activateLedsTrigger.onTrue(
                LEDs.getInstance().setPattern(
                        LEDs.LEDPattern.BLINKING,
                        Color.Colors.TEAM_BLUE.color
                ).andThen(LEDs.getInstance().restoreLEDs())
        );
        volitileTrenchHoodTrigger = new Trigger(
                () -> {
                    Pose2d pose = poseSupplier.get();
                    if (AllianceUtils.isBlueAlliance()) {
                        return (pose.getX() > FRONT_TRENCH_SIDEX_LINE_DIST_METERS &&
                                pose.getY() < TRENCH_SIDEY_LINE_DIST_METERS) &&
                                (pose.getX() < BACK_TRENCH_SIDEX_LINE_DIST_METERS);
                    } else {
                        return (pose.getX() < FIELD_LENGTH_METERS - FRONT_TRENCH_SIDEX_LINE_DIST_METERS &&
                                pose.getY() > FIELD_WIDTH_METERS - TRENCH_SIDEY_LINE_DIST_METERS) &&
                                (pose.getX() > FIELD_LENGTH_METERS - BACK_TRENCH_SIDEX_LINE_DIST_METERS);
                    }
                }
        );

        hoodSoftLimit = new SoftLimit(
                () -> HOOD_MIN_ANGLE_LIMIT,
                () -> {
                    if (volitileTrenchHoodTrigger.getAsBoolean()) {
                        return HOOD_MAX_ANGLE_LIMIT_IN_TRENCH;
                    }
                    return HOOD_MAX_ANGLE_LIMIT;
                }
        );

        setDefaultCommand(defaultCommand());
    }


    public void initAngleMap() {
//        angleDistanceMapTable.put(distance[meters], hood angle);
        angleDistanceMap.put(4.95, 0.6);
        angleDistanceMap.put(3.2, 0.29);
        angleDistanceMap.put(1.57, 0.0);

    }

    public void initVelocityMap() {
//          velocityDistanceMapTable.put(distance[meters], flywheel velocity);
        velocityDistanceMap.put(4.95, 42.0);
        velocityDistanceMap.put(3.2, 35.86);
        velocityDistanceMap.put(1.57, 30.0);

    }

    public Command setHoodAngleCommand(DoubleSupplier angleSetpoint) {
        return new RunCommand(
                () -> hoodMechanism.setVoltage(
                        getPIDForAngle(
                                () -> hoodSoftLimit.limit(
                                        angleSetpoint.getAsDouble()))), this);
    }

    public Command setFlyWheelDynamicVelocity(DoubleSupplier vel, SubsystemBase... req) {
        return flyWheelMechanism.setDynamicVelocityCommand(vel, req);
    }

    public Command setAdjustedFlyWheelVelocity() {
        return new RunCommand(
                () -> {
                    double distance = turretRelativeDistanceFromTarget.getAsDouble();
                    double velocity = velocityDistanceMap.get(distance);
                    flywheelVelocitySetpoint = () -> velocity;
                    flyWheelMechanism.setDynamicVelocity(velocity);
                }
        );
    }

    public Command setAdjustedHoodAngle() {
        return new RunCommand(
                () -> {
                    double distance = turretRelativeDistanceFromTarget.getAsDouble();
                    hoodAngleSetpoint = () -> hoodSoftLimit.limit(angleDistanceMap.get(distance));
                    hoodMechanism.setVoltage(
                            getPIDForAngle(
                                    () -> hoodSoftLimit.limit(angleDistanceMap.get(distance))
                            )
                    );
                }
        );
    }

    public Command setAdjustedTransportBehavior() {
//        return new ConditionalCommand(
//                transportMechanism.manualCommand(() -> TRANSPORT_VOLTAGE),
//                transportMechanism.manualCommand(() -> 0),
//                shootingMode
//        );
        return transportMechanism.manualCommand(() -> TRANSPORT_VOLTAGE);
    }

    public Command defaultCommand() {
        Command c = new ConditionalCommand(
                Commands.none(),
                new ParallelCommandGroup(
                        setAdjustedFlyWheelVelocity(),
                        setAdjustedHoodAngle(),
                        setAdjustedTransportBehavior()),
                () -> shooterTarget.equals(IDLE)
        );
        c.addRequirements(this);
        return c;
    }

    public double getPIDForAngle(DoubleSupplier angleSetpoint) {
        return angleController.calculate(hoodMotor.getMotorPosition(), angleSetpoint.getAsDouble());
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
        return new StartEndCommand(
                () -> CommandScheduler.getInstance().schedule(setTargetCommand(HUB).andThen(turnOnShootingCommand())),
                () -> CommandScheduler.getInstance().schedule(setTargetCommand(IDLE).andThen(turnOffShootingCommand())
                ));
    }

    public Command trackHubCommand() {
        return new StartEndCommand(
                () -> CommandScheduler.getInstance().schedule(setTargetCommand(HUB)),
                () -> CommandScheduler.getInstance().schedule(setTargetCommand(IDLE))
        );
    }

    public Command shootToDeliveryCommand() {
        return new StartEndCommand(
                () -> CommandScheduler.getInstance().schedule(setTargetCommand(DELIVERY).andThen(turnOnShootingCommand())),
                () -> CommandScheduler.getInstance().schedule(setTargetCommand(IDLE).andThen(turnOffShootingCommand()))
        );
    }

    public Command manualTransport() {
        return transportMechanism.manualCommand(() -> TRANSPORT_VOLTAGE);
    }

    @NT
    public double getFlyWheelVelocitySetpoint() {
        return flywheelVelocitySetpoint.getAsDouble() < 0.01 ? 0 : flywheelVelocitySetpoint.getAsDouble();
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


}