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

    private final TalonFXMotor hoodMotor, flyWheelMotor;
    private final CANcoder hoodEncoder;
    private final PIDController angleController;

    private final FlyWheel flyWheelMechanism;
    private final Mechanism hoodMechanism;
    private final Mechanism transportMechanism;

    private final TalonFXMotor transportMotor;
    private DoubleSupplier hoodAngleSupplier;
    private final SoftLimit hoodSoftLimit;

    private final Supplier<Pose2d> robotPositionSupplier;

    private final DoubleSupplier turretRelativeDistanceFromTarget;
    private DoubleSupplier flywheelVelocitySetpoint;

    private final InterpolatingDoubleTreeMap angleDistanceMap;
    private final InterpolatingDoubleTreeMap velocityDistanceMap;

    private final Trigger volitileTrenchHoodTrigger;

    private Target shooterTarget = IDLE;
    private BooleanSupplier shootingMode = () -> false;

    private final Trigger isShootingModeOnTrigger = new Trigger(() -> shootingMode.getAsBoolean());

    public Shooter(DoubleSupplier turretRelativeDistanceFromTarget, Supplier<Pose2d> poseSupplier) {
        hoodMotor = new TalonFXMotor(HOOD_MOTOR_ID, SUBSYSTEMS_CANBUS);
        flyWheelMotor = new TalonFXMotor(FLYWHEEL_MOTOR_ID, SUBSYSTEMS_CANBUS);
        transportMotor = new TalonFXMotor(TRANSPORT_MOTOR_ID, SUBSYSTEMS_CANBUS);
        hoodEncoder = new CANcoder(HOOD_ENCODER_ID, SUBSYSTEMS_CANBUS);

        hoodEncoder.setPosition(hoodEncoder.getAbsolutePosition().getValueAsDouble());
        this.turretRelativeDistanceFromTarget = turretRelativeDistanceFromTarget;

        angleController = new PIDController(HOOD_PID_GAINS.kp, HOOD_PID_GAINS.ki, HOOD_PID_GAINS.kd);
        angleController.setTolerance(0.01);

        hoodMotor.setIdleState(IdleState.COAST);
        hoodMotor.setInverted(DirectionState.REVERSE);
        hoodAngleSupplier = () -> (hoodEncoder.getPosition().getValueAsDouble() * POSITION_CONVERSION_FACTOR) + 0.69;

        hoodMotor.setPositionConversionFactor(0.048869);
        hoodMotor.setMotorPosition(1.348);
        flywheelVelocitySetpoint = () -> 0;

        flyWheelMechanism = new FlyWheel(flyWheelMotor, FLY_WHEEL_MAX_ACCELERATION, FLY_WHEEL_MAX_JERK, FLYWHEEL_GAINS);

        transportMechanism = new Mechanism(transportMotor);

        hoodMechanism = new Mechanism(hoodMotor);

        robotPositionSupplier = poseSupplier;

        angleDistanceMap = new InterpolatingDoubleTreeMap();
        initAngleMap();

        velocityDistanceMap = new InterpolatingDoubleTreeMap();
        initVelocityMap();


        isShootingModeOnTrigger.onTrue(
                LEDs.getInstance().setPattern(
                        LEDs.LEDPattern.TRAIN_CIRCLE,
                        Color.Colors.PURPLE.color,
                        Color.Colors.OFF.color
                ).andThen(new WaitCommand(0.5)).andThen(
                       LEDs.getInstance().setPattern(
                        LEDs.LEDPattern.SOLID,
                        Color.Colors.GREEN.color
                )));

        isShootingModeOnTrigger.onFalse(LEDs.getInstance().setPattern(
                        LEDs.LEDPattern.EXPAND,
                        Color.Colors.TEAM_BLUE.color,
                        Color.Colors.TEAM_GOLD.color));

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
        angleDistanceMap.put(2.16, 1.348);
        angleDistanceMap.put(3.29, 1.248);
        angleDistanceMap.put(4.39, 1.177);
        angleDistanceMap.put(4.67, 1.429);
        angleDistanceMap.put(2.49, 1.347);
        angleDistanceMap.put(2.98, 1.346);
        angleDistanceMap.put(4.08, 1.344);
    }

    public void initVelocityMap() {
//          velocityDistanceMapTable.put(distance[meters], flywheel velocity);
        velocityDistanceMap.put(2.16, 21.0);
        velocityDistanceMap.put(3.29, 24.0);
        velocityDistanceMap.put(4.39, 26.0);
        velocityDistanceMap.put(4.67, 26.0);
        velocityDistanceMap.put(2.49, 22.0);
        velocityDistanceMap.put(2.98, 23.0);
        velocityDistanceMap.put(4.08, 25.0);
    }

    public Command setHoodAngleCommand(DoubleSupplier angleSetpoint) {
        return new RunCommand(
                () -> hoodMechanism.setVoltage(
                        getPIDForAngle(
                                () -> hoodSoftLimit.limit(
                                        angleSetpoint.getAsDouble()))), this);
    }

    public Command setAdjustedFlyWheelVelocity() {
        return new RunCommand(
                () -> {
                    double distance = turretRelativeDistanceFromTarget.getAsDouble();
                    double velocity = velocityDistanceMap.get(distance);
                    flywheelVelocitySetpoint = () -> velocity;
                    flyWheelMechanism.setDynamicVelocity(velocity); // DIRECT motor control
                }
        );
    }

    public Command setAdjustedHoodAngle() {
        return new RunCommand(
                () -> {
                    double distance = turretRelativeDistanceFromTarget.getAsDouble();
                    hoodMechanism.setVoltage(
                            getPIDForAngle(
                                    () -> hoodSoftLimit.limit(angleDistanceMap.get(distance))
                            )
                    );
                }
        );
    }

    public Command setAdjustedTransportBehavior() {
        return new ConditionalCommand(
                transportMechanism.manualCommand(() -> TRANSPORT_VOLTAGE),
                transportMechanism.manualCommand(() -> 0),
                shootingMode
        );
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

    @NT
    public double getFlyWheelVelocitySetpoint() {
        return flywheelVelocitySetpoint.getAsDouble();
    }

    @NT
    public double getFlyWheelVelocity() {
        return flyWheelMechanism.getVelocity();
    }

    @NT
    public double getEncoderAngle() {
        return hoodEncoder.getAbsolutePosition().getValueAsDouble();
    }

}