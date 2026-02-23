package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.mechanisms.Mechanism;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import frc.robot.util.Target;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.excalib.additional_utilities.AllianceUtils.FIELD_LENGTH_METERS;
import static frc.excalib.additional_utilities.AllianceUtils.FIELD_WIDTH_METERS;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class Shooter extends SubsystemBase implements Logged {

    public final TalonFXMotor hoodMotor, flyWheelMotor;
    public final PIDController angleController;

    public final FlyWheel flyWheelMechanism;
    public final Mechanism hoodMechanism;
    public final Mechanism transportMechanism;

    public final TalonFXMotor transportMotor;
    public DoubleSupplier hoodAngleSupplier;
    public final SoftLimit hoodSoftLimit;

    public final Trigger flyWheelToleranceTrigger;
    public double flywheelVelocitySetpoint;

    public final Supplier<Pose2d> robotPositionSupplier;

    public final DoubleSupplier turretRelativeDistanceFromTarget;
    public final CANcoder hoodEncoder;

    public final InterpolatingDoubleTreeMap angleDistanceMap;
    public final InterpolatingDoubleTreeMap velocityDistanceMap;

    public final Trigger volitileTrenchHoodTrigger;

    public Target shooterTarget = Target.IDLE;
    public BooleanSupplier shootingMode = () -> false;

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

        flyWheelMechanism = new FlyWheel(flyWheelMotor, FLY_WHEEL_MAX_ACCELERATION, FLY_WHEEL_MAX_JERK, FLYWHEEL_GAINS);
        flywheelVelocitySetpoint = DEFAULT_FLYWHEEL_VELOCITY;

        flyWheelToleranceTrigger = new Trigger(
                () -> (flyWheelMechanism.getVelocity() < (flywheelVelocitySetpoint + FLY_WHEEL_TOLERANCE) &&
                        flyWheelMechanism.getVelocity() > (flywheelVelocitySetpoint - FLY_WHEEL_TOLERANCE))
        );

        transportMechanism = new Mechanism(transportMotor);

        hoodMechanism = new Mechanism(hoodMotor);

        robotPositionSupplier = poseSupplier;

        angleDistanceMap = new InterpolatingDoubleTreeMap();
        initAngleMap();

        velocityDistanceMap = new InterpolatingDoubleTreeMap();
        initVelocityMap();

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
                    flyWheelMechanism.setDynamicVelocity(velocity); // DIRECT motor control
                }
        );
    }

    public Command setAdjustedHoodAngle() {
        return new RunCommand(
                () -> {
                    double distance = turretRelativeDistanceFromTarget.getAsDouble();
                    setHoodAngleCommand(() -> angleDistanceMap.get(distance)); // DIRECT motor control
                }
        );
    }


    public Command defaultCommand() {
        Command c = new ConditionalCommand(
                Commands.none(),
                new ParallelCommandGroup(
                        setAdjustedFlyWheelVelocity(),
                        setAdjustedHoodAngle(),
                        transportMechanism.manualCommand(() -> TRANSPORT_VOLTAGE).onlyIf(shootingMode).onlyIf(flyWheelToleranceTrigger)),
                () -> shooterTarget.equals(Target.IDLE)
        );
        c.addRequirements(this);
        return c;
    }

    public double getPIDForAngle(DoubleSupplier angleSetpoint) {
        return angleController.calculate(hoodMotor.getMotorPosition(), angleSetpoint.getAsDouble());
    }

    @Log.NT
    public double getEncoderAngle() {
        return hoodEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public Command setTargetCommand(Target targetToSet) {
        return new InstantCommand(() -> shooterTarget = targetToSet);
    }

    public Command shootToHubCommand() {
        return new StartEndCommand(
                () -> new InstantCommand(() -> shooterTarget = Target.HUB).andThen(new InstantCommand(() -> shootingMode = () -> true)),
                () -> new InstantCommand(() -> shooterTarget = Target.IDLE).andThen(new InstantCommand(() -> shootingMode = () -> false))
        );
    }

    public Command shootToDeliveryCommand() {
        return new StartEndCommand(
                () -> new InstantCommand(() -> shooterTarget = Target.DELIVERY).andThen(new InstantCommand(() -> shootingMode = () -> true)),
                () -> new InstantCommand(() -> shooterTarget = Target.IDLE).andThen(new InstantCommand(() -> shootingMode = () -> false))
        );
    }
}