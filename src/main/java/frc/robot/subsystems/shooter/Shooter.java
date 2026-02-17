package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.mechanisms.Mechanism;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

    public final Trigger flyWheelInToleranceTrigger;
    public double flywheelVelocitySetpoint;

    public final Supplier<Pose2d> robotPositionSupplier;

    public final DoubleSupplier turretRelativeDistanceFromHub;
    public final CANcoder hoodEncoder;

    public final InterpolatingDoubleTreeMap angleDistanceMap;
    public final InterpolatingDoubleTreeMap velocityDistanceMap;

    public Shooter(DoubleSupplier turretRelativeDistanceFromHub, Supplier<Pose2d> poseSupplier) {
        hoodMotor = new TalonFXMotor(HOOD_MOTOR_ID, new CANBus("Subsystems"));
        flyWheelMotor = new TalonFXMotor(FLYWHEEL_MOTOR_ID, new CANBus("Subsystems"));
        transportMotor = new TalonFXMotor(TRANSPORT_MOTOR_ID, new CANBus("Subsystems"));
        hoodEncoder = new CANcoder(HOOD_ENCODER_ID, new CANBus("Subsystems"));

        hoodEncoder.setPosition(hoodEncoder.getAbsolutePosition().getValueAsDouble());
        this.turretRelativeDistanceFromHub = turretRelativeDistanceFromHub;

        angleController = new PIDController(HOOD_PID_GAINS.kp, HOOD_PID_GAINS.ki, HOOD_PID_GAINS.kd);
        angleController.setTolerance(0.01);

        hoodMotor.setIdleState(IdleState.COAST);
        hoodMotor.setInverted(DirectionState.REVERSE);
        hoodAngleSupplier = () -> (hoodEncoder.getPosition().getValueAsDouble() * POSITION_CONVERSION_FACTOR) + 0.69;

        hoodMotor.setPositionConversionFactor(0.048869);
//        hoodMotor.setMotorPosition(hoodAngleSupplier.getAsDouble());
        hoodMotor.setMotorPosition(1.348);

        flyWheelMechanism = new FlyWheel(flyWheelMotor, FLY_WHEEL_MAX_ACCELERATION, FLY_WHEEL_MAX_JERK, FLYWHEEL_GAINS);
        flywheelVelocitySetpoint = DEFAULT_FLYWHEEL_VELOCITY;

        flyWheelInToleranceTrigger = new Trigger(
                () -> (flyWheelMechanism.getVelocity() < (flywheelVelocitySetpoint + FLY_WHEEL_TOLERANCE) &&
                        flyWheelMechanism.getVelocity() > (flywheelVelocitySetpoint - FLY_WHEEL_TOLERANCE))
        );

        transportMechanism = new Mechanism(transportMotor);
        hoodMechanism = new Mechanism(hoodMotor);


        robotPositionSupplier = poseSupplier;

        angleDistanceMap = new InterpolatingDoubleTreeMap();
        initAngleMap(angleDistanceMap);

        velocityDistanceMap = new InterpolatingDoubleTreeMap();
        initVelocityMap(velocityDistanceMap);

        hoodSoftLimit = new SoftLimit(() -> HOOD_MIN_ANGLE_LIMIT, () -> HOOD_MAX_ANGLE_LIMIT);

    }


    public void initAngleMap(InterpolatingTreeMap<Double, Double> angleDistanceMapTable) {
//        angleDistanceMapTable.put(distance[meters], hood angle);
        angleDistanceMapTable.put(2.16, 1.348);
        angleDistanceMapTable.put(3.29, 1.248);
        angleDistanceMapTable.put(4.39, 1.177);
        angleDistanceMapTable.put(4.67, 1.429);
        angleDistanceMapTable.put(2.49, 1.347);
        angleDistanceMapTable.put(2.98, 1.346);
        angleDistanceMapTable.put(4.08,  1.344);
    }

    public void initVelocityMap(InterpolatingTreeMap<Double, Double> velocityDistanceMapTable) {
//          velocityDistanceMapTable.put(distance[meters], flywheel velocity);
          velocityDistanceMapTable.put(2.16, 21.0);
          velocityDistanceMapTable.put(3.29, 24.0);
          velocityDistanceMapTable.put(4.39, 26.0);
          velocityDistanceMapTable.put(4.67, 26.0);
          velocityDistanceMapTable.put(2.49, 22.0);
          velocityDistanceMapTable.put(2.98, 23.0);
          velocityDistanceMapTable.put(4.08, 25.0);
    }

    public Command setHoodAngleCommand(DoubleSupplier angleSetpoint) {
        return new RunCommand(
                () -> hoodMechanism.setVoltage(getPIDForAngle(() -> hoodSoftLimit.limit(
                        angleSetpoint.getAsDouble()))), this);
    }

    public Command smartFlyWheelVelocity() {
        return new RunCommand(
                () -> {
                    double distance = turretRelativeDistanceFromHub.getAsDouble();
                    double velocity = velocityDistanceMap.get(distance);
                    flyWheelMechanism.setDynamicVelocity(velocity); // DIRECT motor control
                },
                this
        );
    }

    public double getPIDForAngle(DoubleSupplier angleSetpoint) {
        return angleController.calculate(hoodMotor.getMotorPosition(), angleSetpoint.getAsDouble());
    }

    @Log.NT
    public double getEncoderAngle() {
        return hoodEncoder.getAbsolutePosition().getValueAsDouble();
    }

}