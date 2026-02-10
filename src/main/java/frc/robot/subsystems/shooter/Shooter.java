package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.*;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.mechanisms.Mechanism;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import frc.robot.Constants;
import frc.robot.superstructure.Superstructure;
import monologue.Annotations.Log;
import monologue.Logged;

import javax.swing.*;
import javax.swing.plaf.PanelUI;
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

    public final Supplier<Pose2d> robotPositionSupplier;

    public final DoubleSupplier turretRelativeDistanceFromHub;
    public final CANcoder hoodEncoder;

    public final InterpolatingDoubleTreeMap angleDistanceMap;

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
        hoodAngleSupplier = () -> (-hoodEncoder.getPosition().getValueAsDouble() * POSITION_CONVERSION_FACTOR) + 0.69;

        hoodMotor.setPositionConversionFactor(0.048869);
        hoodMotor.setMotorPosition(hoodAngleSupplier.getAsDouble());

        transportMechanism = new Mechanism(transportMotor);
        hoodMechanism = new Mechanism(hoodMotor);

        flyWheelMechanism = new FlyWheel(flyWheelMotor, FLY_WHEEL_MAX_ACCELERATION, FLY_WHEEL_MAX_JERK, FLYWHEEL_GAINS);

        robotPositionSupplier = poseSupplier;

        angleDistanceMap = new InterpolatingDoubleTreeMap();
        initMap(angleDistanceMap);

//        hoodSoftLimit = new SoftLimit(
//                () -> HOOD_MIN_ANGLE_LIMIT,
//                () -> {
//                    if ((robotPositionSupplier.get().getTranslation().getDistance(Constants.FieldConstants.BLUE_DOWN_FIELD_TRENCH_POSE) <= Constants.FieldConstants.SHOOTER_TO_TRENCH_LIMIT)
//                            || (robotPositionSupplier.get().getTranslation().getDistance(Constants.FieldConstants.BLUE_UP_FIELD_TRENCH_POSE) <= Constants.FieldConstants.SHOOTER_TO_TRENCH_LIMIT)) {
//                        return HOOD_MAX_ANGLE_LIMIT_IN_TRENCH;
//                    } else {
//                        return HOOD_MAX_ANGLE_LIMIT;
//                    }
//                }
//        );

        hoodSoftLimit = new SoftLimit(() -> HOOD_MIN_ANGLE_LIMIT, () -> HOOD_MAX_ANGLE_LIMIT);


    }


    public void initMap(InterpolatingTreeMap<Double, Double> angleDistanceMapTable) {
        angleDistanceMapTable.put(1.0, 0.9);
        angleDistanceMapTable.put(2.0, 0.85);
        angleDistanceMapTable.put(3.0, 0.8);
        angleDistanceMapTable.put(4.0, 0.75);
    }

    public Command setHoodAngleCommand(DoubleSupplier angleSetpoint) {
        return new RunCommand(
                () -> hoodMechanism.setVoltage(getPIDForAngle(() -> hoodSoftLimit.limit(
                        angleSetpoint.getAsDouble()))), this);
    }

    public Command smartHoodAngleCommand() {
        return new RunCommand(
                () -> hoodMechanism.setVoltage(getPIDForAngle(() -> hoodSoftLimit.limit(
                        angleDistanceMap.get(this.turretRelativeDistanceFromHub.getAsDouble())))), this);
    }




    public double getPIDForAngle(DoubleSupplier angleSetpoint) {
        return angleController.calculate(getHoodMotorAngle(), angleSetpoint.getAsDouble());
    }


    public Command prepareShooterAccordingToHubCommand() {
        return new ParallelCommandGroup(
                setFlyWheelVelocityCommand(() -> STATIC_SHOOTING_VELOCITY),
                setHoodAngleCommand(
                        () -> angleDistanceMap.get(turretRelativeDistanceFromHub.getAsDouble())
                )
        );
    }

    public Command setFlyWheelVelocityCommand(DoubleSupplier velocity) {
        return flyWheelMechanism.setDynamicVelocityCommand(velocity);
    }

    public Command getFuelCommand() {
        return new RunCommand(() -> transportMechanism.setVoltage(TRANSPORT_VOLTAGE));
    }


    @Log.NT
    public double getHoodMotorAngle() {
        return hoodMotor.getMotorPosition();
    }

    @Log.NT
    public boolean isInTolerance() {
        return angleController.atSetpoint();
    }


}