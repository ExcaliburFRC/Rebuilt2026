package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import edu.wpi.first.wpilibj2.command.*;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.mechanisms.Mechanism;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import frc.robot.Constants;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class Shooter extends SubsystemBase implements Logged {

    public final TalonFXMotor hoodMotor, flyWheelMotor;
    PIDController angleController;

    public final FlyWheel flyWheelMechanism;
    public final Mechanism hoodMechanism;
    public final Mechanism transportMechanism;

    public final TalonFXMotor transportMotor;
    public DoubleSupplier hoodAngleSupplier;
    public final SoftLimit hoodSoftLimit;

    public final DoubleSupplier distanceFromHubSupplier;
    public final CANcoder hoodEncoder;

    public BooleanSupplier shooting = () -> false;

    public final InterpolatingDoubleTreeMap distanceAngleMap;

    public Shooter(DoubleSupplier distanceFromHubSupplier) {
        hoodMotor = new TalonFXMotor(HOOD_MOTOR_ID, new CANBus("Subsystems"));
        flyWheelMotor = new TalonFXMotor(FLYWHEEL_MOTOR_ID, new CANBus("Subsystems"));
        transportMotor = new TalonFXMotor(TRANSPORT_MOTOR_ID, new CANBus("Subsystems"));
        hoodEncoder = new CANcoder(HOOD_ENCODER_ID, new CANBus("Subsystems"));

        this.distanceFromHubSupplier = distanceFromHubSupplier;

        angleController = new PIDController(HOOD_PID_GAINS.kp, HOOD_PID_GAINS.ki, HOOD_PID_GAINS.kd);
        angleController.setTolerance(0);

        hoodMotor.setIdleState(IdleState.COAST);
        hoodMotor.setInverted(DirectionState.FORWARD);
        hoodAngleSupplier = () -> (hoodEncoder.getAbsolutePosition().getValueAsDouble() * POSITION_CONVERSION_FACTOR);

        hoodMotor.setMotorPosition(hoodEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI);
        hoodMotor.setPositionConversionFactor(POSITION_CONVERSION_FACTOR * 2 * Math.PI);

        transportMechanism = new Mechanism(transportMotor);
        hoodMechanism = new Mechanism(hoodMotor);

        flyWheelMechanism = new FlyWheel(flyWheelMotor, FLY_WHEEL_MAX_ACCELERATION, FLY_WHEEL_MAX_JERK, FLYWHEEL_GAINS);


        distanceAngleMap = new InterpolatingDoubleTreeMap();
        initDistanceAngleMap();

//        hoodSoftLimit = new SoftLimit(
//                () -> HOOD_MIN_ANGLE_LIMIT,
//                () -> {
//                    if ((robotPositionSupplier.get().getDistance(Constants.FieldConstants.BLUE_DOWN_FIELD_TRENCH_POSE) <= Constants.FieldConstants.SHOOTER_TO_TRENCH_LIMET)
//                            || (robotPositionSupplier.get().getDistance(Constants.FieldConstants.BLUE_UP_FIELD_TRENCH_POSE) <= Constants.FieldConstants.SHOOTER_TO_TRENCH_LIMET)) {
//                        return HOOD_MAX_ANGLE_LIMIT_IN_TRENCH;
//                    } else {
//                        return HOOD_MAX_ANGLE_LIMIT;
//                    }
//                }
//        );

        setDefaultCommand(defaultCommand());

        hoodSoftLimit = new SoftLimit(
                () -> HOOD_MIN_ANGLE_LIMIT,
                () -> HOOD_MAX_ANGLE_LIMIT
        );
    }

    public void initDistanceAngleMap() {
        //todo
    }

    public Command defaultCommand() {
        double SHOOTING_VELOCITY = 0;
        double DEFAULT_VELOCITY = 0;
        return new ParallelCommandGroup(
                transportMechanism.manualCommand(
                        () -> shooting.getAsBoolean() ? TRANSPORT_VOLTAGE : 0
                ),
                flyWheelMechanism.setDynamicVelocityCommand(
                        () -> shooting.getAsBoolean() ? SHOOTING_VELOCITY : DEFAULT_VELOCITY
                ),
                setHoodAngleCommand(
                        () -> distanceAngleMap.get(distanceFromHubSupplier.getAsDouble())
                )
        );
    }

    public Command setHoodAngleCommand(DoubleSupplier angleSetpoint) {
        return new RunCommand(
                () -> hoodMechanism.setVoltage(getPIDForAngle(() -> hoodSoftLimit.limit(angleSetpoint.getAsDouble()))),
                this
        );
    }

    public double getPIDForAngle(DoubleSupplier angleSetpoint) {
        return angleController.calculate(getHoodMotorAngle(), hoodSoftLimit.limit(angleSetpoint.getAsDouble()));

    }

    public Command setFlyWheelVelocityCommand(DoubleSupplier velocity) {
        return flyWheelMechanism.smartVelocityCommand(velocity);
    }

    public Command getFuelCommand() {
        return new RunCommand(() -> transportMechanism.setVoltage(TRANSPORT_VOLTAGE));
    }


    public Command adjustShooterForShootingCommand(DoubleSupplier hoodAngleSupplier, DoubleSupplier rollerRadPerSec) {
        Command command = new ParallelCommandGroup(
                setHoodAngleCommand(hoodAngleSupplier),
                flyWheelMechanism.smartVelocityCommand(rollerRadPerSec)
        );
        command.addRequirements(this);
        return command;
    }


    @Log.NT
    public double getHoodMotorAngle() {
        return hoodMotor.getMotorPosition();
    }

    @Log.NT
    public boolean isInTolerance(){
        return angleController.atSetpoint();
    }
}
