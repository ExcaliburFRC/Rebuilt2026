package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
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
    PIDController angleController;

    public final FlyWheel flyWheelMechanism;
    public final Mechanism hoodMechanism;
    public final Mechanism transportMechanism;

    public final TalonFXMotor transportMotor;
    public DoubleSupplier hoodAngleSupplier;
    public final SoftLimit hoodSoftLimit;

    public final Supplier<Translation2d> robotPositionSupplier;
    public final CANcoder hoodEncoder;

    public Shooter(Supplier<Translation2d> translationSupplier) {
        hoodMotor = new TalonFXMotor(HOOD_MOTOR_ID, new CANBus("Subsystems"));
        flyWheelMotor = new TalonFXMotor(FLYWHEEL_MOTOR_ID, new CANBus("Subsystems"));
        transportMotor = new TalonFXMotor(TRANSPORT_MOTOR_ID, new CANBus("Subsystems"));
        hoodEncoder = new CANcoder(HOOD_ENCODER_ID, new CANBus("Subsystems"));

        hoodEncoder.setPosition(hoodEncoder.getAbsolutePosition().getValueAsDouble());
        robotPositionSupplier = translationSupplier;

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
        hoodSoftLimit = new SoftLimit(
                () -> HOOD_MIN_ANGLE_LIMIT,
                () -> HOOD_MAX_ANGLE_LIMIT
        );


    }

    public Command setHoodAngleCommand(DoubleSupplier angleSetpoint) {
        return new RunCommand(
                () -> hoodMechanism.setVoltage(getPIDForAngle(() -> hoodSoftLimit.limit(angleSetpoint.getAsDouble()))),
                this
        );
    }

    public Command flyWheelManualCommand(double volt) {
        return flyWheelMechanism.manualCommand(() -> volt, this);
    }


    public double getPIDForAngle(DoubleSupplier angleSetpoint) {
        double val = angleController.calculate(getHoodMotorAngle(), angleSetpoint.getAsDouble());
        System.out.println(val);
        return val;
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

    public Command transportManualCommand(double voltage) {
        return transportMechanism.manualCommand(() -> voltage, this);
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
