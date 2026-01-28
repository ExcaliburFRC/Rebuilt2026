package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import frc.robot.Constants;
import jdk.jfr.Frequency;
import jdk.jfr.Name;
import jdk.jfr.Registered;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.superstructure.Superstructure.CANivoreBus;

public class Shooter extends SubsystemBase {

    public final TalonFXMotor hoodMotor, flyWheelMotor, supportWheelMotor;

    public final FlyWheel flyWheelMechanism;
    public final Mechanism supportWheelMechanism, hoodMechanism;
    public final Mechanism transportMechanism;

    public final TalonFXMotor transportMotor;
    public final DoubleSupplier hoodAngleSupplier;
    public final SoftLimit hoodSoftLimit;

    public final Supplier<Translation2d> robotPositionSupplier;

    public Shooter(Supplier<Translation2d> translationSupplier) {
        hoodMotor = new TalonFXMotor(HOOD_MOTOR_ID, CANivoreBus);
        flyWheelMotor = new TalonFXMotor(FLYWHEEL_MOTOR_ID, CANivoreBus);
        supportWheelMotor = new TalonFXMotor(SUPPORT_WHEEL_MOTOR_ID, CANivoreBus);
        transportMotor = new TalonFXMotor(TRANSPORT_MOTOR_ID, CANivoreBus);

        robotPositionSupplier = translationSupplier;
        hoodAngleSupplier = () -> (hoodMotor.getPosition().getValueAsDouble() * POSITION_CONVERSION_FACTOR);

        transportMechanism = new Mechanism(transportMotor);
        hoodMechanism = new Mechanism(hoodMotor);
        supportWheelMechanism = new Mechanism(supportWheelMotor);

        flyWheelMechanism = new FlyWheel(flyWheelMotor, FLY_WHEEL_MAX_ACCELERATION, FLY_WHEEL_MAX_JERK, FLYWHEEL_GAINS);

        hoodSoftLimit = new SoftLimit(
                () -> HOOD_MIN_ANGLE_LIMIT,
                () -> {
                    if ((robotPositionSupplier.get().getDistance(Constants.FieldConstants.BLUE_DOWN_FIELD_TRENCH_POSE) <= Constants.FieldConstants.SHOOTER_TO_TRENCH_LIMET)
                            || (robotPositionSupplier.get().getDistance(Constants.FieldConstants.BLUE_UP_FIELD_TRENCH_POSE) <= Constants.FieldConstants.SHOOTER_TO_TRENCH_LIMET)) {
                        return HOOD_MAX_ANGLE_LIMIT_IN_TRENCH;
                    } else {
                        return HOOD_MAX_ANGLE_LIMIT;
                    }
                }
        );


    }

    public Command setHoodAngleCommand(DoubleSupplier angleSetpoint) {
        TrapezoidProfile trapezoidProfile = new TrapezoidProfile(HOOD_CONSTRAINTS);
        PIDController angleController = new PIDController(HOOD_PID_GAINS.kp, HOOD_PID_GAINS.ki, HOOD_PID_GAINS.kd);

        return new RunCommand(
                () -> {
                    TrapezoidProfile.State state =
                            trapezoidProfile.calculate(
                                    0.02,
                                    new TrapezoidProfile.State(
                                            hoodAngleSupplier.getAsDouble(),
                                            hoodMotor.getMotorVelocity()),
                                    new TrapezoidProfile.State(angleSetpoint.getAsDouble(), FINAL_VEL)
                            );

                    double pidValue = angleController.calculate(state.position, angleSetpoint.getAsDouble());

                    hoodMechanism.setVoltage(pidValue);
                }
        );
    }

    public Command setFlyWheelVelocityCommand(DoubleSupplier velocity) {
        return flyWheelMechanism.smartVelocityCommand(velocity);
    }

    public Command getFuelCommand() {
        return new RunCommand(() -> transportMechanism.setVoltage(TRANSPORT_VOLTAGE));
    }


    public Command adjustShooterForShootingCommand(DoubleSupplier hoodAngleSupplier, DoubleSupplier rollerRadPerSec) {
        Command command =  new ParallelCommandGroup(
                setHoodAngleCommand(hoodAngleSupplier),
                flyWheelMechanism.smartVelocityCommand(rollerRadPerSec)
        );
        command.addRequirements(this);
        return command;
    }
}
