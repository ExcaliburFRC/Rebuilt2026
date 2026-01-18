package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class Shooter extends SubsystemBase {

    public final TalonFXMotor hoodMotor, flyWheelMotor, supportWheelMotor;

    public final FlyWheel flyWheelMechanism;
    public final Mechanism supportWheelMechanism, hoodMechanism;
    public final Mechanism transportMechanism;

    public final TalonFXMotor transportMotor;
    public final DoubleSupplier hoodAngleSupplier;
    public final SoftLimit hoodSoftLimit;

    public final DigitalInput beamBreak;
    public final Trigger hasFuel;


    public Shooter() {
        hoodMotor = new TalonFXMotor(HOOD_MOTOR_ID);
        flyWheelMotor = new TalonFXMotor(FLYWHEEL_MOTOR_ID);
        supportWheelMotor = new TalonFXMotor(SUPPORT_WHEEL_MOTOR_ID);
        beamBreak = new DigitalInput(BEAM_BREAK_CHANNEL);
        transportMotor = new TalonFXMotor(TRANSPORT_MOTOR_ID);

        hoodAngleSupplier = () -> (hoodMotor.getPosition().getValueAsDouble() * POSITION_CONVERSION_FACTOR);
        transportMechanism = new Mechanism(transportMotor);
        hoodMechanism = new Mechanism(hoodMotor);
        flyWheelMechanism = new FlyWheel(flyWheelMotor, FLY_WHEEL_MAX_ACCELERATION,
                FLY_WHEEL_MAX_JERK, FLYWHEEL_GAINS);
        supportWheelMechanism = new Mechanism(supportWheelMotor);

        hoodSoftLimit = new SoftLimit(
                () -> HOOD_MIN_ANGLE_LIMIT,
                () -> HOOD_MAX_ANGLE_LIMIT
        );

        hasFuel = new Trigger(beamBreak::get);

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

                    double pidValue = angleController.calculate( state.position,angleSetpoint.getAsDouble());

                    hoodMechanism.setVoltage(pidValue);
                }, this
        );
    }

    public Command flyWheelWarmupCommand(double velocity) {
        return flyWheelMechanism.smartVelocityCommand(() -> velocity, this);
    }

    public Command setFlyWheelVelocityCommand(DoubleSupplier velocity) {
        return flyWheelMechanism.smartVelocityCommand(velocity);
    }

    public Command getFuelCommand() {
        return new RunCommand(() -> transportMechanism.setVoltage(TRANSPORT_VOLTAGE),this);
    }

    public Translation2d calculateShootParameters(Pose3d targetPose, Pose3d currentPose) {
        return null;
    }

    public Command adjustShooterForShootingCommand(Supplier<Translation2d> currentPose, Supplier<Pose3d> targetPose) {

        DoubleSupplier angle = () -> calculateShootParameters(
                targetPose.get(),
                new Pose3d(
                        new Translation3d(currentPose.get()),
                        new Rotation3d())
        ).getAngle().getRadians();

        DoubleSupplier flyWheelVelocity = () -> calculateShootParameters(
                targetPose.get(),
                new Pose3d(
                        new Translation3d(currentPose.get()),
                        new Rotation3d())
        ).getNorm();

        return new ParallelCommandGroup(
                setFlyWheelVelocityCommand(flyWheelVelocity),
                setHoodAngleCommand(angle)
        );
    }
}
