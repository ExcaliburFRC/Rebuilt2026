package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Subsystems.Shooter.ShooterConstants.*;
import static frc.robot.Constants.FieldConstants.*;

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

    public Command setHoodAngleCommand(DoubleSupplier angleSetpoint){
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
                                    new TrapezoidProfile.State(angleSetpoint.getAsDouble(), FINALE_VEL)
                            );

                    double pidValue = angleController.calculate(angleSetpoint.getAsDouble(), state.position);

                    hoodMechanism.setVoltage(pidValue);
                } , this
        );
    }

    public Command flyWheelWarmupCommand(double velocity) {
        return flyWheelMechanism.smartVelocityCommand(() -> velocity, this);
    }

    public Command getFuelCommand() {
        return new RunCommand(
                () -> transportMechanism.setVoltage(TRANSPORT_VOLTAGE) , this
        ).until(hasFuel.negate()).withTimeout(0.1);
    }

    public Translation2d velocityAndAngle(Translation3d target){
        return null;
    }

    public Command shootCommand() {
        double angle = velocityAndAngle(Constants.FieldConstants.BLUE_HUB_CENTER_POSE).getAngle().getRadians();
        double flyWheelSpeed = velocityAndAngle(Constants.FieldConstants.BLUE_HUB_CENTER_POSE).getNorm();
        return new SequentialCommandGroup(
                flyWheelWarmupCommand(flyWheelSpeed),
                setHoodAngleCommand(() -> angle),
                getFuelCommand()
        );
    }
    public Command deliveryCommand(){
        double angle = velocityAndAngle(new Translation3d(0,0,0)).getAngle().getRadians();
        double flyWheelSpeed = velocityAndAngle(new Translation3d(0,0,0)).getNorm();
        return new SequentialCommandGroup(
                flyWheelWarmupCommand(flyWheelSpeed),
                setHoodAngleCommand(() -> angle),
                getFuelCommand()
        );
    }
}
