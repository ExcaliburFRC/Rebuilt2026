package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.controller.PIDController;
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

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Subsystems.Shooter.ShooterConstants.*;

public class Shooter extends SubsystemBase {

    public final TalonFXMotor hoodMotor, flyWheelMotor, supportWheelMotor;

    public final FlyWheel flyWheelMechanism;
    public final Mechanism supportWheelMechanism, hoodMechanism;
    public final SoftLimit hoodSoftLimit;
    public final TalonFXMotor transportMotor;
    public final DoubleSupplier hoodAngleSupplier;
    public final DigitalInput beamBreak;
    public final Trigger hasFuel;
    public final Mechanism transportMechanism;

    public Shooter() {
        hoodMotor = new TalonFXMotor(HOOD_MOTOR_ID);
        flyWheelMotor = new TalonFXMotor(FLYWHEEL_MOTOR_ID);
        supportWheelMotor = new TalonFXMotor(SUPPORT_WHEEL_MOTOR_ID);
        beamBreak = new DigitalInput(BEAM_BREAK_CHANNEL);
        transportMotor = new TalonFXMotor(TRANSPORT_MOTOR_ID);

        hoodAngleSupplier = () -> (hoodMotor.getPosition().getValueAsDouble() * POSITION_CONVERSION_FACTOR);
        transportMechanism = new Mechanism(transportMotor);
        hoodMechanism = new Mechanism(hoodMotor);
        flyWheelMechanism = new FlyWheel(flyWheelMotor, FLY_WHEEL_MAX_ACCELERATION, FLY_WHEEL_MAX_JERK, FLYWHEEL_GAINS);
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
                                    new TrapezoidProfile.State(angleSetpoint.getAsDouble(), 0)
                            );

                    double pidValue = angleController.calculate(angleSetpoint.getAsDouble(), state.position);

                    hoodMechanism.setVoltage(pidValue);
                }
        );
    }

    public Command flyWheelWarmupCommand(double velocity) {
        return flyWheelMechanism.smartVelocityCommand(() -> velocity, this);
    }

    public Command getFuelCommand() {
        return new RunCommand(
                () -> transportMechanism.setVoltage(TRANSPORT_VOLTAGE)
        ).until(hasFuel.negate()).withTimeout(0.1);
    }

    public Command shootCommand() {
        double angle = 0;
        double flyWheelSpeed = 0;
        return new SequentialCommandGroup(
                flyWheelWarmupCommand(flyWheelSpeed),
                setHoodAngleCommand(() -> angle),
                getFuelCommand()
        );
    }


}
