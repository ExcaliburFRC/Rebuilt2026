package frc.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.robot.util.Target;
import monologue.Annotations.Log;
import monologue.Logged;

import javax.xml.crypto.dsig.spec.XSLTTransformParameterSpec;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.turret.TurretConstants.*;

public class Turret extends SubsystemBase implements Logged {
    public final TalonFXMotor turretMotor;
    public final frc.excalib.mechanisms.turret.Turret turretMechanism;
    public final DoubleSupplier turretAngleSupplier;
    public final CANcoder turretEncoder;
    public final DoubleSupplier turretRelativeAngleToTarget;
    public Target turretTarget;

    public Turret(DoubleSupplier turretRelativeAngleToTarget) {
        turretMotor = new TalonFXMotor(TURRET_MOTOR_ID, SUBSYSTEMS_CANBUS);
        turretEncoder = new CANcoder(TURRET_ENCODER_ID, SUBSYSTEMS_CANBUS);
        turretEncoder.setPosition(turretEncoder.getAbsolutePosition().getValueAsDouble());
        turretAngleSupplier = () -> turretEncoder.getPosition().getValueAsDouble() * ENCODER_POSITION_CONVERSION_FACTOR;
        turretMotor.setMotorPosition(turretEncoder.getPosition().getValueAsDouble());

        turretMotor.setCurrentLimit(120, 80);
        this.turretRelativeAngleToTarget = turretRelativeAngleToTarget;
        turretMotor.setInverted(DirectionState.REVERSE);
        turretTarget = Target.IDLE;

//      turretMotor.setMotorPosition(turretAngleSupplier.getAsDouble());
        turretMotor.setPositionConversionFactor(MOTOR_POSITION_CONVERSION_FACTOR);
        turretMotor.setVelocityConversionFactor(MOTOR_POSITION_CONVERSION_FACTOR);


        turretMotor.setIdleState(IdleState.BRAKE);
        turretMotor.setMotorPosition(turretAngleSupplier.getAsDouble());


        turretMechanism = new frc.excalib.mechanisms.turret.Turret(
                turretMotor,
                TURRET_CONTINUOUS_SOFTLIMIT,
                TURRET_GAINS,
                PID_TOLERANCE,
                this::getEncoderPosition,
                new TrapezoidProfile.Constraints(Math.PI * 60, Math.PI * 100)
        );

        setDefaultCommand(defaultCommand());
    }


    public Command defaultCommand() {
        Command c = new ConditionalCommand(
                Commands.none(),
                setPositionCommand(
                        () -> Rotation2d.fromRadians(
                                SOFT_LIMIT.limit(
                                        TURRET_CONTINUOUS_SOFTLIMIT.getSetpoint(
                                                turretAngleSupplier.getAsDouble(),
                                                turretRelativeAngleToTarget.getAsDouble())
                                )
                        )
                ).alongWith(new PrintCommand("" +
                        Rotation2d.fromRadians(
                                SOFT_LIMIT.limit(
                                        TURRET_CONTINUOUS_SOFTLIMIT.getSetpoint(
                                                turretAngleSupplier.getAsDouble(),
                                                turretRelativeAngleToTarget.getAsDouble())

                                )))).alongWith(
                        new PrintCommand(
                                "measurment" + turretAngleSupplier.getAsDouble()
                        ).alongWith(
                                new PrintCommand(
                                        "setpoint" + turretRelativeAngleToTarget.getAsDouble()
                                ))
                ),
                () -> turretTarget.equals(Target.IDLE)
        );
        c.addRequirements(this);
        return c;
    }


    public Command setPositionCommand(Supplier<Rotation2d> position) {
        return turretMechanism.setPositionCommand(position, this);
    }

    public Command targetHubCommand() {
        return new InstantCommand(() -> turretTarget = Target.HUB, this);
    }

    public Command targetDeliveryCommand() {
        return new InstantCommand(() -> turretTarget = Target.DELIVERY, this);
    }

    public Command idleCommand() {
        return new InstantCommand(() -> turretTarget = Target.IDLE, this);
    }

    @Log.NT
    public double getEncoderPosition() {
        return turretAngleSupplier.getAsDouble();
    }

    @Log.NT
    public double getTurretRelativeAngleToTarget() {
        return turretRelativeAngleToTarget.getAsDouble();
    }

    @Log.NT
    public String getTarget() {
        return turretTarget.name();
    }

}