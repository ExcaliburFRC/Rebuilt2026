package frc.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.robot.util.Target;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.SUBSYSTEMS_CANBUS;
import static frc.robot.subsystems.turret.TurretConstants.*;

public class Turret extends SubsystemBase implements Logged {
    public final TalonFXMotor turretMotor;
    public final CANcoder turretEncoder;

    public final frc.excalib.mechanisms.turret.Turret turretMechanism;

    public final DoubleSupplier turretAngleSupplier;

    public final DoubleSupplier turretRelativeAngleToTarget;
    public final DoubleSupplier robotAngleSupplier;

    public final Trigger isTurretAlligned;
    public Target turretTarget;

    public Turret(DoubleSupplier turretRelativeAngleToTarget, DoubleSupplier robotAngleSupplier) {
        turretMotor = new TalonFXMotor(TURRET_MOTOR_ID, SUBSYSTEMS_CANBUS);
        turretEncoder = new CANcoder(TURRET_ENCODER_ID, SUBSYSTEMS_CANBUS);
        turretEncoder.setPosition(turretEncoder.getAbsolutePosition().getValueAsDouble());
        turretAngleSupplier = () -> turretEncoder.getPosition().getValueAsDouble() * ENCODER_POSITION_CONVERSION_FACTOR;
        turretMotor.setMotorPosition(turretEncoder.getPosition().getValueAsDouble());

        turretMotor.setCurrentLimit(120, 80);
        this.turretRelativeAngleToTarget = turretRelativeAngleToTarget;
        turretMotor.setInverted(DirectionState.REVERSE);
        turretTarget = Target.HUB;

        this.robotAngleSupplier = robotAngleSupplier;
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
                turretMotor::getMotorPosition,
                new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 100)
        );

        isTurretAlligned = new Trigger(
                () -> {
                    if (turretTarget.equals(Target.IDLE)) {
                        return true;
                    } else if (turretTarget.equals(Target.HUB) || turretTarget.equals(Target.DELIVERY)) {
                        return Math.abs(
                                turretMechanism.getPosition().getRadians() -
                                        SOFT_LIMIT.limit(
                                                TURRET_CONTINUOUS_SOFTLIMIT.getSetpoint(
                                                        turretAngleSupplier.getAsDouble(),
                                                        turretRelativeAngleToTarget.getAsDouble()))) < PID_TOLERANCE;
                    }
                    return false;
                }
        );
    }

    public Command defaultCommand() {
        Command c = new ConditionalCommand(
                turretMechanism.stopTurret(),
                setPositionCommand(
                        () -> Rotation2d.fromRadians(
                                SOFT_LIMIT.limit(
                                        TURRET_CONTINUOUS_SOFTLIMIT.getSetpoint(
                                                turretAngleSupplier.getAsDouble(),
                                                turretRelativeAngleToTarget.getAsDouble())
                                )
                        )
                ),
                () -> turretTarget.equals(Target.IDLE)
        );
        c.addRequirements(this);
        return c;
    }

    public Command setPositionCommand(Supplier<Rotation2d> position) {
        return turretMechanism.setPositionCommand(position, this);
    }

    public Command setPositionFieldRelativeCommand(Supplier<Rotation2d> angle) {
        return setPositionCommand(
                () -> new Rotation2d(
                        (Math.PI - robotAngleSupplier.getAsDouble() + angle.get().getRadians())
                )
        );
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
    public double getTurretPointingAtHubSetpoint() {
        return turretRelativeAngleToTarget.getAsDouble();
    }

    @Log
    public double getLimitedTurretPointingAtHubSetpoint() {
        return SOFT_LIMIT.limit(
                TURRET_CONTINUOUS_SOFTLIMIT.getSetpoint(
                        turretAngleSupplier.getAsDouble(),
                        turretRelativeAngleToTarget.getAsDouble()));

    }


    @Log.NT
    public String getTarget() {
        return turretTarget.name();
    }

    @Log.NT
    public double getTurretOnFieldAngle() {
        return Units.radiansToDegrees(robotAngleSupplier.getAsDouble() + (turretMechanism.getPosition().getRadians()) - Math.PI);
    }

    @Log.NT
    public double getEncoderValue(){
        return turretAngleSupplier.getAsDouble();
    }

    @Log.NT
    public boolean isTurretAlligned() {
        return isTurretAlligned.getAsBoolean();
    }

    @Log.NT
    public String getTurretTarget() {
        return turretTarget.name();
    }
}