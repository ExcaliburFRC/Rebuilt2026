package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.math.MathUtils;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.robot.util.ShootingTarget;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.turret.TurretConstants.*;

public class Turret extends SubsystemBase implements Logged {
    public final TalonFXMotor turretMotor;
    public final frc.excalib.mechanisms.turret.Turret turretMechanism;
    public final DoubleSupplier turretAngleSupplier;
    public final CANcoder turretEncoder;
    public final DoubleSupplier turretRelativeAngleToHub;
    public ShootingTarget currentTarget = ShootingTarget.HUB;

    public Turret(DoubleSupplier turretRelativeAngleToHub) {
        turretMotor = new TalonFXMotor(TURRET_MOTOR_ID, new CANBus("Subsystems"));
        turretEncoder = new CANcoder(TURRET_ENCODER_ID, new CANBus("Subsystems"));
        turretEncoder.setPosition(turretEncoder.getAbsolutePosition().getValueAsDouble());
        turretAngleSupplier = () -> turretEncoder.getPosition().getValueAsDouble() * ENCODER_POSITION_CONVERSION_FACTOR;
        turretMotor.setMotorPosition(turretEncoder.getPosition().getValueAsDouble());

        this.turretRelativeAngleToHub = turretRelativeAngleToHub;
        turretMotor.setInverted(DirectionState.FORWARD);

//      turretMotor.setMotorPosition(turretAngleSupplier.getAsDouble());
        turretMotor.setPositionConversionFactor(MOTOR_POSITION_CONVERSION_FACTOR);
        turretMotor.setVelocityConversionFactor(MOTOR_POSITION_CONVERSION_FACTOR);


        turretMotor.setIdleState(IdleState.COAST);
        turretMotor.setMotorPosition(turretAngleSupplier.getAsDouble());


        turretMechanism = new frc.excalib.mechanisms.turret.Turret(
                turretMotor,
                TURRET_CONTINUOUS_SOFTLIMIT,
                TURRET_GAINS,
                PID_TOLERANCE,
                this::getEncoderPosition,
                new TrapezoidProfile.Constraints(Math.PI * 4, Math.PI * 50)
        );

//        setDefaultCommand(followTargetCommand());
    }

    public Command setTargetCommand(ShootingTarget targetToSet) {
        return new InstantCommand(() -> currentTarget = targetToSet);
    }

    public Command speedRelativeFollowHubCommand() {
        return turretMechanism.setPositionCommand(
                () -> Rotation2d.fromRadians(turretRelativeAngleToHub.getAsDouble())
        );
    }

    public Command setPositionCommand(Supplier<Rotation2d> position) {
        return turretMechanism.setPositionCommand(position, this);
    }

    @Log.NT
    public double getEncoderPosition() {
        return turretAngleSupplier.getAsDouble();
    }

    @Log.NT
    public double getError() {
        return turretMechanism.m_anglePIDcontroller.getPositionError();
    }

    @Log.NT
    public boolean isInTolerance() {
        return turretMechanism.m_anglePIDcontroller.atSetpoint();
    }

}