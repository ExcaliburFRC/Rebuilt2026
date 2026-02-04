package frc.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.math.MathUtils;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.robot.util.ShootingTarget;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.turret.TurretConstants.*;

public class Turret extends SubsystemBase {
    public final TalonFXMotor turretMotor;
    public final frc.excalib.mechanisms.turret.Turret turretMechanism;
    public final DoubleSupplier turretAngleSupplier;
    public final CANcoder turretEncoder;
    public ShootingTarget currentTarget = ShootingTarget.HUB;
    public final DoubleSupplier turretRelativeAngleToHub;


    public Turret(DoubleSupplier turretRelativeAngleToHub) {
        turretMotor = new TalonFXMotor(TURRET_MOTOR_ID);
        turretEncoder = new CANcoder(TURRET_ENCODER_ID);

        this.turretRelativeAngleToHub = turretRelativeAngleToHub;

        turretAngleSupplier = () -> turretEncoder.getPosition().getValueAsDouble() * ROTATIONS_TO_RAD;

        turretMotor.setMotorPosition(turretAngleSupplier.getAsDouble());

        turretMotor.setIdleState(IdleState.BRAKE);
        turretMotor.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
        turretMotor.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

        turretEncoder.getPosition().getValueAsDouble();


        turretMechanism = new frc.excalib.mechanisms.turret.Turret(
                turretMotor,
                TURRET_CONTINOUS_SOFTLIMIT,
                TURRET_GAINS,
                PID_TOLERANCE,
                turretAngleSupplier
        );

        setDefaultCommand(followTargetCommand());
    }

    public Command setTargetCommand(ShootingTarget targetToSet) {
        return new InstantCommand(() -> currentTarget = targetToSet);
    }

    public Command followTargetCommand() {
        return turretMechanism.setPositionCommand(
                () -> Rotation2d.fromRadians(
                        TURRET_SOFTLIMIT.limit(turretRelativeAngleToHub.getAsDouble())
                )
        );
    }

}
