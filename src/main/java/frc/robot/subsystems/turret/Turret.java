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

import java.util.function.Supplier;

import static frc.robot.subsystems.turret.TurretConstants.*;

public class Turret extends SubsystemBase {
    public final TalonFXMotor turretMotor;
    public final frc.excalib.mechanisms.turret.Turret turretMechanism;
    public final CANcoder turretEncoder;
    public final Supplier<Pose2d> robotPoseSupplier;
    public ShootingTargets currentTarget = ShootingTargets.HUB;

    public Turret(Supplier<Pose2d> poseSupplier) {
        turretMotor = new TalonFXMotor(TURRET_MOTOR_ID);
        turretEncoder = new CANcoder(TURRET_ENCODER_ID);

        this.robotPoseSupplier = poseSupplier;

        turretMechanism = new frc.excalib.mechanisms.turret.Turret(
                turretMotor,
                TURRET_CONTINOUS_SOFTLIMIT,
                TURRET_GAINS,
                PID_TOLLERANCE,
                () -> turretEncoder.getPosition().getValueAsDouble() * ROTATIONS_TO_RAD
        );

        setDefaultCommand(followTargetCommand());
    }

    public Command setTargetCommand(ShootingTargets targetToSet) {
        return new InstantCommand(() -> currentTarget = targetToSet);
    }

    public Command followTargetCommand() {
        return turretMechanism.setPositionCommand(
                () -> getRelativeTargetAngle(
                        currentTarget.getTranslation()),
                this
        );
    }

    public Translation2d getTurretFieldRelativePosition() {
        Pose2d robotPose = robotPoseSupplier.get();

        double turretXPosition = robotPose.getY() +
                (TURRET_OFFSET_RELATIVE_ROBOT.getX() + robotPose.getRotation().getCos());
        double turretYPosition = robotPose.getX() +
                (TURRET_OFFSET_RELATIVE_ROBOT.getY() + robotPose.getRotation().getSin());

        return new Translation2d(turretXPosition, turretYPosition);
    }

    private Rotation2d getRelativeTargetAngle(Translation2d target) {
        Pose2d robotPose = robotPoseSupplier.get();
        Translation2d turretPosition = getTurretFieldRelativePosition();

        Rotation2d targetAngle = new Rotation2d(
                MathUtils.getPosesTangentAngle(turretPosition,
                        target));

        return targetAngle.minus(robotPose.getRotation());
    }

}
