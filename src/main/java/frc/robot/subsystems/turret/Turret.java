package frc.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.math.MathUtils;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.swerve.Swerve;
import frc.robot.Constants;
import frc.robot.Superstructure;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.turret.TurretConstans.*;

public class Turret extends SubsystemBase {
    public final TalonFXMotor turretMotor;
    public final frc.excalib.mechanisms.turret.Turret turretMechanism;
    public final CANcoder turretEncoder;
    public final Supplier<Pose2d> poseSupplier;

    public Turret(Supplier<Pose2d> poseSupplier){
        turretMotor = new TalonFXMotor(TURRET_MOTOR_ID);
        turretEncoder = new CANcoder(TURRET_ENCODER_ID);
        this.poseSupplier = poseSupplier;
        turretMechanism = new frc.excalib.mechanisms.turret.Turret(
                turretMotor,
                TURRET_CONTINOUS_SOFTLIMIT,
                TURRET_GAINS,
                PID_TOLLERANCE,
                ()-> turretEncoder.getPosition().getValueAsDouble() * 2 * Math.PI
        );
    }
    public Command turnTurretCommand(Supplier<ShootingTargets> shootingTargetSupplier){
        return turretMechanism.setPositionCommand(()-> getRelativeTargetAngle(
                shootingTargetSupplier.get().getTranslation()),
                this
        );
    }
    public Translation2d getTurretPosition(){
        Pose2d robotPose = poseSupplier.get();

        double turretXPosition = robotPose.getY() +
                (TURRET_DISTENCE_FROM_ROBOT / robotPose.getRotation().getCos());
        double turretYPosition = robotPose.getX() +
                (TURRET_DISTENCE_FROM_ROBOT / robotPose.getRotation().getSin());

        return new Translation2d(turretXPosition, turretYPosition);
    }

    private Rotation2d getRelativeTargetAngle(Translation2d target){
        Pose2d robotPose = poseSupplier.get();
        Translation2d turretPosition = getTurretPosition();

        Rotation2d targetAngle = new Rotation2d(
                MathUtils.angleBetweenPoses(turretPosition,
                        target));

        return targetAngle.minus(robotPose.getRotation());
    }

}
