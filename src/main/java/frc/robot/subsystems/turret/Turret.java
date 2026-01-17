package frc.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.CANcoder;
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

import static frc.robot.subsystems.turret.TurretConstans.*;

public class Turret extends SubsystemBase {
    public final TalonFXMotor turretMotor;
    public final frc.excalib.mechanisms.turret.Turret turretMechanism;
    public final CANcoder turretEncoder;
    public final DoubleSupplier angleSupplier;
    public final Swerve swerve;

    public Turret(){
        turretMotor = new TalonFXMotor(TURRET_MOTOR_ID);
        turretEncoder = new CANcoder(TURRET_ENCODER_ID);
        swerve = Constants.SwerveConstants.configureSwerve(Constants.startingPose);
        angleSupplier = ()-> turretEncoder.getPosition().getValueAsDouble() * 2 * Math.PI;
        turretMechanism = new frc.excalib.mechanisms.turret.Turret(
                turretMotor,
                TURRET_CONTINOUS_SOFTLIMIT,
                TURRET_GAINS,
                PID_TOLLERANCE,
                angleSupplier
        );
    }
    public Command turnTurretCommand(){

        double turretXPosition = swerve.getPose2D().getY() +
                (TURRET_DISTENCE_FROM_ROBOT / Math.cos(swerve.getRotation2D().getRadians()));
        double turretYPosition = swerve.getPose2D().getX() +
                (TURRET_DISTENCE_FROM_ROBOT / Math.sin(swerve.getRotation2D().getRadians()));

        Translation2d turretPosition = new Translation2d(turretXPosition, turretYPosition);

        Rotation2d targetAngle = new Rotation2d(
                MathUtils.angleBetweenPoses(turretPosition,
                        Constants.FieldConstants.BLUE_HUB_CENTER_POSE.get().getTranslation()));
        Rotation2d relativeTargetAngle = targetAngle.minus(swerve.getRotation2D());


        return turretMechanism.setPositionCommand(()-> relativeTargetAngle, this);
    }

}
