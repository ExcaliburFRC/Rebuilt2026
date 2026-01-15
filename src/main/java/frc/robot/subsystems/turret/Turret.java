package frc.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.excalib.control.math.MathUtils;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.swerve.Swerve;
import frc.robot.Constants;
import frc.robot.Superstructure;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.turret.TurretConstans.*;

public class Turret {
    public final TalonFXMotor turretMotor;
    public final frc.excalib.mechanisms.turret.Turret turretMechanism;
    public final CANcoder turretEncoder;
    public final DoubleSupplier angleSuplier;

    public Turret(){
        turretMotor = new TalonFXMotor(TURRET_MOTOR_ID);
        turretEncoder = new CANcoder(TURRET_ENCODER_ID);
        angleSuplier = ()-> turretEncoder.getPosition().getValueAsDouble() * 2 * Math.PI;
        turretMechanism = new frc.excalib.mechanisms.turret.Turret(
                turretMotor,
                TURRET_CONTINOUS_SOFTLIMIT,
                TURRET_GAINS,
                PID_TOLLERANCE,
                angleSuplier
        );
    }
//    public Command turnTurret(Translation3d target){
//        Rotation2d targetAngle = new Rotation2d(MathUtils.angleBetweenPoses(,target.toTranslation2d()));
//        return new RunCommand(turretMechanism.setPositionCommand(targetAngle,this));
//    }

}
