package frc.robot.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import frc.excalib.swerve.Swerve;

import java.util.function.Supplier;

public class Shooter {

    public final TalonFXMotor flyWheelMotor;
    public final TalonFXMotor cHoodMotor;
    public final FlyWheel flyWheelMechanism;
    public final Supplier<Pose2d> robotPose;


    public Shooter(Swerve swerve) {
        this.flyWheelMotor = new TalonFXMotor(ShooterConstants.FlyWheelMotorID);
        this.cHoodMotor = new TalonFXMotor(ShooterConstants.cHoodMotorID);
        this.flyWheelMechanism = new FlyWheel(this.flyWheelMotor, ShooterConstants.flyWheelMaxAcceleration, ShooterConstants.flyWheelMaxJerk, ShooterConstants.flyWheelGains);
        this.robotPose = swerve::getPose2D;
    }

    public double flyWheelVelocity() {
        
    }



}
