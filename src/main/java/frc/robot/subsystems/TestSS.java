package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import monologue.Logged;

public class TestSS extends SubsystemBase implements Logged {
    public TalonFXMotor motor1, motor2, stars;
    public MotorGroup motorGroup;

    public FlyWheel flyWheel;
    public Mechanism mechanism;
    Command defC;

    public TestSS() {
        motor1 = new TalonFXMotor(45);
        motor2 = new TalonFXMotor(31);
        stars = new TalonFXMotor(33);

        mechanism = new Mechanism(stars);
        motorGroup = new MotorGroup(motor1, motor2);

        flyWheel = new FlyWheel(
                motorGroup,
                30,
                30,
                new Gains(0, 0, 0, 0, 0.28248*0.4556, 0, 0)
        );

        defC = flyWheel.setDynamicVelocityCommand(() -> -95).alongWith(mechanism.manualCommand(()-> -6));
        defC.addRequirements(this);

        setDefaultCommand(defC);
    }


}
