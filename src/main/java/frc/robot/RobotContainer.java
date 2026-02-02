// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.ShooterPhysics;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.util.ShootingTarget;
import monologue.Annotations.Log;
import monologue.Logged;

import static frc.robot.Constants.PRIMARY_CONTROLLER_PORT;


public class RobotContainer implements Logged {
    public final ShooterPhysics shooterPhysics;

    public final Swerve swerve = Constants.SwerveConstants.configureSwerve(Constants.INITIAL_POSE);

    public final CommandPS5Controller primary = new CommandPS5Controller(PRIMARY_CONTROLLER_PORT);

    public static ShootingTarget shootingTarget = ShootingTarget.HUB;

//    public static Turret turret;

    public static Intake intake;


    public RobotContainer() {
        shooterPhysics = new ShooterPhysics(
                swerve::getApproximatedFuturePose2D,
                () -> swerve.getRobotRelativeSpeeds().vxMetersPerSecond,
                () -> swerve.getRobotRelativeSpeeds().vyMetersPerSecond
        );


//        turret = new Turret(swerve::getPose2D);
        intake = new Intake();


        configureBindings();
        registerCommands();
    }


    private void configureBindings() {

//        swerve.setDefaultCommand(
//                swerve.driveCommand(
//                        () -> new Vector2D(
//                                applyDeadband(-primary.getLeftY()) * MAX_VEL,
//                        applyDeadband(-primary.getLeftX()) * MAX_VEL),
//                        () -> applyDeadband(-primary.getRightX()) * MAX_OMEGA_RAD_PER_SEC,
//                        () -> false
//                )
//        );

        //turret.setDefaultCommand(turret.setPositionCommand(() -> new Rotation2d(primary.getLeftX())));
//        primary.triangle().toggleOnTrue(turret.setPositionCommand(new Rotation2d(primary.getLeftX()*0.5)));
        // primary.circle().toggleOnTrue(turret.setPositionCommand(()->new Rotation2d(0)));
//            primary.circle().whileTrue(intake.openIntakeCommand());

            primary.triangle().toggleOnTrue(intake.angleManualCommand(()-> 6));
    }



    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public double applyDeadband(double val) {
        return Math.abs(val) < 0.09 ? 0 : val;
    }

    public void registerCommands() {
        NamedCommands.registerCommand("floorIntake", new InstantCommand());
        NamedCommands.registerCommand("prepareShooter", new InstantCommand());
        NamedCommands.registerCommand("shoot", new InstantCommand());
        NamedCommands.registerCommand("extendClimber", new InstantCommand());
        NamedCommands.registerCommand("retractClimber", new InstantCommand());
        NamedCommands.registerCommand("retractIntake", new InstantCommand());
    }

    @Log.NT
    public double getLeftX(){
        return primary.getLeftX()*1.5;
    }

}
