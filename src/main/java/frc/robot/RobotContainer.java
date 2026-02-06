// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.turret.Turret;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.excalib.control.math.Vector2D;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.superstructure.Superstructure;
import frc.robot.util.ShooterPhysics;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.util.ShootingTarget;
import monologue.Annotations.Log;
import monologue.Logged;

import static frc.robot.Constants.PRIMARY_CONTROLLER_PORT;
import static frc.robot.Constants.SwerveConstants.MAX_OMEGA_RAD_PER_SEC;
import static frc.robot.Constants.SwerveConstants.MAX_VEL;


public class RobotContainer extends TimedRobot implements Logged {
    public final ShooterPhysics shooterPhysics;

    public final Swerve swerve = Constants.SwerveConstants.configureSwerve(Constants.INITIAL_POSE);

    public final CommandPS5Controller primary = new CommandPS5Controller(PRIMARY_CONTROLLER_PORT);

    public static ShootingTarget shootingTarget = ShootingTarget.HUB;
    public final Superstructure superstructure = new Superstructure(primary, swerve);

    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        shooterPhysics = new ShooterPhysics(
                swerve::getApproximatedFuturePose2D,
                () -> swerve.getRobotRelativeSpeeds().vxMetersPerSecond,
                () -> swerve.getRobotRelativeSpeeds().vyMetersPerSecond
        );
        swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)));

        autoChooser.setDefaultOption("/ null Auto", "/ null Auto");

        for (String autoName : AutoBuilder.getAllAutoNames()) {
            autoChooser.addOption(autoName, autoName);
        }

        SmartDashboard.putData("Auto Chooser", autoChooser);


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
         primary.cross().toggleOnTrue(superstructure.testShotCommand(Math.PI/4, 0.6, 40));
//        primary.circle().toggleOnTrue(superstructure.flyWheelCommand());

        swerve.setDefaultCommand(
                swerve.driveCommand(
                        () -> new Vector2D(
                                applyDeadband(-primary.getLeftY()) * MAX_VEL,
                                applyDeadband(-primary.getLeftX()) * MAX_VEL),
                        () -> applyDeadband(primary.getRightX()) *
                                MAX_OMEGA_RAD_PER_SEC,
                        () -> true
                )
        );

        primary.PS().onTrue(new InstantCommand(() -> swerve.resetOdometry(new Pose2d())).ignoringDisable(true));

    }



    public Command getAutonomousCommand() {
        String selectedAuto = autoChooser.getSelected();

        if (selectedAuto == null || selectedAuto.equals("/ null Auto")) {
            return new PrintCommand("This auto is empty");
        }

        return AutoBuilder.buildAuto(selectedAuto);
    }

    public double applyDeadband(double val) {
        return Math.abs(val) < 0.09 ? 0 : val;
    }

    public void registerCommands() {
        NamedCommands.registerCommand("floorIntake", new PrintCommand("floorIntake"));
        NamedCommands.registerCommand("closeIntake", new PrintCommand("closeIntake"));
        NamedCommands.registerCommand("prepareShooter", new PrintCommand("prepareShooter"));
        NamedCommands.registerCommand("shoot", new PrintCommand("Shoot"));
        NamedCommands.registerCommand("extendClimber", new PrintCommand("extentClimber"));
        NamedCommands.registerCommand("retractClimber", new PrintCommand("retractClimber"));
        NamedCommands.registerCommand("retractIntake", new PrintCommand("retractIntake"));
        NamedCommands.registerCommand("depotIntake", new PrintCommand("depotIntake"));
        SmartDashboard.putNumber("match time", DriverStation.getMatchTime());


    }

    @Log.NT
    public double getLeftX(){
        return primary.getLeftX();
    }

}
