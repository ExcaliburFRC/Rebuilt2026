// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.excalib.swerve.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.excalib.control.math.Vector2D;

import frc.robot.subsystems.intake.Intake;
import frc.robot.superstructure.Superstructure;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import monologue.Annotations;
import monologue.Logged;

import static frc.robot.Constants.PRIMARY_CONTROLLER_PORT;
import static frc.robot.Constants.SwerveConstants.MAX_OMEGA_RAD_PER_SEC;
import static frc.robot.Constants.SwerveConstants.MAX_VEL;
import static monologue.Annotations.*;


public class RobotContainer implements Logged {

    public final Swerve swerve = Constants.SwerveConstants.configureSwerve(Constants.INITIAL_POSE);

    public Pose2d auroraPose = new Pose2d();
    public final CommandPS5Controller primary = new CommandPS5Controller(PRIMARY_CONTROLLER_PORT);

    public final Superstructure superstructure = new Superstructure(primary, swerve);

    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));

        autoChooser.setDefaultOption("/ null Auto", "/ null Auto");

        for (String autoName : AutoBuilder.getAllAutoNames()) {
            autoChooser.addOption(autoName, autoName);
        }

        SmartDashboard.putData("Auto Chooser", autoChooser);


        configureBindings();
        registerCommands();

    }


    private void configureBindings() {

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

        primary.triangle().toggleOnTrue(superstructure.testShotCommand());

        primary.PS().onTrue(new InstantCommand(() -> swerve.resetOdometry(new Pose2d())).ignoringDisable(true));

        primary.circle().onTrue(superstructure.testIntake(-0.9)); //todo moves cHood for some reason

        primary.cross().onTrue(new PrintCommand("" + Units.radiansToDegrees(superstructure.getTurretToHubVectorAngle())).ignoringDisable(true));
    }


    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("Auto #1");
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
        SmartDashboard.putNumber("match time", DriverStation.getMatchTime());
    }

    @Log.NT
    public Pose2d getAuroraPose2d() {
        auroraPose = new Pose2d(
                new Translation2d(
                        NetworkTableInstance.getDefault().getTable("Aurora").getSubTable("robotPose").getEntry("x").getDouble(0),
                        NetworkTableInstance.getDefault().getTable("Aurora").getSubTable("robotPose").getEntry("y").getDouble(0)
                ),
                new Rotation2d(NetworkTableInstance.getDefault().getTable("Aurora").getSubTable("robotPose").getEntry("yaw").getDouble(0)
                )
        );

        return auroraPose;
    };

}
