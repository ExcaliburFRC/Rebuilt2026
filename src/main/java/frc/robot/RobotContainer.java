// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.excalib.additional_utilities.LoggablePS5Controller;
import frc.excalib.swerve.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.excalib.control.math.Vector2D;

import frc.robot.subsystems.intake.Intake;
import frc.robot.superstructure.Superstructure;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.util.AuroraPoseGetter;
import frc.robot.util.HubTimerSubsystem;
import monologue.Annotations;
import monologue.Logged;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static frc.robot.Constants.CONTROLLER_DEADBAND;
import static frc.robot.Constants.PRIMARY_CONTROLLER_PORT;
import static frc.robot.Constants.SwerveConstants.MAX_OMEGA_RAD_PER_SEC;
import static frc.robot.Constants.SwerveConstants.MAX_VEL;
import static frc.robot.subsystems.shooter.ShooterConstants.HOOD_MAX_ANGLE_LIMIT_IN_TRENCH;
import static monologue.Annotations.*;


public class RobotContainer implements Logged {

    private final LoggablePS5Controller primary = new LoggablePS5Controller(PRIMARY_CONTROLLER_PORT);

    private final Swerve swerve = Constants.SwerveConstants.configureSwerve(Constants.INITIAL_POSE);
    public final Superstructure superstructure = new Superstructure(primary, swerve);

    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    private final HubTimerSubsystem hubTimer = new HubTimerSubsystem();

    public RobotContainer() {
        swerve.resetOdometry(AuroraPoseGetter.getPose2d());

        setAutoChooser();
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

        superstructure.shooter.setDefaultCommand(superstructure.autoHoodAndTurretAim());

        primary.PS().onTrue(
                new InstantCommand(
                        () -> swerve.resetOdometry(
                                new Pose2d(
                                        new Translation2d(3.65, 4.03),
                                        new Rotation2d(0)
                                )
                        )
                ).ignoringDisable(true));
    }


    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("Auto #1");
    }

    public double applyDeadband(double val) {
        return Math.abs(val) < CONTROLLER_DEADBAND ? 0 : val;
    }

    public void registerCommands() {
        NamedCommands.registerCommand("floorIntake", new InstantCommand());
        NamedCommands.registerCommand("prepareShooter", new InstantCommand());
        NamedCommands.registerCommand("shoot", new InstantCommand());
        NamedCommands.registerCommand("extendClimber", new InstantCommand());
        NamedCommands.registerCommand("retractClimber", new InstantCommand());
        NamedCommands.registerCommand("retractIntake", new InstantCommand());
    }

    public void setAutoChooser() {
        autoChooser.setDefaultOption("/ null Auto", "/ null Auto");

        for (String autoName : AutoBuilder.getAllAutoNames()) {
            autoChooser.addOption(autoName, autoName);
        }

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
}
