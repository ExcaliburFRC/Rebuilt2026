// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.superstructure.Superstructure;
import frc.robot.util.ShooterPhysics;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.util.ShootingTarget;
import monologue.Logged;

import static frc.robot.Constants.PRIMARY_CONTROLLER_PORT;
import static frc.robot.Constants.SwerveConstants.MAX_OMEGA_RAD_PER_SEC;
import static frc.robot.Constants.SwerveConstants.MAX_VEL;

public class RobotContainer implements Logged {
    public final ShooterPhysics shooterPhysics;

    public final Swerve swerve = Constants.SwerveConstants.configureSwerve(Constants.INITIAL_POSE);

    public final CommandPS5Controller primary = new CommandPS5Controller(PRIMARY_CONTROLLER_PORT);

    public static ShootingTarget shootingTarget = ShootingTarget.HUB;


    public RobotContainer() {
        shooterPhysics = new ShooterPhysics(
                swerve::getApproximatedFuturePose2D,
                () -> swerve.getRobotRelativeSpeeds().vxMetersPerSecond,
                () -> swerve.getRobotRelativeSpeeds().vyMetersPerSecond
        );


        primary.triangle().toggleOnTrue(swerve.driveToPoseCommand(new Pose2d(new Translation2d(8,4),new Rotation2d())));

        configureBindings();
        registerCommands();
    }


    private void configureBindings() {

        swerve.setDefaultCommand(
                swerve.driveCommand(
                        () -> new Vector2D(
                                applyDeadband(-primary.getLeftY()) * MAX_VEL,
                                applyDeadband(-primary.getLeftX()) * MAX_VEL),
                        () -> applyDeadband(primary.getRightX()) * MAX_OMEGA_RAD_PER_SEC,
                        () -> true
                )
        );

        primary.PS().onTrue(new InstantCommand(() -> swerve.resetOdometry(new Pose2d())).ignoringDisable(true));

    }

    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("Auto #1");
    }

    public double applyDeadband(double val) {
        return Math.abs(val) < 0.09 ? 0 : val;
    }

    public void registerCommands() {
        NamedCommands.registerCommand("floorIntake", new PrintCommand("floorIntake"));
        NamedCommands.registerCommand("prepareShooter", new PrintCommand("prepareShooter"));
        NamedCommands.registerCommand("shoot", new PrintCommand("Shoot"));
        NamedCommands.registerCommand("extendClimber", new PrintCommand("extentClimber"));
        NamedCommands.registerCommand("retractClimber", new PrintCommand("retractClimber"));
        NamedCommands.registerCommand("retractIntake", new PrintCommand("retractIntake"));
        NamedCommands.registerCommand("depotIntake", new PrintCommand("depotIntake"));
    }

}
