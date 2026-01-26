// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.excalib.swerve.Swerve;
import frc.robot.superstructure.Superstructure;
import frc.robot.util.ShooterPhysics;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import monologue.Logged;

import static frc.robot.Constants.PRIMARY_CONTROLLER_PORT;


public class RobotContainer implements Logged {
    public final ShooterPhysics shooterPhysics;

    public final Swerve swerve = Constants.SwerveConstants.configureSwerve(Constants.INITIAL_POSE);

    public final Superstructure superstructure;

    public final CommandPS5Controller primary = new CommandPS5Controller(PRIMARY_CONTROLLER_PORT);

    public RobotContainer() {
        shooterPhysics = new ShooterPhysics(
                swerve::getApproximatedFuturePose2D,
                () -> swerve.getRobotRelativeSpeeds().vxMetersPerSecond,
                () -> swerve.getRobotRelativeSpeeds().vyMetersPerSecond
        );

        superstructure = new Superstructure(primary, swerve);

        configureBindings();
        registerCommands();
    }


    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void registerCommands() {
        NamedCommands.registerCommand("floorIntake", new InstantCommand());
        NamedCommands.registerCommand("prepareShooter", new InstantCommand());
        NamedCommands.registerCommand("shoot", new InstantCommand());
        NamedCommands.registerCommand("extendClimber", new InstantCommand());
        NamedCommands.registerCommand("retractClimber", new InstantCommand());
        NamedCommands.registerCommand("retractIntake", new InstantCommand());
    }
}
