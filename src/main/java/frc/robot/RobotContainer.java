// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import monologue.Logged;

public class RobotContainer implements Logged {
    CommandPS5Controller driver = new CommandPS5Controller(0);

    Swerve swerve = Constants.SwerveConstants.configureSwerve(new Pose2d());

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        driver.cross().onTrue(swerve.pathfindThenFollowPathCommand("Example Path"));

         swerve.setDefaultCommand(
                swerve.driveCommand(
                        () -> new Vector2D(
                                applyDeadband(-driver.getLeftY()) * Constants.SwerveConstants.MAX_VEL,
                                applyDeadband(-driver.getLeftX()) * Constants.SwerveConstants.MAX_VEL),
                        () -> applyDeadband(-driver.getRightX()) * Constants.SwerveConstants.MAX_OMEGA_RAD_PER_SEC,
                        () -> true
                )
        );

    }

    public double applyDeadband(double val) {
        return Math.abs(val) < Constants.DEADBAND_X ? 0 : val;
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
