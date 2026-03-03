// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.excalib.additional_utilities.LoggablePS5Controller;
import frc.excalib.swerve.Swerve;
import frc.excalib.control.math.Vector2D;

import frc.robot.util.AuroraPoseGetter;
import frc.robot.util.HubTimerSubsystem;
import monologue.Logged;

import static frc.robot.Constants.CONTROLLER_DEADBAND;
import static frc.robot.Constants.PRIMARY_CONTROLLER_PORT;
import static frc.robot.Constants.SwerveConstants.MAX_OMEGA_RAD_PER_SEC;
import static frc.robot.Constants.SwerveConstants.MAX_VEL;


public class RobotContainer implements Logged {

    private final LoggablePS5Controller primary = new LoggablePS5Controller(PRIMARY_CONTROLLER_PORT);

    private final Swerve swerve = Constants.SwerveConstants.configureSwerve(Constants.INITIAL_POSE);
//    public final Superstructure superstructure = new Superstructure(primary, swerve);

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
                        () -> applyDeadband(primary.getRightX()) * MAX_OMEGA_RAD_PER_SEC,
                        () -> true
                )
        );

//        superstructure.shooter.setDefaultCommand(superstructure.autoHoodAndTurretAim());

        primary.options().onTrue(new RunCommand(() -> swerve.resetOdometry(new Pose2d())));

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

    public void perodic() {
        if (!AuroraPoseGetter.getPose2d().equals(new Pose2d())) {
            swerve.m_odometry.addVisionMeasurement(AuroraPoseGetter.getPose2d(), Timer.getFPGATimestamp());
        }
    }

}
