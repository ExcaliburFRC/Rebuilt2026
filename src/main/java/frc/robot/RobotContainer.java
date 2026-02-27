// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.Color;
import frc.excalib.additional_utilities.LEDs;
import frc.excalib.additional_utilities.LoggablePS5Controller;
import frc.excalib.swerve.Swerve;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AuroraPoseGetter;
import frc.robot.util.HubTimerSubsystem;
import monologue.Logged;

import static edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.*;
import static frc.excalib.additional_utilities.Color.Colors.*;
import static frc.excalib.additional_utilities.LEDs.LEDPattern.*;
import static frc.robot.Constants.*;


public class RobotContainer implements Logged {

    private final LoggablePS5Controller primary = new LoggablePS5Controller(PRIMARY_CONTROLLER_PORT);

    private final Swerve swerve = Constants.SwerveConstants.configureSwerve(Constants.INITIAL_POSE);
    private final PowerDistribution PowerDistributionHub = new PowerDistribution(PDH_PORT, PowerDistribution.ModuleType.kRev);
//    public final Superstrcture superstructure = new Superstructure(primary, swerve);

    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    private final HubTimerSubsystem hubTimer = new HubTimerSubsystem();
    private final Shooter shooter = new Shooter(() -> 0, Pose2d::new);

    private final LEDs leds = LEDs.getInstance();

    private final Alert primaryDisconnected = new Alert("Primary controller disconnected (port 0).", Alert.AlertType.kWarning);
    private final Alert autoNotChosen = new Alert("!!! AUTO NOT SET !!!", Alert.AlertType.kError);

    private final Alert lowBatteryAlert = new Alert("Battery voltage is low", Alert.AlertType.kWarning);
    private final Trigger lowBatteryTrigger = new Trigger(lowBatteryAlert::get);


    public RobotContainer() {
//        swerve.resetOdometry(AuroraPoseGetter.getPose2d());

        lowBatteryTrigger.onTrue(leds.setPattern(BLINKING, ORANGE.color).withInterruptBehavior(kCancelIncoming));

        setAutoChooser();
        configureBindings();
        registerCommands();
    }

    private void configureBindings() {

        primary.cross().onTrue(shooter.setHoodAngleCommand(() -> 0.5));
        primary.triangle().onTrue(shooter.setHoodAngleCommand(() -> 0.3));
        primary.square().onTrue(shooter.setHoodAngleCommand(() -> 0.8));
        primary.circle().onTrue(shooter.setHoodAngleCommand(() -> 0));

    }


    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto(autoChooser.getSelected());
    }

    public double applyDeadband(double val) {
        return Math.abs(val) < CONTROLLER_DEADBAND ? 0 : val;
    }

    public void registerCommands() {
        NamedCommands.registerCommand("floorIntake", new InstantCommand());
        NamedCommands.registerCommand("prepareShooter", new InstantCommand());
        NamedCommands.registerCommand("shoot", new InstantCommand());
        NamedCommands.registerCommand("retractIntake", new InstantCommand());
    }

    public void setAutoChooser() {
        autoChooser.setDefaultOption("/ null Auto", "/ null Auto");

        for (String autoName : AutoBuilder.getAllAutoNames()) {
            autoChooser.addOption(autoName, autoName);
        }
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void periodic() {
        if (!AuroraPoseGetter.getPose2d().equals(new Pose2d())) {
            swerve.m_odometry.addVisionMeasurement(AuroraPoseGetter.getPose2d(), Timer.getFPGATimestamp());
        }

        primaryDisconnected.set(!DriverStation.isJoystickConnected(primary.getHID().getPort()));
        autoNotChosen.set(autoChooser.getSelected().equals("/ null Auto"));
        lowBatteryAlert.set(PowerDistributionHub.getVoltage() < 12.0);
    }

}
