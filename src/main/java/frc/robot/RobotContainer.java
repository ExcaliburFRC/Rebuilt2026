// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.Color;
import frc.excalib.additional_utilities.LEDs;
import frc.excalib.additional_utilities.LoggablePS5Controller;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.superstructure.Superstructure;
import frc.robot.util.AuroraPoseGetter;
import frc.robot.util.HubTimerSubsystem;
import monologue.Logged;

import static edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.*;
import static frc.excalib.additional_utilities.Color.Colors.*;
import static frc.excalib.additional_utilities.LEDs.LEDPattern.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.SwerveConstants.*;


public class RobotContainer implements Logged {

    private final LoggablePS5Controller primary = new LoggablePS5Controller(PRIMARY_CONTROLLER_PORT);

    private final Swerve swerve = Constants.SwerveConstants.configureSwerve(Constants.INITIAL_POSE);
    private final PowerDistribution PowerDistributionHub = new PowerDistribution(PDH_PORT, PowerDistribution.ModuleType.kRev);
    public final Superstructure superstructure = new Superstructure(primary, swerve);

    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    private final HubTimerSubsystem hubTimer = new HubTimerSubsystem();

    private final LEDs leds = LEDs.getInstance();

    private final Alert primaryDisconnected = new Alert("Primary controller disconnected (port 0).", Alert.AlertType.kWarning);
    private final Alert autoNotChosen = new Alert("!!! AUTO NOT SET !!!", Alert.AlertType.kError);
    private final Alert lowBatteryAlert = new Alert("Battery voltage is low", Alert.AlertType.kWarning);
    private final Alert endgameAlert = new Alert("Endgame approaching! < 30 seconds left.", Alert.AlertType.kInfo);

    private final Trigger lowBatteryTrigger = new Trigger(lowBatteryAlert::get);
    private final Trigger endgameTrigger = new Trigger(
            () -> DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= 30
    );
    private final Trigger flywheelReadyTrigger = new Trigger(
            () -> superstructure.shooter.getFlyWheelVelocitySetpoint() > 0
                    && Math.abs(superstructure.shooter.getFlyWheelVelocity()
                    - superstructure.shooter.getFlyWheelVelocitySetpoint()) < 1.0
    );


    public RobotContainer() {
        configureDrive();

        lowBatteryTrigger.onTrue(leds.setPattern(BLINKING, ORANGE.color).withInterruptBehavior(kCancelIncoming));
        endgameTrigger.onTrue(
                leds.setPattern(BLINKING, RED.color, PURPLE.color)
                        .withTimeout(3)
                        .withInterruptBehavior(kCancelIncoming)
        );
        flywheelReadyTrigger.onTrue(
                primary.vibrateControllerCommand(0.3, 0.5, GenericHID.RumbleType.kBothRumble)
        );

        setAutoChooser();
        configureBindings();
        registerCommands();
    }

    private void configureDrive() {
        swerve.setDefaultCommand(
                swerve.driveCommand(
                        () -> new Vector2D(
                                -applyDeadband(primary.getLeftY()) * MAX_VEL,
                                -applyDeadband(primary.getLeftX()) * MAX_VEL
                        ),
                        () -> -applyDeadband(primary.getRightX()) * MAX_OMEGA_RAD_PER_SEC,
                        () -> true
                )
        );
    }

    private void configureBindings() {
        // === Shooter bindings ===
        // L2: Shoot to hub (aims turret, spins flywheel, feeds transport)
        primary.L2().whileTrue(superstructure.shootToHubCommand());
        // L1: Pre-track hub without feeding (spin up flywheel, aim turret)
        primary.L1().whileTrue(superstructure.trackHubCommand());

        // === Intake bindings ===
        // R2 hold: Deploy intake arm and run rollers
        primary.R2().whileTrue(
                new ParallelCommandGroup(
                        superstructure.intake.setAnglePosition(Intake.IntakeState.OPEN),
                        superstructure.intakeRollerActivationCommand(6.0)
                )
        );
        // R2 release: Retract intake arm
        primary.R2().onFalse(superstructure.intake.setAnglePosition(Intake.IntakeState.CLOSE));

        // === Robot heading auto-aim toward hub while driving ===
        // R1 hold: Auto-rotate robot to face hub while driver controls translation
        primary.R1().whileTrue(
                swerve.turnToAngleCommand(
                        () -> new Vector2D(
                                -applyDeadband(primary.getLeftY()) * MAX_VEL,
                                -applyDeadband(primary.getLeftX()) * MAX_VEL
                        ),
                        () -> {
                            Translation2d hub = BLUE_HUB_CENTER_POSE.get().getTranslation();
                            Translation2d robot = swerve.getPose2D().getTranslation();
                            return Rotation2d.fromRadians(
                                    Math.atan2(hub.getY() - robot.getY(), hub.getX() - robot.getX())
                            );
                        }
                )
        );

        // === Utility bindings ===
        // Options: Reset gyro heading to field-forward
        primary.options().onTrue(swerve.resetAngleCommand());
        // PS: Coast drivetrain (for being pushed or end-of-match)
        primary.PS().whileTrue(swerve.coastCommand());
    }


    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto(autoChooser.getSelected());
    }

    public double applyDeadband(double val) {
        return Math.abs(val) < CONTROLLER_DEADBAND ? 0 : val;
    }

    public void registerCommands() {
        NamedCommands.registerCommand("floorIntake", new ParallelCommandGroup(
                superstructure.intake.setAnglePosition(Intake.IntakeState.OPEN),
                superstructure.intakeRollerActivationCommand(6.0)
        ));
        NamedCommands.registerCommand("prepareShooter", superstructure.trackHubCommand());
        NamedCommands.registerCommand("shoot", superstructure.shootToHubCommand());
        NamedCommands.registerCommand("retractIntake", superstructure.intake.setAnglePosition(Intake.IntakeState.CLOSE));
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
        endgameAlert.set(endgameTrigger.getAsBoolean());
    }

}
