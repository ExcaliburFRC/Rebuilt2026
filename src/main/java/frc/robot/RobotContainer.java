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

    // ── Triggers ──────────────────────────────────────────────────────────────
    private final Trigger lowBatteryTrigger = new Trigger(lowBatteryAlert::get);

    private final Trigger endgameTrigger = new Trigger(
            () -> DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= 30
    );

    /** Fires when flywheel is at speed AND turret is on target — the moment it's safe to feed. */
    private final Trigger readyToShootTrigger = new Trigger(superstructure::isReadyToShoot);

    /** Fires when flywheel is spinning but not yet on target (pre-spin feedback). */
    private final Trigger flywheelReadyTrigger = new Trigger(
            () -> superstructure.shooter.getFlyWheelVelocitySetpoint() > 0
                    && Math.abs(superstructure.shooter.getFlyWheelVelocity()
                    - superstructure.shooter.getFlyWheelVelocitySetpoint()) < 1.0
    );

    /**
     * Auto pre-spin: true when robot is within PRE_SPIN_ZONE_METERS of the hub.
     * The scheduled command uses kCancelSelf so L2/L1 override it immediately.
     */
    private final Trigger nearHubTrigger = new Trigger(
            () -> swerve.getPose2D().getTranslation()
                    .getDistance(BLUE_HUB_CENTER_POSE.get().getTranslation())
                    < Superstructure.PRE_SPIN_ZONE_METERS
    );


    public RobotContainer() {
        configureDrive();

        // ── LED feedback ──────────────────────────────────────────────────────
        lowBatteryTrigger.onTrue(leds.setPattern(BLINKING, ORANGE.color).withInterruptBehavior(kCancelIncoming));
        endgameTrigger.onTrue(
                leds.setPattern(BLINKING, RED.color, PURPLE.color)
                        .withTimeout(3)
                        .withInterruptBehavior(kCancelIncoming)
        );
        // Solid green = shooter + turret both on target → safe to release transport
        readyToShootTrigger.onTrue(leds.setPattern(SOLID, GREEN.color));
        readyToShootTrigger.onFalse(leds.setPattern(EXPAND, TEAM_BLUE.color, TEAM_GOLD.color));

        // ── Haptic feedback ───────────────────────────────────────────────────
        flywheelReadyTrigger.onTrue(
                primary.vibrateControllerCommand(0.3, 0.5, GenericHID.RumbleType.kBothRumble)
        );

        // ── Auto pre-spin ─────────────────────────────────────────────────────
        // Spins flywheel up and aims turret automatically when near hub.
        // kCancelSelf means L2/L1 take over instantly.
        nearHubTrigger.whileTrue(superstructure.autoPreSpinCommand());

        setAutoChooser();
        configureBindings();
        registerCommands();
    }

    /**
     * Sets up the field-relative swerve drive default command.
     * Speed is scaled to 45 % while any shoot/track command is active to
     * prevent drivetrain skid from disturbing aiming.
     */
    private void configureDrive() {
        swerve.setDefaultCommand(
                swerve.driveCommand(
                        () -> new Vector2D(
                                -applyDeadband(primary.getLeftY()) * MAX_VEL * superstructure.getDriveSpeedScale(),
                                -applyDeadband(primary.getLeftX()) * MAX_VEL * superstructure.getDriveSpeedScale()
                        ),
                        () -> -applyDeadband(primary.getRightX()) * MAX_OMEGA_RAD_PER_SEC,
                        () -> true
                )
        );
    }

    private void configureBindings() {
        // ── Shooter ───────────────────────────────────────────────────────────
        // L2: Full hub shoot — turret tracks, flywheel spins, transport gates on ready
        primary.L2().whileTrue(superstructure.shootToHubCommand());
        // L1: Pre-spin + aim without feeding (prepare while driving to shot position)
        primary.L1().whileTrue(superstructure.trackHubCommand());
        // D-pad down: Shoot to nearest delivery station
        primary.povDown().whileTrue(superstructure.shootToDeliveryCommand());

        // ── Intake ────────────────────────────────────────────────────────────
        // R2 hold: Deploy intake arm + run rollers
        primary.R2().whileTrue(
                new ParallelCommandGroup(
                        superstructure.intake.setAnglePosition(Intake.IntakeState.OPEN),
                        superstructure.intakeRollerActivationCommand(6.0)
                )
        );
        // R2 release: Retract intake arm
        primary.R2().onFalse(superstructure.intake.setAnglePosition(Intake.IntakeState.CLOSE));

        // ── Unjam ─────────────────────────────────────────────────────────────
        // Triangle: Reverse both transport motors briefly to clear fuel jams
        primary.triangle().onTrue(superstructure.unjamCommand());

        // ── Heading auto-aim ──────────────────────────────────────────────────
        // R1 hold: Robot auto-rotates to face hub while driver controls translation
        primary.R1().whileTrue(
                swerve.turnToAngleCommand(
                        () -> new Vector2D(
                                -applyDeadband(primary.getLeftY()) * MAX_VEL * superstructure.getDriveSpeedScale(),
                                -applyDeadband(primary.getLeftX()) * MAX_VEL * superstructure.getDriveSpeedScale()
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

        // ── Utility ───────────────────────────────────────────────────────────
        // Options: Reset gyro heading to field-forward
        primary.options().onTrue(swerve.resetAngleCommand());
        // PS: Coast drivetrain (end-of-match or pushed)
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
