// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.excalib.additional_utilities.*;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStates;
import frc.robot.superstructure.RobotState;
import frc.robot.superstructure.Superstructure;
import frc.robot.util.HubTimerSubsystem;
import frc.robot.util.TelemetryManager;
import monologue.Logged;

import static frc.robot.Constants.*;
import static frc.robot.Constants.SwerveConstants.MAX_OMEGA_RAD_PER_SEC;
import static frc.robot.Constants.SwerveConstants.MAX_VEL;

public class RobotContainer implements Logged {

    private static final double CONTROLLER_DEADBAND = 0.09;

    private final CommandXboxController primary;
    private final boolean isSim;

    @monologue.Annotations.Log.NT
    private final Swerve swerve = frc.robot.util.SwerveFactory.createSwerve(Constants.INITIAL_POSE);

    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    // HubTimer removed as part of architecture cleanup

    private final LEDs leds = LEDs.getInstance();
    @monologue.Annotations.Log.NT
    private final Superstructure superstructure = new Superstructure(swerve);

    private final TelemetryManager telemetryManager;

    // ===== Alerts =====
    private final Alert autoNotChosen = new Alert("!!! AUTO NOT SET !!!", Alert.AlertType.kError);

    private final ControllerStateTracker primaryControllerTracker;


    public RobotContainer(TelemetryManager telemetryManager) {
        this.telemetryManager = telemetryManager;
        this.isSim = edu.wpi.first.wpilibj.RobotBase.isSimulation();
        primary = new CommandXboxController(PRIMARY_CONTROLLER_PORT);
        primaryControllerTracker = new ControllerStateTracker(primary.getHID(), "Primary Controller");
        if (this.isSim) {
            // Simulator fallbacks (if any) go here
        }
        
        new edu.wpi.first.wpilibj2.command.button.Trigger(telemetryManager::isBatteryLow)
            .onTrue(leds.setPattern(frc.excalib.additional_utilities.LEDs.LEDPattern.BLINKING, frc.excalib.additional_utilities.Color.Colors.ORANGE.color)
            .withInterruptBehavior(edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.kCancelIncoming));

        flywheelVel.setDouble(0);
        hoodAngle.setDouble(0);

        //registerCommands();
        setAutoChooser();
        configureBindings();
    }

    private void configureBindings() {
        // Driver Controls
        primary.x().toggleOnTrue(superstructure.trackHubCommand().alongWith(new PrintCommand("X Button Pressed (Track)")));
        primary.y().toggleOnTrue(superstructure.shootToHubCommand().alongWith(new PrintCommand("Y Button Pressed (Shoot)")));
    
        // Use both keyboard AND controller securely in simulation, or just controller on robot
        swerve.setDefaultCommand(
            swerve.driveCommand(
                () -> {
                    double fwd = -applyDeadband(primary.getLeftY());
                    double str = -applyDeadband(primary.getLeftX());
                    return new Vector2D(fwd * MAX_VEL, str * MAX_VEL);
                },
                () -> {
                    double rot = -applyDeadband(primary.getRightX());
                    return rot * MAX_OMEGA_RAD_PER_SEC;
                },
                () -> true
            )
        );
    }

    public Command getAutonomousCommand() {
        String selected = autoChooser.getSelected();
        if (selected == null || "/ null Auto".equals(selected)) {
            return Commands.none();
        }
        return AutoBuilder.buildAuto(selected);
    }

    public double applyDeadband(double val) {
        return Math.abs(val) < CONTROLLER_DEADBAND ? 0 : val;
    }

    public void registerCommands() {
        NamedCommands.registerCommand("idle", superstructure.setStateCommand(RobotState.IDLE).alongWith(new PrintCommand("idle")));
        NamedCommands.registerCommand("shoot", superstructure.setStateCommand(RobotState.NO_INTAKE_SHOOT_HUB).alongWith(new PrintCommand("shoot")));
        NamedCommands.registerCommand("intake", superstructure.setStateCommand(RobotState.INTAKE_AIM_HUB).alongWith(new PrintCommand("intake")));
    }



    public void setAutoChooser() {
        autoChooser.setDefaultOption("/ null Auto", "/ null Auto");

        for (String autoName : AutoBuilder.getAllAutoNames()) {
            autoChooser.addOption(autoName, autoName);
        }
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void periodic() {
        superstructure.periodic();
        Pose2d visionPose = AuroraPoseGetter.getPose2d();
        if (!visionPose.equals(new Pose2d())) {
            swerve.m_odometry.addVisionMeasurement(AuroraPoseGetter.getPose2d(), edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
        }

        primaryControllerTracker.update();

        autoNotChosen.set(autoChooser.getSelected() == null ||
                autoChooser.getSelected().equals("/ null Auto"));
            }

    @NT
    public double getInterpolationFlywheelVel() {
        return flywheelVel.getDouble(0);
    }
}
