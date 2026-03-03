// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.*;
import frc.excalib.swerve.Swerve;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AuroraPoseGetter;
import frc.robot.util.HubTimerSubsystem;
import frc.excalib.slam.mapper.VisionMeasurementValidator;
import monologue.Logged;

import static edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.*;
import static frc.excalib.additional_utilities.Color.Colors.*;
import static frc.excalib.additional_utilities.LEDs.LEDPattern.*;
import static frc.robot.Constants.*;

public class RobotContainer implements Logged {

    // ===== System Constants =====
    private static final double BATTERY_VOLTAGE_WARNING_THRESHOLD = 12.0; // Volts
    private static final double BATTERY_VOLTAGE_HYSTERESIS = 0.5; // Volts (to prevent alert flickering)

    private final LoggablePS5Controller primary = new LoggablePS5Controller(PRIMARY_CONTROLLER_PORT);

    private final Swerve swerve = Constants.SwerveConstants.configureSwerve(Constants.INITIAL_POSE);
    private final PowerDistribution PowerDistributionHub = new PowerDistribution(PDH_PORT, PowerDistribution.ModuleType.kRev);

    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    private final HubTimerSubsystem hubTimer = new HubTimerSubsystem();

    // Initialize Shooter with actual swerve pose supplier instead of hardcoded Pose2d::new
    private final Shooter shooter = new Shooter(() -> 0, swerve::getPose2D);

    private final LEDs leds = LEDs.getInstance();

    // ===== Alerts =====
    private final Alert primaryDisconnected = new Alert("Primary controller disconnected (port 0).", Alert.AlertType.kWarning);
    private final Alert autoNotChosen = new Alert("!!! AUTO NOT SET !!!", Alert.AlertType.kError);
    private final Alert lowBatteryAlert = new Alert("Battery voltage is low", Alert.AlertType.kWarning);
    private final Trigger lowBatteryTrigger = new Trigger(lowBatteryAlert::get);

    // ===== Vision System State =====
    private Pose2d lastValidVisionPose = null;
    private boolean batteryLow = false;

    // ===== Diagnostic & Monitoring Systems =====
    private final RobotDiagnostics robotDiagnostics = new RobotDiagnostics(PowerDistributionHub);
    private final CANHealthMonitor canHealthMonitor = new CANHealthMonitor();
    private final ControllerStateTracker primaryControllerTracker =
        new ControllerStateTracker(primary.getHID(), "Primary Controller");
    private final PerformanceMetricsTracker performanceMetricsTracker =
            new PerformanceMetricsTracker();


    public RobotContainer() {
//        swerve.resetOdometry(AuroraPoseGetter.getPose2d());

        lowBatteryTrigger.onTrue(leds.setPattern(BLINKING, ORANGE.color).withInterruptBehavior(kCancelIncoming));

        setAutoChooser();
        configureBindings();
        registerCommands();
    }

    private void configureBindings() {
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
        // ===== Vision System Updates =====
        Pose2d visionPose = AuroraPoseGetter.getPose2d();
        if (VisionMeasurementValidator.isValidVisionMeasurement(visionPose, lastValidVisionPose)) {
            swerve.m_odometry.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
            lastValidVisionPose = visionPose;
        }

        // ===== System Health Monitoring =====
        // Update diagnostic metrics for power analysis
        robotDiagnostics.update();

        // Monitor CAN bus for communication errors
        canHealthMonitor.update();

        // Track controller connection state
        primaryControllerTracker.update();

        // ===== Alert Management =====
        // Monitor controller connection status
        primaryDisconnected.set(!DriverStation.isJoystickConnected(primary.getHID().getPort()));

        // Warn if no auto is selected
        autoNotChosen.set(autoChooser.getSelected() == null ||
                         autoChooser.getSelected().equals("/ null Auto"));

        // Monitor battery voltage with hysteresis to prevent alert flickering
        double voltage = PowerDistributionHub.getVoltage();
        if (voltage < BATTERY_VOLTAGE_WARNING_THRESHOLD - BATTERY_VOLTAGE_HYSTERESIS) {
            batteryLow = true;
        } else if (voltage > BATTERY_VOLTAGE_WARNING_THRESHOLD + BATTERY_VOLTAGE_HYSTERESIS) {
            batteryLow = false;
        }
        lowBatteryAlert.set(batteryLow);

        // ===== Performance Tracking =====
        // Record power consumption for performance analysis
        performanceMetricsTracker.recordPowerConsumption(PowerDistributionHub.getTotalPower());
    }

    /**
     * Gets the performance metrics tracker for monitoring robot performance
     */
    public PerformanceMetricsTracker getPerformanceMetricsTracker() {
        return performanceMetricsTracker;
    }

}
