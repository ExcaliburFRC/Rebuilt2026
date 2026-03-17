// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.*;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;

import frc.robot.superstructure.Superstructure;
import frc.robot.util.AuroraPoseGetter;
import frc.robot.util.HubTimerSubsystem;
import monologue.Logged;

import static frc.robot.Constants.*;
import static frc.robot.Constants.SwerveConstants.MAX_OMEGA_RAD_PER_SEC;
import static frc.robot.Constants.SwerveConstants.MAX_VEL;

public class RobotContainer implements Logged {

    // ===== System Constants =====
    private static final double BATTERY_VOLTAGE_WARNING_THRESHOLD = 12.0; // Volts
    private static final double BATTERY_VOLTAGE_HYSTERESIS = 0.5; // Volts (to prevent alert flickering)

    private final LoggablePS5Controller primary = new LoggablePS5Controller(PRIMARY_CONTROLLER_PORT);

    private final Swerve swerve = Constants.SwerveConstants.configureSwerve(Constants.INITIAL_POSE);
    private final PowerDistribution powerDistributionHub = new PowerDistribution(PDH_PORT, PowerDistribution.ModuleType.kRev);

    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    private final HubTimerSubsystem hubTimer = new HubTimerSubsystem();

    private final LEDs leds = LEDs.getInstance();
    private final Superstructure superstructure = new Superstructure(swerve);

    // ===== Alerts =====
    private final Alert primaryDisconnected = new Alert("Primary controller disconnected (port 0).", Alert.AlertType.kWarning);
    private final Alert autoNotChosen = new Alert("!!! AUTO NOT SET !!!", Alert.AlertType.kError);
    private final Alert lowBatteryAlert = new Alert("Battery voltage is low", Alert.AlertType.kWarning);
    private final Trigger lowBatteryTrigger = new Trigger(lowBatteryAlert::get);

    // ===== Vision System State =====
    private boolean batteryLow = false;

    //    private final RobotDiagnostics robotDiagnostics = new RobotDiagnostics(powerDistributionHub);
    private final CANHealthMonitor canHealthMonitor = new CANHealthMonitor();
    private final ControllerStateTracker primaryControllerTracker =
            new ControllerStateTracker(primary.getHID(), "Primary Controller");
    private final PerformanceMetricsTracker performanceMetricsTracker =
            new PerformanceMetricsTracker();


    public RobotContainer() {
//        lowBatteryTrigger.onTrue(leds.setPattern(BLINKING, ORANGE.color).withInterruptBehavior(kCancelIncoming));

        setAutoChooser();
        configureBindings();
        registerCommands();
    }

    private void configureBindings() {
        // Driver Controls
        primary.square().toggleOnTrue(superstructure.trackHubCommand());
        primary.triangle().toggleOnTrue(superstructure.shootToHubCommand());
        primary.povUp().whileTrue(superstructure.shootFixedCommand(44, 0.4));

        // Intake Controls
        primary.L1().whileTrue(superstructure.intakeCommand());
        primary.R1().whileTrue(superstructure.ejectCommand());

        swerve.setDefaultCommand(
                swerve.driveCommand(
                        () -> new Vector2D(
                                applyDeadband(-primary.getLeftY()) * MAX_VEL,
                                applyDeadband(-primary.getLeftX()) * MAX_VEL),
                        () -> -applyDeadband(primary.getRightX()) * MAX_OMEGA_RAD_PER_SEC,
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
        NamedCommands.registerCommand("floorIntake", superstructure.intakeCommand());
        NamedCommands.registerCommand("prepareShooter", superstructure.trackHubCommand());
        NamedCommands.registerCommand("shoot", superstructure.shootToHubCommand());
        NamedCommands.registerCommand("retractIntake", superstructure.stopIntakeCommand());
    }

    public void setAutoChooser() {
        autoChooser.setDefaultOption("/ null Auto", "/ null Auto");

        for (String autoName : AutoBuilder.getAllAutoNames()) {
            autoChooser.addOption(autoName, autoName);
        }
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void periodic() {
        Pose2d visionPose = AuroraPoseGetter.getPose2d();
        if (!visionPose.equals(new Pose2d())) {
            swerve.m_odometry.addVisionMeasurement(AuroraPoseGetter.getPose2d(), Timer.getFPGATimestamp());
        }

//        robotDiagnostics.update();
//        canHealthMonitor.update();
        primaryControllerTracker.update();
        primaryDisconnected.set(!DriverStation.isJoystickConnected(primary.getHID().getPort()));

        autoNotChosen.set(autoChooser.getSelected() == null ||
                autoChooser.getSelected().equals("/ null Auto"));

        double voltage = powerDistributionHub.getVoltage();
        if (voltage < BATTERY_VOLTAGE_WARNING_THRESHOLD - BATTERY_VOLTAGE_HYSTERESIS) {
            batteryLow = true;
        } else if (voltage > BATTERY_VOLTAGE_WARNING_THRESHOLD + BATTERY_VOLTAGE_HYSTERESIS) {
            batteryLow = false;
        }
        lowBatteryAlert.set(batteryLow);

        performanceMetricsTracker.recordPowerConsumption(powerDistributionHub.getTotalPower());
    }

    public PerformanceMetricsTracker getPerformanceMetricsTracker() {
        return performanceMetricsTracker;
    }

}
