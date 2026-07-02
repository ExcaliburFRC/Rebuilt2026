// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.*;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import frc.robot.superstructure.RobotState;
import frc.robot.superstructure.Superstructure;
import monologue.Annotations.Log.NT;
import monologue.Logged;

import static frc.robot.Constants.*;
import static frc.robot.Constants.SwerveConstants.MAX_OMEGA_RAD_PER_SEC;
import static frc.robot.Constants.SwerveConstants.MAX_VEL;

public class RobotContainer implements Logged {

    // ===== System Constants =====
    private static final double BATTERY_VOLTAGE_WARNING_THRESHOLD = 12.0; // Volts
    private static final double BATTERY_VOLTAGE_HYSTERESIS = 0.5; // Volts (to prevent alert flickering)

    private final LoggablePS5Controller primary = new LoggablePS5Controller(PRIMARY_CONTROLLER_PORT);

    public final Swerve swerve = Constants.SwerveConstants.configureSwerve(Constants.INITIAL_POSE);
    private final PowerDistribution powerDistributionHub = new PowerDistribution(PDH_PORT, PowerDistribution.ModuleType.kRev);

    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    private final LEDs leds = LEDs.getInstance();

    // ===== Alerts =====
    private final Alert primaryDisconnected = new Alert("Primary controller disconnected (port 0).", Alert.AlertType.kWarning);
    private Trigger shouldDeliverTrigger = new Trigger(() -> false);

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Tab1");
    NetworkTableEntry flywheelVel = table.getEntry("flywheelVel");
    NetworkTableEntry hoodAngle = table.getEntry("hoodAngle");

    // ===== Vision System State =====
    private boolean batteryLow = false;

    //    private final RobotDiagnostics robotDiagnostics = new RobotDiagnostics(powerDistributionHub);
    private final CANHealthMonitor canHealthMonitor = new CANHealthMonitor();
    private final ControllerStateTracker primaryControllerTracker =
            new ControllerStateTracker(primary.getHID(), "Primary Controller");
    private final PerformanceMetricsTracker performanceMetricsTracker =
            new PerformanceMetricsTracker();
//    private final Superstructure superstructure = new Superstructure(swerve, primary.R1(), new Trigger(ShiftUtil::isOwnHubActive), shouldDeliverTrigger);
    private final Superstructure superstructure = new Superstructure(swerve, primary.R2(), primary.square(), primary.circle());
//
    public RobotContainer() {
//        lowBatteryTrigger.onTrue(leds.setPattern(BLINKING, ORANGE.color).withInterruptBehavior(kCancelIncoming));

        flywheelVel.setDouble(0);
        hoodAngle.setDouble(0);

        // P-04 fix: autos reference these named commands; unregistered names silently no-op.
        registerCommands();
        setAutoChooser();
        configureBindings();
    }

    private void configureBindings() {
        // Driver Control
        primary.touchpad().onTrue(superstructure.coastCommand());

//        superstructure.shooter.setDefaultCommand(
//                superstructure.shooter.yoavHatesThisCommandCommand(
//                        () -> hoodAngle.getDouble(0),
//                        () -> flywheelVel.getDouble(0)
//
//                )
//        );

        swerve.setDefaultCommand(
                swerve.driveCommand(
                        () -> new Vector2D(
                                applyDeadband(-primary.getLeftY()) * MAX_VEL,
                                applyDeadband(-primary.getLeftX()) * MAX_VEL),
                        () -> -applyDeadband(primary.getRightX()) * MAX_OMEGA_RAD_PER_SEC,
                        () -> true
                ).unless(() -> DISABLE_SWERVE)
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

//        robotDiagnostics.update();
//        canHealthMonitor.update();
        primaryControllerTracker.update();
        primaryDisconnected.set(!DriverStation.isJoystickConnected(primary.getHID().getPort()));

        performanceMetricsTracker.recordPowerConsumption(powerDistributionHub.getTotalPower());
    }

    public PerformanceMetricsTracker getPerformanceMetricsTracker() {
        return performanceMetricsTracker;
    }

    @NT
    public double getInterpolationFlywheelVel() {
        return flywheelVel.getDouble(0);
    }
}
