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
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.excalib2.util.LedStrip;
import frc.robot.superstructure.RobotState;
import frc.robot.superstructure.Superstructure;

import static frc.robot.Constants.*;

public class RobotContainer {

    private static final int LED_PWM_PORT = 8;   // from ExcaLib v1 LEDs
    private static final int LED_STRIP_LENGTH = 22;

    private final CommandPS5Controller primary = new CommandPS5Controller(PRIMARY_CONTROLLER_PORT);

    public final frc.excalib2.swerve.SwerveSubsystem swerve = new frc.excalib2.swerve.SwerveSubsystem(
            SwerveConfig.DRIVETRAIN,
            SwerveConfig.FRONT_LEFT, SwerveConfig.FRONT_RIGHT,
            SwerveConfig.BACK_LEFT, SwerveConfig.BACK_RIGHT);

    private final PowerDistribution powerDistributionHub = new PowerDistribution(PDH_PORT, PowerDistribution.ModuleType.kRev);

    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    private final LedStrip leds = new LedStrip(LED_PWM_PORT, LED_STRIP_LENGTH);

    // ===== Alerts =====
    private final Alert primaryDisconnected = new Alert("Primary controller disconnected (port 0).", Alert.AlertType.kWarning);
    private Trigger shouldDeliverTrigger = new Trigger(() -> false);

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Tab1");
    NetworkTableEntry flywheelVel = table.getEntry("flywheelVel");
    NetworkTableEntry hoodAngle = table.getEntry("hoodAngle");

//    private final Superstructure superstructure = new Superstructure(swerve, primary.R1(), new Trigger(ShiftUtil::isOwnHubActive), shouldDeliverTrigger);
    private final Superstructure superstructure = new Superstructure(
            () -> swerve.getState().Pose,
            () -> swerve.getState().Speeds,
            primary.R2(), primary.square(), primary.circle());
//
    public RobotContainer() {
        dev.doglog.DogLog.setPdh(powerDistributionHub); // automatic PDH channel/voltage logging
        leds.setDefaultPattern(LEDPattern.solid(Color.kBlue));

        flywheelVel.setDouble(0);
        hoodAngle.setDouble(0);

        // Turret-mounted limelight: MT2 orientation = turret platform heading; the returned
        // pose is turret-centered and converted back to a robot pose (v1 turretToRobot math).
        swerve.withLimelight(
                        "limelight-turret",
                        () -> swerve.getHeading().plus(frc.robot.util.TurretOffsetGetter.instance.getTurretOffset()),
                        frc.robot.util.TurretOffsetGetter.instance::isFast)
                .withVisionPoseTransform(SwerveConfig::turretPoseToRobotPose);
        swerve.configureAutoBuilder(SwerveConfig.PP_TRANSLATION_GAINS, SwerveConfig.PP_ROTATION_GAINS);

        // P-04 fix: autos reference these named commands; unregistered names silently no-op.
        registerCommands();
        setAutoChooser();
        configureBindings();
        configureTestBindings();   // bring-up harness — active only in DriverStation Test mode
    }

    /**
     * On-robot bring-up controls, <b>gated to Test mode</b> ({@code RobotModeTriggers.test()}),
     * so none of this can fire during teleop or autonomous. Enter "Test" + "Enable" in the
     * Driver Station to use them. See {@code docs/BRINGUP_TESTPLAN.html}.
     *
     * <p>Layout on the primary controller (Test mode only):
     * <ul>
     *   <li><b>L1</b> — coast the whole robot (swerve + intake four-bar + turret) for
     *       hand-positioning / zeroing.</li>
     *   <li><b>D-pad</b> — SysId on the <i>mechanism under test</i>: ▲ quasistatic-fwd,
     *       ▼ quasistatic-rev, ▶ dynamic-fwd, ◀ dynamic-rev. Change {@code MUT} below to
     *       characterize a different mechanism.</li>
     *   <li><b>△ / ✕</b> — quick intake OPEN / CLOSE direction &amp; range check.</li>
     *   <li><b>◯ (hold)</b> — spin the flywheel at a low fixed speed (10 rot/s) to check
     *       direction and readiness.</li>
     * </ul>
     */
    private void configureTestBindings() {
        Trigger test = edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.test();

        // --- coast everything for hand-positioning ---
        test.and(primary.L1()).whileTrue(
                swerve.coastCommand()
                        .alongWith(superstructure.intake.coastCommand())
                        .alongWith(superstructure.coastCommand()));   // coasts the turret

        // --- SysId of the MECHANISM UNDER TEST (edit these two lines to pick a mechanism) ---
        var mut = superstructure.shooter.hood;      // the mechanism to characterize
        var mutSubsystem = superstructure.shooter;  // the subsystem that owns it (SysId needs the requirement)
        test.and(primary.povUp())
                .whileTrue(mut.sysIdQuasistatic(mutSubsystem, Direction.kForward));
        test.and(primary.povDown())
                .whileTrue(mut.sysIdQuasistatic(mutSubsystem, Direction.kReverse));
        test.and(primary.povRight())
                .whileTrue(mut.sysIdDynamic(mutSubsystem, Direction.kForward));
        test.and(primary.povLeft())
                .whileTrue(mut.sysIdDynamic(mutSubsystem, Direction.kReverse));
        // torque-current (FOC) SysId: swap the four lines above for sysIdQuasistaticTorque / sysIdDynamicTorque.

        // --- quick per-subsystem checks ---
        test.and(primary.triangle()).whileTrue(superstructure.intake.setStateCommand(frc.robot.subsystems.intake.IntakeStates.OPEN));
        test.and(primary.cross()).whileTrue(superstructure.intake.setStateCommand(frc.robot.subsystems.intake.IntakeStates.CLOSE));
        test.and(primary.circle()).whileTrue(
                superstructure.shooter.run(() -> superstructure.shooter.flywheel.setVelocityRotationsPerSecond(10)));
    }

    private void configureBindings() {
        // Driver Control
        primary.touchpad().onTrue(superstructure.coastCommand());

        // v1 LED behavior: blink orange while the flywheel spins
        superstructure.shooter.flywheelSpinning.whileTrue(
                leds.runPatternCommand(LEDPattern.solid(Color.kOrange).blink(Units.Seconds.of(0.2))));

//        superstructure.shooter.setDefaultCommand(
//                superstructure.shooter.yoavHatesThisCommandCommand(
//                        () -> hoodAngle.getDouble(0),
//                        () -> flywheelVel.getDouble(0)
//
//                )
//        );

        swerve.setDefaultCommand(
                swerve.fieldCentricDriveCommand(
                        () -> -primary.getLeftY(),
                        () -> -primary.getLeftX(),
                        () -> -primary.getRightX(),
                        SwerveConfig.MAX_SPEED_MPS,
                        SwerveConfig.MAX_OMEGA_RAD_PER_SEC,
                        CONTROLLER_DEADBAND
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
        primaryDisconnected.set(!DriverStation.isJoystickConnected(primary.getHID().getPort()));
    }
}
