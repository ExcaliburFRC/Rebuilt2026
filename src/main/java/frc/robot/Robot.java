// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.excalib2.device.SignalHub;
import frc.excalib2.telemetry.FaultReporter;
import frc.excalib2.telemetry.LoopTimer;
import frc.excalib2.telemetry.Telemetry;
import frc.excalib2.util.AllianceFlip;
import frc.robot.util.ShiftUtil;

/**
 * Main robot class that extends TimedRobot.
 * Manages the overall robot initialization, periodic updates, and game mode transitions.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private final RobotContainer robotContainer;

    public Robot() {
        Telemetry.init(false, null);
        AllianceFlip.configure(
                Constants.FieldConstants.FIELD_LENGTH_METERS,
                Constants.FieldConstants.FIELD_WIDTH_METERS);

        robotContainer = new RobotContainer();

        // Disable unregistered status signals on all excalib2 devices (bus utilization).
        SignalHub.optimizeAll();
    }

    /**
     * Called every 20ms. Updates all periodic functions and executes scheduled commands.
     */
    @Override
    public void robotPeriodic() {
        LoopTimer.start();

        // Refresh excalib2 status signals BEFORE commands run, so they act on fresh data.
        SignalHub.refreshAll();
        FaultReporter.poll();

        ShiftUtil.update();
        robotContainer.periodic();

        CommandScheduler.getInstance().run();

        LoopTimer.end();
    }

    @Override
    public void disabledInit() {
        ShiftUtil.reset();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    /**
     * Called when autonomous period starts.
     * Safely schedules the selected autonomous command if one exists.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        } else {
            System.err.println("WARNING: No autonomous command selected!");
        }

        LoopTimer.reset();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    /**
     * Called when teleop period starts.
     * Cancels any running autonomous command and enables driver control.
     */
    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        // Commands are managed by CommandScheduler
    }

    @Override
    public void teleopExit() {
    }

    /**
     * Called when entering test mode.
     * Cancels all running commands for safe testing.
     */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
