// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.excalib.additional_utilities.LEDs;
import frc.excalib.control.math.periodics.PeriodicScheduler;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.additional_utilities.PerformanceMetricsTracker;
import frc.robot.util.AuroraPoseGetter;
import frc.robot.util.GameDataClient;
import monologue.Monologue;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Main robot class that extends LoggedRobot.
 * Manages the overall robot initialization, periodic updates, and game mode transitions.
 * AdvantageKit's LoggedRobot handles the logging loop timing automatically.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private final RobotContainer robotContainer;
    private final PerformanceMetricsTracker performanceMetricsTracker;

    public Robot() {
        // --- AdvantageKit Logger Setup ---
        // Must be configured before any subsystems are instantiated.
        Logger.recordMetadata("ProjectName", "Rebuilt2026");
        Logger.recordMetadata("RuntimeType", getRuntimeType().toString());

        switch (Constants.CURRENT_MODE) {
            case REAL:
                // Log to USB drive on the RoboRIO and publish over NT4
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIM:
                // Only publish over NT4 in sim
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                // No receivers needed; replay source would be set separately
                setUseTiming(false);
                break;
        }

        Logger.start();

        // --- Robot Initialization ---
        AuroraPoseGetter.periodic();

        robotContainer = new RobotContainer();

        performanceMetricsTracker = robotContainer.getPerformanceMetricsTracker();
        LEDs.getInstance().restoreLEDs();

        Monologue.setupMonologue(robotContainer, "Robot", false, false);
    }

    /**
     * Called every 20ms. Updates all periodic functions and executes scheduled commands.
     * Maintains high thread priority for motor updates to ensure consistent control.
     */
    @Override
    public void robotPeriodic() {
        // Record loop start time for performance tracking
        performanceMetricsTracker.recordLoopStart();

        AuroraPoseGetter.periodic();

        GameDataClient.updateGameData();
        robotContainer.periodic();
        PeriodicScheduler.PERIOD.MILLISECONDS_20.run();

        Threads.setCurrentThreadPriority(true, 99);

        CommandScheduler.getInstance().run();
        Monologue.updateAll();
        TalonFXMotor.refreshAll();

        performanceMetricsTracker.recordLoopEnd();
    }

    @Override
    public void disabledInit() {
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

        performanceMetricsTracker.reset();
    }

    @Override
    public void autonomousPeriodic() {
    }

    /**
     * Called when transitioning from autonomous to disabled or next phase.
     */
    @Override
    public void autonomousExit() {
        // Print performance summary after autonomous period
        performanceMetricsTracker.printSummary();
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
        performanceMetricsTracker.printSummary();
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
