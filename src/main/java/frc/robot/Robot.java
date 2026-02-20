// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.robot.util.AuroraPoseGetter;
import frc.robot.util.GameDataClient;
import monologue.Monologue;

import static frc.robot.Constants.PHYSICS_PERIODIC_TIME;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private final RobotContainer robotContainer;

    public Robot() {
        AuroraPoseGetter.periodic();
        robotContainer = new RobotContainer();
        Monologue.setupMonologue(robotContainer, "Robot", false, false);
    }

    @Override
    public void robotPeriodic() {
        AuroraPoseGetter.periodic();
        GameDataClient.updateGameData();
        robotContainer.perodic();

        Threads.setCurrentThreadPriority(true, 99);

        CommandScheduler.getInstance().run();
        Monologue.updateAll();
        TalonFXMotor.refreshAll();

        Threads.setCurrentThreadPriority(false, 10);
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

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

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
