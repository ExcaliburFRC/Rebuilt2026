// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import monologue.Logged;

public class RobotContainer implements Logged {
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

//  private void init() {
//    .registerCommand("shootToSpeakerCommand", scoreNoteCommand(shooter.shootToSpeakerManualCommand(), new Trigger(() -> true), false));
//    NamedCommands.registerCommand("prepShooterCommand", shooter.prepShooterCommand());
//
//    NamedCommands.registerCommand("intakeFromGround", intake.halfIntakeFromGround());
//    NamedCommands.registerCommand("closeIntake", intake.closeIntakeCommand());
//    NamedCommands.registerCommand("pumpNote", intake.pumpNoteCommand());
//
//    NamedCommands.registerCommand("setSwerveIdle", swerve.setDriveIdleMode(CANSparkBase.IdleMode.kCoast));
//
//    NamedCommands.registerCommand("forceShoot", shooter.forceShootCommand().alongWith(new WaitCommand(0.75).andThen(intake.forceTransport(() -> false))));
//
//    NamedCommands.registerCommand("farShooter", scoreNoteCommand(shooter.manualShooter(1, 0.6), new Trigger(() -> true), false));
//    NamedCommands.registerCommand("prepFarShooter", shooter.prepFarShooter(() -> swerve.getDistanceFromPose(SPEAKER.pose.get())));
//    NamedCommands.registerCommand("shootFromDistance",
//            new SequentialCommandGroup(
//                    new InstantCommand(timer::restart),
//                    scoreNoteCommand(shooter.setShooterCommand(new ShooterVelocity(() -> swerve.getDistanceFromPose(SPEAKER.pose.get()))), shooter.getCurrentVelocity().velocitiesReady.and(timerTrigger), false)));
//
//    pitTab.add("System tester", systemTesterCommand().withName("SystemTest")).withSize(2, 2);
//
//    matchTab.addBoolean("intake note", intake.hasNoteTrigger).withPosition(19, 0).withSize(4, 3);
//    matchTab.add("delivery", new DeliveryCommand(shooter)).withPosition(16, 1).withSize(3, 2);
//    matchTab.add("climberMode", new ClimberModeCommand(climberLoop, swerve)).withPosition(16, 3).withSize(3, 2);
//
//    autoChooser.setDefaultOption("none", new InstantCommand(() -> {}));
//    autoChooser.addOption("123", swerve.runAuto("123"));
//    autoChooser.addOption("321", swerve.runAuto("321"));
//
//    autoChooser.addOption("3214", swerve.runAuto("3214"));
//    autoChooser.addOption("73", swerve.runAuto("73"));
//
//    autoChooser.addOption("shoot", shooter.forceShootCommand().alongWith(new WaitCommand(0.75).andThen(intake.forceTransport(() -> false))));
//    autoChooser.addOption("leaveFromBottom", swerve.runAuto("shootAndLeave"));
//
//    Shuffleboard.getTab("Auto").add(autoChooser);
//  }


}
