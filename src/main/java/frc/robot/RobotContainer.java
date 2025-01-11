// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.d

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.controllers.CommandCustomXboxController;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem;

public class RobotContainer {

  private final CommandCustomXboxController driveController = new CommandCustomXboxController(
      Constants.XboxDriverControllerPort);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ScoringSubsystem scoringSubsystem = new ScoringSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driveSubsystem
        .setDefaultCommand(
            driveSubsystem.driveCommand(() -> -driveController.getLeftY(), () -> -driveController.getRightX()));

    driveController.rightTrigger()
        .whileTrue(scoringSubsystem.shoot());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
