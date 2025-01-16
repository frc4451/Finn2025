// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.d

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.controllers.CommandCustomXboxController;
import frc.robot.subsystems.coral.CoralIOSpark;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {

  // space for calling subsystems and what not
  private final CommandCustomXboxController driveController = new CommandCustomXboxController(
      Constants.XboxDriverControllerPort);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final CoralSubsystem coralSubsystem = new CoralSubsystem(new CoralIOSpark());

  public RobotContainer() {
    configureBindings();
  }

  // *configures the bindings for any controllers */
  private void configureBindings() {
    // sets the default command for the drive train
    driveSubsystem
        .setDefaultCommand(
            driveSubsystem.driveCommand(() -> -driveController.getLeftY(), () -> -driveController.getRightX()));

    driveController.rightBumper().whileTrue(coralSubsystem.runCoral(10.0));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
