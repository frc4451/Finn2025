// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.d

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.controllers.CommandCustomXboxController;
import frc.robot.subsystems.coral.CoralIO;
import frc.robot.subsystems.coral.CoralIOSim;
import frc.robot.subsystems.coral.CoralIOSpark;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSpark;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon1;

public class RobotContainer {

  // space for calling subsystems and what not
  private final CommandCustomXboxController driveController = new CommandCustomXboxController(
      Constants.XboxDriverControllerPort);

  private final DriveSubsystem driveSubsystem;
  private final CoralSubsystem coralSubsystem;

  private final LoggedDashboardChooser<Command> autoChooser;
  private final AutoFactory autoFactory;

  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        driveSubsystem = new DriveSubsystem(new DriveIOSpark(), new GyroIOPigeon1());
        coralSubsystem = new CoralSubsystem(new CoralIOSpark());
        break;

      case SIM:
        driveSubsystem = new DriveSubsystem(new DriveIOSim(), new GyroIO() {

        });
        coralSubsystem = new CoralSubsystem(new CoralIOSim() {
        });
        break;

      case REPLAY:
      default:
        driveSubsystem = new DriveSubsystem(new DriveIO() {
        }, new GyroIO() {

        });
        coralSubsystem = new CoralSubsystem(new CoralIO() {
        });
        break;
    }
    NamedCommands.registerCommand("Score", coralSubsystem.runCoral(6.0).withTimeout(1));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoFactory = new AutoFactory(
        driveSubsystem::getPose,
        driveSubsystem::setPose,
        driveSubsystem::followTrajectory,
        true,
        driveSubsystem);

    autoChooser.addOption("[Characterization] FeedForward", driveSubsystem.feedforwardCharacterization());
    autoChooser.addOption("Choreo Test", oreoTest());

    configureBindings();
  }

  private Command oreoTest() {
    return Commands.sequence(
        autoFactory.resetOdometry("Test"),
        autoFactory.trajectoryCmd("Test"));
  };

  // *configures the bindings for any controllers */
  private void configureBindings() {
    // sets the default command for the drive train
    driveSubsystem
        .setDefaultCommand(
            driveSubsystem.driveCommand(() -> -driveController.getLeftY(), () -> -driveController.getRightX()));

    driveController.rightTrigger().whileTrue(coralSubsystem.runCoral(6.0));
    driveController.leftTrigger().whileTrue(coralSubsystem.runCoral(-6.0));
    driveController.b().whileTrue(coralSubsystem.runCoral(12.0));
    driveController.y().and(DriverStation::isDisabled)
        .onTrue(Commands.runOnce(() -> driveSubsystem.setPose(Pose2d.kZero), driveSubsystem)
            .ignoringDisable(true));

  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
