// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.d

package frc.robot;

import java.util.Set;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.bobot_state.BobotState;
import frc.robot.bobot_state.varc.TargetAngleTracker;
import frc.robot.commands.RotateToTarget;
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
import frc.robot.subsystems.servo.ServoIOReal;
import frc.robot.subsystems.servo.ServoSubsystem;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {

  // space for calling subsystems and what not
  private final CommandCustomXboxController driveController = new CommandCustomXboxController(
      Constants.XboxDriverControllerPort);

  private final DriveSubsystem driveSubsystem;
  protected final CoralSubsystem coralSubsystem;
  private final ServoSubsystem servoSubsystem = new ServoSubsystem(new ServoIOReal());
  private final Vision vision = new Vision();

  private final AutoFactory autoFactory;
  public final AutoChooser oreoChooser;
  private final AutoRoutines autoRoutines;

  public RobotContainer() {
    new BobotState();

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

    oreoChooser = new AutoChooser();

    RobotModeTriggers.autonomous().whileTrue(oreoChooser.selectedCommandScheduler());

    autoFactory = new AutoFactory(
        driveSubsystem::getPose,
        driveSubsystem::setPose,
        driveSubsystem::followTrajectory,
        true,
        driveSubsystem);

    autoRoutines = new AutoRoutines(autoFactory, coralSubsystem, driveSubsystem);
    SmartDashboard.putData("Auto Choices", oreoChooser);

    // oreoChooser.addCmd("OreoTest", () -> Commands.sequence(
    // autoFactory.resetOdometry("Test"),
    // autoFactory.trajectoryCmd("Test")));
    oreoChooser.addCmd("Shpeal", autoRoutines::Shpeal);
    oreoChooser.addCmd("Wailmer", autoRoutines::Wailmer);
    oreoChooser.addCmd("Seel", autoRoutines::Seel);

    RobotModeTriggers.autonomous().onTrue(servoSubsystem.setAngle(90));

    configureBindings();

  }

  // *configures the bindings for any controllers */
  private void configureBindings() {
    // sets the default command for the drive train
    driveSubsystem
        .setDefaultCommand(
            driveSubsystem.driveCommand(() -> -driveController.getLeftY(), () -> -driveController.getRightX()));
    // driveSubsystem
    // .setDefaultCommand(
    // Commands.run(() -> driveSubsystem.runClosedLoop(1, 1), driveSubsystem));

    driveController.rightTrigger().whileTrue(coralSubsystem.runCoral(7.0));
    driveController.leftTrigger().whileTrue(coralSubsystem.runCoral(-6.0));
    driveController.rightBumper().whileTrue(coralSubsystem.runCoral(5.0));
    driveController.y().and(DriverStation::isDisabled)
        .onTrue(Commands.runOnce(() -> driveSubsystem.setPose(Pose2d.kZero), driveSubsystem)
            .ignoringDisable(true));
    driveController.x()
        .whileTrue(new RotateToTarget(driveSubsystem, () -> BobotState.getRotationToClosestReefIfPresent(),
            () -> -driveController.getLeftY()));
    driveController.a()
        .whileTrue(new RotateToTarget(driveSubsystem, () -> BobotState.getRotationToClosestHPSIfPresent(),
            () -> -driveController.getLeftY()));

  }
}