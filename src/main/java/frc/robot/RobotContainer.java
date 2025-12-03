// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.d

package frc.robot;

import java.lang.module.FindException;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.autos.AutoRoutines;
import frc.robot.bobot_state.BobotState;
import frc.robot.commands.RotateToTarget;
import frc.robot.controllers.CommandCustomXboxController;
import frc.robot.subsystems.singleRoller.SingleRollerIO;
import frc.robot.subsystems.singleRoller.SingleRollerIOSim;
import frc.robot.subsystems.singleRoller.SingleRollerIOSpark;
import frc.robot.subsystems.singleRoller.SingleRollerSubsystem;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSpark;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon1;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOSpark;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.servo.ServoJJ;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {

        // space for calling subsystems and what not
        private final CommandCustomXboxController driveController = new CommandCustomXboxController(
                        Constants.XboxDriverControllerPort);
        private final CommandCustomXboxController operatorController = new CommandCustomXboxController(
                        Constants.XboxOperatorControllerPort);

        private final DriveSubsystem driveSubsystem;
        protected final SingleRollerSubsystem coralSubsystem;
        protected final PivotSubsystem climberSubsystem;
        private final ServoJJ frontFlap = new ServoJJ(0);
        private final ServoJJ intakeDropout1 = new ServoJJ(1);
        private final ServoJJ intakeDropout2 = new ServoJJ(2);
        private final Vision vision = new Vision();

        private final AutoFactory autoFactory;
        public final AutoChooser oreoChooser;
        private final AutoRoutines autoRoutines;

        public final Field2d field = new Field2d();

        public RobotContainer() {
                new BobotState();

                switch (Constants.currentMode) {
                        case REAL:
                                driveSubsystem = new DriveSubsystem(new DriveIOSpark(), new GyroIOPigeon1());
                                coralSubsystem = new SingleRollerSubsystem(new SingleRollerIOSpark());
                                climberSubsystem = new PivotSubsystem(new PivotIOSpark());
                                break;

                        case SIM:
                                driveSubsystem = new DriveSubsystem(new DriveIOSim(), new GyroIO() {
                                });
                                coralSubsystem = new SingleRollerSubsystem(new SingleRollerIOSim() {
                                });
                                climberSubsystem = new PivotSubsystem(new PivotIOSim() {
                                });
                                break;

                        case REPLAY:
                        default:
                                driveSubsystem = new DriveSubsystem(new DriveIO() {
                                }, new GyroIO() {
                                });
                                coralSubsystem = new SingleRollerSubsystem(new SingleRollerIO() {
                                });
                                climberSubsystem = new PivotSubsystem(new PivotIO() {
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
                // oreoChooser.addCmd("SeelTest", autoRoutines::Seel);
                oreoChooser.addCmd("Seel", autoRoutines::Seel);
                oreoChooser.addCmd("FF Calibration", () -> driveSubsystem.feedforwardCharacterization());

                // RobotModeTriggers.autonomous().onTrue(servoSubsystem.setAngle(90));
                RobotModeTriggers.teleop().onTrue(frontFlap.setAngle(90));
                configureBindings();

        }

        // *configures the bindings for any controllers */
        private void configureBindings() {
                // sets the default command for the drive train
                driveSubsystem.setDefaultCommand(
                                driveSubsystem.driveCommand(
                                        () -> -driveController.getLeftY(),
                                        () -> -driveController.getRightX()));
                // driveSubsystem
                // .setDefaultCommand(
                // Commands.run(() -> driveSubsystem.runClosedLoop(1, 1), driveSubsystem));

                driveController.rightTrigger().whileTrue(coralSubsystem.runSingleRoller(6.5));
                driveController.leftTrigger().whileTrue(coralSubsystem.runSingleRoller(-7.0));
                driveController.rightBumper().and(driveController.leftBumper().negate())
                                .whileTrue(coralSubsystem.runSingleRoller(6.0));

                driveController.y().and(DriverStation::isDisabled)
                                .onTrue(Commands.runOnce(() -> driveSubsystem.setPose(Pose2d.kZero), driveSubsystem)
                                                .ignoringDisable(true));

                driveController.x()
                                .whileTrue(new RotateToTarget(driveSubsystem,
                                                BobotState::getRotationToClosestReefIfPresent,
                                                () -> -driveController.getLeftY()));
                driveController.a()
                                .whileTrue(new RotateToTarget(driveSubsystem,
                                                BobotState::getRotationToClosestHPSIfPresent,
                                                () -> -driveController.getLeftY()));
                driveController.y().and(DriverStation::isTeleop)
                                .whileTrue(new RotateToTarget(driveSubsystem,
                                                BobotState::getRotationToClosestBargeIfPresent,
                                                () -> -driveController.getLeftY()));

                operatorController.leftBumper()
                                .whileTrue(climberSubsystem.setReference(0));
                operatorController.rightBumper().and(driveController.rightTrigger().negate())
                                .whileTrue(climberSubsystem.setReference(-160));
                operatorController.a()
                                .whileTrue(climberSubsystem.setReference(160));
                // operatorController.x()
                // .whileTrue(climberSubsystem.setReference(-10));
                operatorController.b()
                                .whileTrue(intakeDropout1.setAngle(180))
                                .whileTrue(intakeDropout2.setAngle(0));

        }
}