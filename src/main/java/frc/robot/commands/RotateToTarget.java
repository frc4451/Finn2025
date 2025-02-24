package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.bobot_state.BobotState;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RotateToTarget extends Command {
    private final ProfiledPIDController angleController = DriveCommandConstants.makeAngleController();

    private final DriveSubsystem drive;
    private final Supplier<Rotation2d> targetRotationSupplier;

    public RotateToTarget(DriveSubsystem drive, Supplier<Rotation2d> targetRotationSupplier) {
        this.drive = drive;
        this.targetRotationSupplier = targetRotationSupplier;

    }

    @Override
    public void execute() {
        Logger.recordOutput("Commands/" + getName(), BobotState.getRotationToClosestReefIfPresent());
        Rotation2d targetRotation = targetRotationSupplier.get();

        Rotation2d robotRotation = BobotState.getGlobalPose().getRotation();
        double rotationError = robotRotation.minus(targetRotation).getRadians();
        double output = angleController.calculate(rotationError);

        drive.drive(0, output);
    }
}
