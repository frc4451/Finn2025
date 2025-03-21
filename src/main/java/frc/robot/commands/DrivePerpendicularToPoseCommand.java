package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.PoseUtils;

public class DrivePerpendicularToPoseCommand extends Command {
    private final PIDController parallelController = DriveCommandConstants.makeTranslationController();

    private final ProfiledPIDController angleController = DriveCommandConstants.makeAngleController();

    private final DriveSubsystem drive;
    private final Supplier<Pose2d> targetPoseSupplier;
    private final Supplier<Double> perpendicularInput;

    private double perpendicularError = 0;

    public DrivePerpendicularToPoseCommand(
            DriveSubsystem drive, Supplier<Pose2d> targetPoseSupplier, Supplier<Double> perpendicularInput) {
        addRequirements(drive);
        this.drive = drive;
        this.targetPoseSupplier = targetPoseSupplier;
        this.perpendicularInput = perpendicularInput;
    }

    public static DrivePerpendicularToPoseCommand withJoystickRumble(
            DriveSubsystem drive,
            Supplier<Pose2d> targetPoseSupplier,
            Supplier<Double> perpendicularInput,
            Command rumbleCommand) {
        DrivePerpendicularToPoseCommand command = new DrivePerpendicularToPoseCommand(drive, targetPoseSupplier,
                perpendicularInput);

        // command.atSetpoint().onTrue(Commands.deferredProxy(() -> rumbleCommand));

        return command;
    }

    @Override
    public void execute() {
        Pose2d robotPose = drive.getPose();
        Pose2d targetPose = targetPoseSupplier.get();
        Logger.recordOutput("Commands/" + getName() + "/targetPose", targetPose);

        Rotation2d desiredTheta = targetPose.getRotation().plus(Rotation2d.kPi);

        perpendicularError = PoseUtils.getPerpendicularError(robotPose, targetPose);
        Logger.recordOutput("Commands/" + getName() + "/PerpendicularError", perpendicularError);

        double thetaError = robotPose.getRotation().minus(desiredTheta).getRadians();
        Logger.recordOutput("Commands/" + getName() + "/ThetaError", thetaError);

        double angularSpeed = angleController.calculate(thetaError, 0);
        angularSpeed = !angleController.atSetpoint() ? angularSpeed : 0;

        ChassisSpeeds speeds = new ChassisSpeeds(
                perpendicularInput.get() * drive.getMaxLinearSpeedMetersPerSec(),
                angularSpeed, angularSpeed);

        // drive.runVelocity(speeds);
    }

    // public Trigger atSetpoint() {
    // return new Trigger(
    // () ->
    // parallelController.atSetpoint()
    // && angleController.atSetpoint()
    // && perpendicularError < Units.feetToMeters(2.5));
    // }
}