import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveToPoseCommand extends Command {
    private final PIDController perpendicularController = DriveCommandConstants.makeTranslationController();
    private final PIDController ParallelController = DriveCommandConstants.makeTranslationController();
    private final ProfiledPIDController angleController = DriveCommandConstants.makeAngleController();

    private final DriveSubsystem drive;
    private final Supplier<Pose2d> targetPoseSupplier;

    public DriveToPoseCommand(DriveSubsystem drive, Supplier<Pose2d> targetPoseSupplier) {
        addRequirements(drive);

        this.drive = drive;
        this.targetPoseSupplier = targetPoseSupplier;
    }

    @Override
    public void execute() {
        Pose2d robotPose = drive.getPose();
        Pose2d targetPose = targetPoseSupplier.get();
        Logger.recordOutput("Commands/" + getName() + "/TargetPose", targetPose);
        Transform2d error = robotPose.minus(targetPose);
        Logger.recordOutput("Commands/" + getName() + "/Error", error);
    }
}