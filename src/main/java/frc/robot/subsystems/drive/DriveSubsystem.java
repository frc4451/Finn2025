package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.controllers.ControllerConstants;

public class DriveSubsystem implements Subsystem {
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private final DriveIO driveIO;

    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final GyroIO gyroIO;

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
            DriveConstants.kTrackWidthMeters);
    private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
            kinematics,
            new Rotation2d(),
            0.0,
            0.0,
            new Pose2d());

    private Rotation2d rawGyroRotation = new Rotation2d();
    private double lastLeftPositionMeters = 0.0;
    private double lastRightPositionMeters = 0.0;

    public DriveSubsystem(DriveIO driveIO, GyroIO gyroIO) {
        this.driveIO = driveIO;
        this.gyroIO = gyroIO;
    }

    @Override
    public void periodic() {
        driveIO.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);
        if (DriverStation.isDisabled()) {
            driveIO.stop();
        }

        if (gyroInputs.connected) {
            rawGyroRotation = gyroInputs.yaw;
        } else {
            Twist2d twist = kinematics.toTwist2d(
                    getLeftPositionMeters() - lastLeftPositionMeters,
                    getRightPositionMeters() - lastRightPositionMeters);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            lastLeftPositionMeters = getLeftPositionMeters();
            lastRightPositionMeters = getRightPositionMeters();
        }

        poseEstimator.update(rawGyroRotation, getLeftPositionMeters(), getRightPositionMeters());
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput
    public double getLeftPositionMeters() {
        return inputs.leftPositionRad * DriveConstants.kWheelRadiusMeters;
    }

    @AutoLogOutput
    public double getRightPositionMeters() {
        return inputs.rightPositionRad * DriveConstants.kWheelRadiusMeters;
    }

    @AutoLogOutput
    public double getLeftVelocityMetersPerSecond() {
        return inputs.leftVelocityRadPerSec * DriveConstants.kWheelRadiusMeters;
    }

    @AutoLogOutput
    public double getRightVelocityMetersPerSecond() {
        return inputs.rightVelocityRadPerSec * DriveConstants.kWheelRadiusMeters;
    }

    /**
     * Executes commands given from driveCommand, converts from meters per second to
     * radians per second for velocity
     */
    private void runClosedLoop(double leftMetersPerSec, double rightMetersPerSec) {
        double leftRadPerSec = leftMetersPerSec / DriveConstants.kWheelRadiusMeters;
        double rightRadPerSec = rightMetersPerSec / DriveConstants.kWheelRadiusMeters;
        Logger.recordOutput("DriveSubsystem/Setpoint/LeftRadPerSec", leftRadPerSec);
        Logger.recordOutput("DriveSubsystem/Setpoint/RightRadPerSec", rightRadPerSec);
        driveIO.setVelocity(leftRadPerSec, rightRadPerSec);
    }

    /** Command for controlling to drivetrain */
    public Command driveCommand(DoubleSupplier forward, DoubleSupplier rotation) {
        return Commands.run(() -> {
            WheelSpeeds speeds;
            if (forward.getAsDouble() > ControllerConstants.kJoystickDeadband
                    && forward.getAsDouble() < -ControllerConstants.kJoystickDeadband) {
                speeds = DifferentialDrive.curvatureDriveIK(forward.getAsDouble(), rotation.getAsDouble(), false);
            } else {
                speeds = DifferentialDrive.curvatureDriveIK(forward.getAsDouble(), rotation.getAsDouble(), true);
            }
            runClosedLoop(speeds.left, speeds.right);
        }, this);
    }
}
