package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.controllers.ControllerConstants;
import frc.robot.util.LocalADStarAK;

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

        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                () -> kinematics.toChassisSpeeds(
                        new DifferentialDriveWheelSpeeds(
                                getLeftVelocityMetersPerSecond(),
                                getRightVelocityMetersPerSecond())),
                (ChassisSpeeds speeds) -> runClosedLoop(speeds),
                new PPLTVController(0.02, DriveConstants.kMaxSpeed),
                DriveConstants.ppConfig,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                activePath -> Logger.recordOutput(
                        "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging.setLogTargetPoseCallback(
                targetPose -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

    }

    @Override
    public void periodic() {
        driveIO.updateInputs(inputs);
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive", inputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
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

    public void setPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
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

    public void runClosedLoop(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        runClosedLoop(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    /**
     * Executes commands given from driveCommand, converts from meters per second to
     * radians per second for velocity
     */
    private void runClosedLoop(double leftMetersPerSec, double rightMetersPerSec) {
        double leftRadPerSec = leftMetersPerSec / DriveConstants.kWheelRadiusMeters;
        double rightRadPerSec = rightMetersPerSec / DriveConstants.kWheelRadiusMeters;
        Logger.recordOutput("DriveSubsystem/Setpoint/LeftRadPerSec", leftRadPerSec);
        Logger.recordOutput("DriveSubsystem/Setpoint/RightRadPerSec",
                rightRadPerSec);
        driveIO.setVelocity(leftRadPerSec, rightRadPerSec);
    }

    private void runDutyCycle(double leftOut, double rightOut) {
        driveIO.setDutyCycle(leftOut, rightOut);
    }

    /** Command for controlling to drivetrain */
    public Command driveCommand(DoubleSupplier forward, DoubleSupplier rotation) {
        return Commands.run(() -> {
            WheelSpeeds speeds;
            if (Math.abs(forward.getAsDouble()) > ControllerConstants.kJoystickDeadband) {
                speeds = DifferentialDrive.curvatureDriveIK(forward.getAsDouble(), rotation.getAsDouble(), false);
            } else {
                speeds = DifferentialDrive.curvatureDriveIK(forward.getAsDouble(), rotation.getAsDouble(), true);
            }
            runClosedLoop(
                    speeds.left * DriveConstants.kMaxSpeed,
                    speeds.right * DriveConstants.kMaxSpeed);
        }, this);
    }
}
