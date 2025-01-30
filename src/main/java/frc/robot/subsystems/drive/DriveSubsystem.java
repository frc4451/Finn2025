package frc.robot.subsystems.drive;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LocalADStarAK;

public class DriveSubsystem implements Subsystem {
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private final DriveIO driveIO;

    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final GyroIO gyroIO;

    private final PIDController drivePID = new PIDController(1.0, 0.0, 0.0);

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
            DriveConstants.kTrackWidthMeters);
    private final double kS = Constants.currentMode == Mode.SIM ? DriveConstants.kSimKs : DriveConstants.kMotorKs;
    private final double kV = Constants.currentMode == Mode.SIM ? DriveConstants.kSimKv : DriveConstants.kMotorKv;
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
                this::getRobotRelativeSpeeds,
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

        setPose(Pose2d.kZero);
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
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(
                new DifferentialDriveWheelSpeeds(
                        getLeftVelocityMetersPerSecond(),
                        getRightVelocityMetersPerSecond()));
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
        Logger.recordOutput("DriveSubsystem/A", speeds);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        runClosedLoop(speeds.vxMetersPerSecond, speeds.vxMetersPerSecond);
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

        double leftFFVolts = kS * Math.signum(leftRadPerSec) + kV * leftRadPerSec;
        double rightFFVolts = kS * Math.signum(rightRadPerSec) + kV * rightRadPerSec;

        driveIO.setVelocity(leftRadPerSec, rightRadPerSec, leftFFVolts, rightFFVolts);
    }

    private void runDutyCycle(double leftOut, double rightOut) {
        driveIO.setDutyCycle(leftOut, rightOut);
    }

    /** Command for controlling to drivetrain */
    public Command driveCommand(DoubleSupplier forward, DoubleSupplier rotation) {
        return Commands.run(() -> {
            WheelSpeeds speeds;
            if (forward.getAsDouble() != 0) {
                speeds = DifferentialDrive.curvatureDriveIK(
                        forward.getAsDouble() * Math.abs(forward.getAsDouble()),
                        rotation.getAsDouble() * Math.abs(rotation.getAsDouble()) / 2, false);
            } else {
                speeds = DifferentialDrive.arcadeDriveIK(
                        0,
                        rotation.getAsDouble(),
                        true);

            }
            runClosedLoop(
                    speeds.left * DriveConstants.kMaxSpeed,
                    speeds.right * DriveConstants.kMaxSpeed);
        }, this);
    }

    /** Runs the drive in open loop. */
    public void runOpenLoop(double leftVolts, double rightVolts) {
        this.driveIO.setVoltage(leftVolts, rightVolts);
    }

    /** Returns the average velocity in radians/second. */
    public double getCharacterizationVelocity() {
        return (this.inputs.leftVelocityRadPerSec + this.inputs.rightVelocityRadPerSec) / 2.0;
    }

    /** Measures the velocity feedforward constants for the drive. */
    public Command feedforwardCharacterization() {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(
                        () -> {
                            velocitySamples.clear();
                            voltageSamples.clear();
                            timer.restart();
                        }),

                // Accelerate and gather data
                Commands.run(
                        () -> {
                            double voltage = timer.get() * DriveConstants.FF_RAMP_RATE;
                            this.runOpenLoop(voltage, voltage);
                            velocitySamples.add(this.getCharacterizationVelocity());
                            voltageSamples.add(voltage);

                            Logger.recordOutput("DriveFeedForward", voltage);
                        },
                        this)

                        // When cancelled, calculate and print results
                        .finallyDo(
                                () -> {
                                    int n = velocitySamples.size();
                                    double sumX = 0.0;
                                    double sumY = 0.0;
                                    double sumXY = 0.0;
                                    double sumX2 = 0.0;
                                    for (int i = 0; i < n; i++) {
                                        sumX += velocitySamples.get(i);
                                        sumY += voltageSamples.get(i);
                                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                                    }
                                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                                    NumberFormat formatter = new DecimalFormat("#0.00000");
                                    System.out.println("********** Drive FF Characterization Results **********");
                                    System.out.println("\tkS: " + formatter.format(kS));
                                    System.out.println("\tkV: " + formatter.format(kV));
                                }));
    }

}
