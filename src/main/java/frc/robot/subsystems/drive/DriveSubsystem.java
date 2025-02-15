package frc.robot.subsystems.drive;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import choreo.trajectory.DifferentialSample;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.bobot_state.BobotState;
import frc.robot.subsystems.vision.PoseObservation;

public class DriveSubsystem implements Subsystem {
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private final DriveIO driveIO;
    private boolean isBrake = false;

    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final GyroIO gyroIO;

    // private final PIDController drivePID = new PIDController(1.0, 0.0, 0.0);

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

    private final LTVUnicycleController ltvController = new LTVUnicycleController(0.02, DriveConstants.kMaxSpeed);

    public DriveSubsystem(DriveIO driveIO, GyroIO gyroIO) {
        this.driveIO = driveIO;
        this.gyroIO = gyroIO;
        setPose(Pose2d.kZero);
    }

    @Override
    public void periodic() {
        driveIO.updateInputs(inputs);
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive", inputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        if (DriverStation.isDisabled()) {
            if (!isBrake) {
                // driveIO.configureMotorSettings(IdleMode.kBrake);
                isBrake = true;
            }
            driveIO.stop();
        } else {
            if (isBrake) {
                // driveIO.configureMotorSettings(IdleMode.kCoast);
                isBrake = false;
            }
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

        PoseObservation observation;
        while ((observation = BobotState.getVisionObservations().poll()) != null) {
            poseEstimator.addVisionMeasurement(
                    observation.robotPose().toPose2d(), observation.timestampSeconds()
            // ,observation.stdDevs()
            );
        }
        BobotState.updateGlobalPose(getPose());
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
        Logger.recordOutput("DriveSubsystem/SpeedSetpoint", speeds);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        runClosedLoop(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    /**
     * Executes commands given from driveCommand, converts from meters per second to
     * radians per second for velocity
     */
    public void runClosedLoop(double leftMetersPerSec, double rightMetersPerSec) {
        double leftRadPerSec = leftMetersPerSec / DriveConstants.kWheelRadiusMeters;
        double leftRotPerMin = leftRadPerSec * (60 / 2 * Math.PI);
        double rightRadPerSec = rightMetersPerSec / DriveConstants.kWheelRadiusMeters;
        double rightRotPerMin = rightRadPerSec * (60 / 2 * Math.PI);
        Logger.recordOutput("DriveSubsystem/Setpoint/LeftRadPerSec", leftRadPerSec);
        Logger.recordOutput("DriveSubsystem/Setpoint/RightRadPerSec", rightRadPerSec);
        Logger.recordOutput("DriveSubsystem/LeftRPM", leftRotPerMin);
        Logger.recordOutput("DriveSubsystem/RightRPM", rightRotPerMin);

        double leftFFVolts = (kS * Math.signum(leftRadPerSec)) + (kV * leftRadPerSec);
        double rightFFVolts = (kS * Math.signum(rightRadPerSec)) + (kV * rightRadPerSec);

        driveIO.setVelocity(leftRadPerSec, rightRadPerSec, leftFFVolts, rightFFVolts);
    }

    public void followTrajectory(DifferentialSample sample) {
        // Get the current pose of the robot
        Pose2d pose = BobotState.getGlobalPose();

        // Get the velocity feedforward specified by the sample
        ChassisSpeeds ff = sample.getChassisSpeeds();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = ltvController.calculate(
                pose,
                sample.getPose(),
                ff.vxMetersPerSecond,
                ff.omegaRadiansPerSecond);

        // Apply the generated speeds
        runClosedLoop(speeds);
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
            Logger.recordOutput("DriveSubsystem/Setpoint/inputtedX", forward.getAsDouble());
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
                                    Logger.recordOutput("DriveSubsystem/kS", kS);
                                    Logger.recordOutput("DriveSubsystem/kV", kV);

                                }));
    }

}
