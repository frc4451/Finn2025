package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.controllers.ControllerConstants;
import frc.robot.subsystems.drive.DriveIO.DriveIOInputs;
import frc.robot.util.SparkUtil;

public class DriveSubsystem implements Subsystem {

    private final DriveIO io = new DriveIOSpark();
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
            DriveConstants.kTrackWidthMeters);

    private void runClosedLoop(double leftMetersPerSec, double rightMetersPerSec) {
        double leftRadPerSec = leftMetersPerSec / DriveConstants.kWheelRadiusMeters;
        double rightRadPerSec = rightMetersPerSec / DriveConstants.kWheelRadiusMeters;
        Logger.recordOutput("Drive/LeftRadPerSec", leftRadPerSec);
        Logger.recordOutput("Drive/RightRadPerSec", rightRadPerSec);
        io.setVelocity(leftRadPerSec, rightRadPerSec);

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
