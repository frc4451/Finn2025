package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.controllers.CommandCustomXboxController;
import frc.robot.util.SparkUtil;

public class DriveSubsystem implements Subsystem {

    public final SparkMax frontLeft = new SparkMax(DriveConstants.kFrontLeftId, MotorType.kBrushed);
    public final SparkMax frontRight = new SparkMax(DriveConstants.kFrontRightId, MotorType.kBrushed);
    public final SparkMax backLeft = new SparkMax(DriveConstants.kBackLeftId, MotorType.kBrushed);
    public final SparkMax backRight = new SparkMax(DriveConstants.kBackRightId, MotorType.kBrushed);

    private final DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);

    private final CommandCustomXboxController driverController = new CommandCustomXboxController(0);

    public DriveSubsystem() {
        configureMotorSettings();
    }

    private void configureMotorSettings() {
        SparkBaseConfig config = new SparkMaxConfig();
        config.openLoopRampRate(DriveConstants.kRampRateSeconds)
                .idleMode(IdleMode.kBrake);

        config.inverted(DriveConstants.kLeftInverted);
        SparkUtil.tryUntilOk(
                frontLeft,
                5,
                () -> frontLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        config.inverted(DriveConstants.kRightInverted);
        SparkUtil.tryUntilOk(
                frontRight,
                5,
                () -> frontRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        config.inverted(DriveConstants.kLeftInverted)
                .follow(frontLeft);
        SparkUtil.tryUntilOk(
                backLeft,
                5,
                () -> backLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        config.inverted(DriveConstants.kRightInverted)
                .follow(frontRight);
        SparkUtil.tryUntilOk(
                backRight,
                5,
                () -> backRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    }

    public Command driveCommand(DoubleSupplier forward, DoubleSupplier rotation) {
        return Commands.run(() -> {
            if (forward.getAsDouble() != 0) {
                drive.curvatureDrive(forward.getAsDouble(), rotation.getAsDouble(), false);
            } else {
                drive.curvatureDrive(forward.getAsDouble(), rotation.getAsDouble() / 2.0, true);
            }
        }, this);
    }
}
