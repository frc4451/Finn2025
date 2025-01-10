package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.controllers.ControllerConstants;
import frc.robot.util.SparkUtil;

public class DriveSubsystem implements Subsystem {

    // makes the drive train motors
    public final SparkMax frontLeft = new SparkMax(DriveConstants.kFrontLeftId, MotorType.kBrushed);
    public final SparkMax frontRight = new SparkMax(DriveConstants.kFrontRightId, MotorType.kBrushed);
    public final SparkMax backLeft = new SparkMax(DriveConstants.kBackLeftId, MotorType.kBrushed);
    public final SparkMax backRight = new SparkMax(DriveConstants.kBackRightId, MotorType.kBrushed);

    //connects the motors for curvatureDrive
    private final DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);

    public DriveSubsystem() {
        configureMotorSettings();
    }

    @Override
    public void periodic() {
        // pulled periodic for drivetrain/motor logging
        logMotor("FrontLeft", frontLeft);
        logMotor("FrontRight", frontRight);
        logMotor("BackLeft", backLeft);
        logMotor("BackRight", backRight);
    }

    /*
    *for logging SparkBase motors, logs Applied Voltage, Current, and Motor Temp in Celsius */
    private void logMotor(String motorName, SparkBase spark) {
        SmartDashboard.putNumber(motorName + "/AppliedVoltage", spark.getBusVoltage() * spark.getAppliedOutput());
        SmartDashboard.putNumber(motorName + "/Current", spark.getOutputCurrent());
        SmartDashboard.putNumber(motorName + "/MotorTemperatureCelsius", spark.getMotorTemperature());

    }

    //*configures motors for drivetrain. sets the open loop ramp rate, idle mode
    //, inverson, and back motor follwing */
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
            if (forward.getAsDouble() > ControllerConstants.kJoystickDeadband
                    && forward.getAsDouble() < -ControllerConstants.kJoystickDeadband) {
                drive.curvatureDrive(forward.getAsDouble(), rotation.getAsDouble(), false);
            } else {
                drive.curvatureDrive(forward.getAsDouble(), rotation.getAsDouble() / 2.0, true);
            }
        }, this);
    }
}
