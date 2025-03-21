package frc.robot.subsystems.drive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.SparkUtil;

public class DriveIOSpark implements DriveIO {
        /**
         * Creates different objects for all the motors, and encoder and controller
         * objects for the lead motors
         */
        private final SparkMax leftLeader = new SparkMax(DriveConstants.kFrontLeftId, DriveConstants.kMotorType);
        private final SparkMax rightLeader = new SparkMax(DriveConstants.kFrontRightId, DriveConstants.kMotorType);
        private final SparkMax leftFollower = new SparkMax(DriveConstants.kBackLeftId, DriveConstants.kMotorType);
        private final SparkMax rightFollower = new SparkMax(DriveConstants.kBackRightId, DriveConstants.kMotorType);
        private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
        private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
        private final SparkClosedLoopController leftController = leftLeader.getClosedLoopController();
        private final SparkClosedLoopController rightController = rightLeader.getClosedLoopController();

        public DriveIOSpark() {
                configureMotorSettings(IdleMode.kBrake);
        }

        /**
         * Configures Motor settings. Try until ok exists because of Rev Hardware bug
         * that makes it so sometimes they just don't apply the config and they just
         * won't work
         */
        @Override
        public void configureMotorSettings(IdleMode mode) {
                SparkBaseConfig config = new SparkMaxConfig();
                config.openLoopRampRate(DriveConstants.kRampRateSeconds)
                                .closedLoopRampRate(DriveConstants.kRampRateSeconds)
                                .idleMode(mode)
                                .voltageCompensation(12.0)
                                .smartCurrentLimit(DriveConstants.kCurrentLimit);
                // config.closedLoop.pidf(DriveConstants.kMotorKp, DriveConstants.kMotorKi,
                // DriveConstants.kMotorKd,
                // DriveConstants.kMotorKf);
                config.encoder
                                .positionConversionFactor((2 * Math.PI) / DriveConstants.kMotorReduction)
                                .velocityConversionFactor(((2 * Math.PI) / 60.0) / DriveConstants.kMotorReduction)
                                .uvwMeasurementPeriod(20)
                                .uvwAverageDepth(2);

                config.inverted(DriveConstants.kLeftInverted);
                SparkUtil.tryUntilOk(
                                leftLeader,
                                5,
                                () -> leftLeader.configure(config, ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));

                config.inverted(DriveConstants.kRightInverted);
                SparkUtil.tryUntilOk(
                                rightLeader,
                                5,
                                () -> rightLeader.configure(config, ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));

                config.inverted(DriveConstants.kLeftInverted)
                                .follow(leftLeader, false);
                SparkUtil.tryUntilOk(
                                leftFollower,
                                5,
                                () -> leftFollower.configure(config, ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));

                config.inverted(DriveConstants.kRightInverted)
                                .follow(rightLeader, false);
                SparkUtil.tryUntilOk(
                                rightFollower,
                                5,
                                () -> rightFollower.configure(config, ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));

                SparkUtil.tryUntilOk(
                                leftLeader,
                                5,
                                () -> leftEncoder.setPosition(0));

                SparkUtil.tryUntilOk(
                                rightLeader,
                                5,
                                () -> rightEncoder.setPosition(0));

        }

        @Override
        public void updateInputs(DriveIOInputs inputs) {
                inputs.leftPositionRad = leftEncoder.getPosition();
                inputs.leftVelocityRadPerSec = leftEncoder.getVelocity();
                inputs.leftAppliedVolts = leftLeader.getBusVoltage() * leftLeader.getAppliedOutput();
                inputs.leftCurrentAmps = new double[] { leftLeader.getOutputCurrent(),
                                leftFollower.getOutputCurrent() };

                inputs.rightPositionRad = rightEncoder.getPosition();
                inputs.rightVelocityRadPerSec = rightEncoder.getVelocity();
                inputs.rightAppliedVolts = rightLeader.getBusVoltage() * rightLeader.getAppliedOutput();
                inputs.rightCurrentAmps = new double[] { rightLeader.getOutputCurrent(),
                                rightFollower.getOutputCurrent() };
        }

        @Override
        public void setVoltage(double leftVolts, double rightVolts) {
                leftLeader.setVoltage(leftVolts);
                rightLeader.setVoltage(rightVolts);
        }

        @Override
        public void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
                leftController.setReference(leftRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, leftFFVolts);
                rightController.setReference(rightRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0,
                                rightFFVolts);
        }

        @Override
        public void setDutyCycle(double leftOut, double rightOut) {
                leftController.setReference(leftOut, ControlType.kDutyCycle);
                rightController.setReference(rightOut, ControlType.kDutyCycle);
        }

}