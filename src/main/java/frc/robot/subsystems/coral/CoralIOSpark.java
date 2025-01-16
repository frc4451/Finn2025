package frc.robot.subsystems.coral;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.util.SparkUtil;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class CoralIOSpark implements CoralIO {

    private final SparkMax coralMotor = new SparkMax(CoralConstants.kCoralMotorId, CoralConstants.kCoralMotorType);
    private final RelativeEncoder coralEncoder = coralMotor.getEncoder();

    public CoralIOSpark() {
        configureCoralSettings();
    }

    private void configureCoralSettings() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(0)
                .voltageCompensation(12.0);
        config.encoder
                .uvwMeasurementPeriod(20)
                .uvwAverageDepth(2);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(CoralConstants.kDriveKp, 0, CoralConstants.kDriveKd, 0);
        config.signals
                .primaryEncoderPositionPeriodMs(20)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        SparkUtil.tryUntilOk(
                coralMotor,
                5,
                () -> coralMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    }

    @Override
    public void updateInputs(CoralIOInputs inputs) {
        SparkUtil.ifOk(coralMotor, coralEncoder::getPosition, (value) -> inputs.coralPositionRad = value);
        SparkUtil.ifOk(coralMotor, coralEncoder::getVelocity, (value) -> inputs.coralVelocityRadPerSec = value);
        SparkUtil.ifOk(coralMotor, new DoubleSupplier[] { coralMotor::getAppliedOutput, coralMotor::getBusVoltage },
                (value) -> inputs.coralAppliedVolts = value[0] * value[1]);
    }

    @Override
    public void runVolts(double volts) {
        coralMotor.setVoltage(volts);
    }
}
