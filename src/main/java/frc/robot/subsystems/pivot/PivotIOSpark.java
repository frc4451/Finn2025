package frc.robot.subsystems.pivot;

import java.util.function.DoubleSupplier;

import javax.security.auth.login.Configuration;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.pivot.PivotIO.PivotIOInputs;
import frc.robot.util.SparkUtil;

public class PivotIOSpark implements PivotIO {
    public PivotIOSpark() {
        configurePivotSettings();
    }

    private final SparkMax PivotMotor = new SparkMax(PivotConstants.kPivotMotorId, PivotConstants.kPivotMotorType);
    private final RelativeEncoder PivotEncoder = PivotMotor.getEncoder();
    private final SparkClosedLoopController closedLoopController = PivotMotor.getClosedLoopController();

    /**
     * Configures motor settings for Pivot motor, can be tweaked for some
     * tuning if
     * needed (Needed)
     */
    private void configurePivotSettings() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60)
                .voltageCompensation(12.0);
        config.encoder
                .uvwMeasurementPeriod(PivotConstants.kUpdatePeriodMilliseconds)
                .uvwAverageDepth(2);
        // config.closedLoop
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // .pidf(PivotConstants.kP, PivotConstants.kI,
        // PivotConstants.kD,
        // PivotConstants.kF);
        config.signals
                .primaryEncoderPositionPeriodMs(PivotConstants.kUpdatePeriodMilliseconds)
                .primaryEncoderVelocityPeriodMs(PivotConstants.kUpdatePeriodMilliseconds)
                .appliedOutputPeriodMs(PivotConstants.kUpdatePeriodMilliseconds)
                .busVoltagePeriodMs(PivotConstants.kUpdatePeriodMilliseconds)
                .outputCurrentPeriodMs(PivotConstants.kUpdatePeriodMilliseconds);
        // config.closedLoop.smartMotion

        // config.closedLoop.maxMotion
        // .maxVelocity(0)
        // .maxAcceleration(0)
        // .allowedClosedLoopError(0);
        config.closedLoop.pid(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);
        SparkUtil.tryUntilOk(
                PivotMotor,
                5,
                () -> PivotMotor.configure(
                        config,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));

    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        SparkUtil.ifOk(PivotMotor, PivotEncoder::getPosition,
                (value) -> inputs.PivotPositionRad = value);
        SparkUtil.ifOk(PivotMotor, PivotEncoder::getVelocity,
                (value) -> inputs.PivotVelocityRadPerSec = value);
        SparkUtil.ifOk(PivotMotor,
                new DoubleSupplier[] { PivotMotor::getAppliedOutput,
                        PivotMotor::getBusVoltage },
                (value) -> inputs.PivotAppliedVolts = value[0] * value[1]);
    }

    @Override
    public void runVolts(double volts) {
        PivotMotor.setVoltage(volts);
    }

    @Override
    public void setReference(double setpoint) {
        closedLoopController.setReference(setpoint, ControlType.kPosition);
    }
}
