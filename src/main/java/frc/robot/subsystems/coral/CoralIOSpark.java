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
        /** Creates the coralMotor and Encoder objects */
        private final SparkMax coralMotor = new SparkMax(CoralConstants.kCoralMotorId, CoralConstants.kCoralMotorType);
        private final RelativeEncoder coralEncoder = coralMotor.getEncoder();

        public CoralIOSpark() {
                configureCoralSettings();
        }

        /**
         * Configures motor settings for coral motor, can be tweaked for some tuning if
         * needed (Needed)
         */
        private void configureCoralSettings() {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(true);
                config.idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(40)
                                .voltageCompensation(12.0);
                config.encoder
                                .uvwMeasurementPeriod(CoralConstants.kUpdatePeriodMilliseconds)
                                .uvwAverageDepth(2);
                // config.closedLoop
                // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // .pidf(CoralConstants.kP, CoralConstants.kI, CoralConstants.kD,
                // CoralConstants.kF);
                config.signals
                                .primaryEncoderPositionPeriodMs(CoralConstants.kUpdatePeriodMilliseconds)
                                .primaryEncoderVelocityPeriodMs(CoralConstants.kUpdatePeriodMilliseconds)
                                .appliedOutputPeriodMs(CoralConstants.kUpdatePeriodMilliseconds)
                                .busVoltagePeriodMs(CoralConstants.kUpdatePeriodMilliseconds)
                                .outputCurrentPeriodMs(CoralConstants.kUpdatePeriodMilliseconds);
                SparkUtil.tryUntilOk(
                                coralMotor,
                                5,
                                () -> coralMotor.configure(
                                                config,
                                                ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));

        }

        @Override
        public void updateInputs(CoralIOInputs inputs) {
                SparkUtil.ifOk(coralMotor, coralEncoder::getPosition, (value) -> inputs.coralPositionRad = value);
                SparkUtil.ifOk(coralMotor, coralEncoder::getVelocity, (value) -> inputs.coralVelocityRadPerSec = value);
                SparkUtil.ifOk(coralMotor,
                                new DoubleSupplier[] { coralMotor::getAppliedOutput, coralMotor::getBusVoltage },
                                (value) -> inputs.coralAppliedVolts = value[0] * value[1]);
        }

        @Override
        public void runVolts(double volts) {
                coralMotor.setVoltage(volts);
        }
}
