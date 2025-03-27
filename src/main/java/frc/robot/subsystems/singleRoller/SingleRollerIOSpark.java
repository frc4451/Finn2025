package frc.robot.subsystems.singleRoller;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.SparkUtil;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SingleRollerIOSpark implements SingleRollerIO {
        /** Creates the SingleRollerMotor and Encoder objects */
        protected int CANid;

        public SingleRollerIOSpark(int CANid) {
                this.CANid = CANid;
                configureSingleRollerSettings();
        }

        private final SparkMax SingleRollerMotor = new SparkMax(CANid, SingleRollerConstants.kSingleRollerMotorType);
        private final RelativeEncoder SingleRollerEncoder = SingleRollerMotor.getEncoder();
        private final SparkClosedLoopController closedLoopController = SingleRollerMotor.getClosedLoopController();

        /**
         * Configures motor settings for SingleRoller motor, can be tweaked for some
         * tuning if
         * needed (Needed)
         */
        private void configureSingleRollerSettings() {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(true);
                config.idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(60)
                                .voltageCompensation(12.0);
                config.encoder
                                .uvwMeasurementPeriod(SingleRollerConstants.kUpdatePeriodMilliseconds)
                                .uvwAverageDepth(2);
                // config.closedLoop
                // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // .pidf(SingleRollerConstants.kP, SingleRollerConstants.kI,
                // SingleRollerConstants.kD,
                // SingleRollerConstants.kF);
                config.signals
                                .primaryEncoderPositionPeriodMs(SingleRollerConstants.kUpdatePeriodMilliseconds)
                                .primaryEncoderVelocityPeriodMs(SingleRollerConstants.kUpdatePeriodMilliseconds)
                                .appliedOutputPeriodMs(SingleRollerConstants.kUpdatePeriodMilliseconds)
                                .busVoltagePeriodMs(SingleRollerConstants.kUpdatePeriodMilliseconds)
                                .outputCurrentPeriodMs(SingleRollerConstants.kUpdatePeriodMilliseconds);
                SparkUtil.tryUntilOk(
                                SingleRollerMotor,
                                5,
                                () -> SingleRollerMotor.configure(
                                                config,
                                                ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));

        }

        @Override
        public void updateInputs(SingleRollerIOInputs inputs) {
                SparkUtil.ifOk(SingleRollerMotor, SingleRollerEncoder::getPosition,
                                (value) -> inputs.singleRollerPositionRad = value);
                SparkUtil.ifOk(SingleRollerMotor, SingleRollerEncoder::getVelocity,
                                (value) -> inputs.singleRollerVelocityRadPerSec = value);
                SparkUtil.ifOk(SingleRollerMotor,
                                new DoubleSupplier[] { SingleRollerMotor::getAppliedOutput,
                                                SingleRollerMotor::getBusVoltage },
                                (value) -> inputs.singleRollerAppliedVolts = value[0] * value[1]);
        }

        @Override
        public void runVolts(double volts) {
                SingleRollerMotor.setVoltage(volts);
        }

        @Override
        public void setReference(double setpoint) {
                closedLoopController.setReference(setpoint, ControlType.kMAXMotionPositionControl);
        }

}
