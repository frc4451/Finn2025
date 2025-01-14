package frc.robot.subsystems.coral;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.SparkUtil;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class CoralIOSpark implements CoralIO {

    private final SparkMax coralMotor = new SparkMax(CoralConstants.kCoralMotorId, CoralConstants.kCoralMotorType);
    private final RelativeEncoder coralEncoder = coralMotor.getEncoder();
    private final SparkClosedLoopController coralController = coralMotor.getClosedLoopController();

    private void configureCoralSettings() {
        SparkBaseConfig config = new SparkMaxConfig();
        config.openLoopRampRate(0)
                .idleMode(IdleMode.kBrake);

        config.inverted(false);
        SparkUtil.tryUntilOk(
                coralMotor,
                5,
                () -> coralMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    }
}
