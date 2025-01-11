package frc.robot.subsystems.scoring;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;

public class ScoringSubsystem extends SubsystemBase {

    public final SparkMax scoringMotor = new SparkMax(ScoringMotorConstants.kScoringMotorId, MotorType.kBrushed);

    public ScoringSubsystem() {
        configureMotorSettings();
    }

    @Override
    public void periodic() {
        // Continuously logs info
        logMotor("ScoringMotor", scoringMotor);
    }

    private void logMotor(String motorName, SparkBase spark) {
        // Logs information about scoring motor to SmartDashboard
        SmartDashboard.putNumber(motorName + "/AppliedVoltage", spark.getBusVoltage() * spark.getAppliedOutput());
        SmartDashboard.putNumber(motorName + "/Current", spark.getOutputCurrent());
        SmartDashboard.putNumber(motorName + "/MotorTemperatureCelsius", spark.getMotorTemperature());

    }

    /**
     * 
     */
    private void configureMotorSettings() {
        // Configures scoring motor to default state
        SparkMaxConfig config = new SparkMaxConfig();
        SparkUtil.tryUntilOk(
                scoringMotor,
                5,
                () -> scoringMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    public Command shoot() {
        return runEnd(() -> scoringMotor.set(0.7), scoringMotor::stopMotor);
    }
}
