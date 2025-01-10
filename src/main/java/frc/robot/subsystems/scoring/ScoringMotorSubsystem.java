package frc.robot.subsystems.scoring;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ScoringMotorSubsystem implements Subsystem {

    public final SparkMax scoringMotor = new SparkMax(ScoringMotorConstants.kScoringMotorId, MotorType.kBrushed);
    
    public ScoringMotorSubsystem() {
        configureMotorSettings();
    }

    @Override
    public void periodic() {
        //Continuously logs info
        logMotor("ScoringMotor", scoringMotor);
    }

    private void logMotor(String motorName, SparkBase spark) {
        //Logs information about scoring motor to SmartDashboard
        SmartDashboard.putNumber(motorName + "/AppliedVoltage", spark.getBusVoltage() * spark.getAppliedOutput());
        SmartDashboard.putNumber(motorName + "/Current", spark.getOutputCurrent());
        SmartDashboard.putNumber(motorName + "/MotorTemperatureCelsius", spark.getMotorTemperature());

    }

    private void configureMotorSettings() {
        //Configures scoring motor to default state
        SparkMaxConfig config = new SparkMaxConfig();
        SparkUtil.tryUntilOk(
            scoringMotor,
            5,
            () -> scoringMotor.configure(config, ResetMode.kResetSafeParameters, Persist.kPersistParameters)
        );
    }

    public void spin(boolean score) {
        //I have zero idea if this actually works, this is about the laziest solution i could find because rev docs are kinda really bad
        if (score) {
            scoringMotor.set(0.7);
        } else {
            scoringMotor.set(0.0);
        }
    }
}
