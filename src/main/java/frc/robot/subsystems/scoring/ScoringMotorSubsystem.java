package frc.robot.subsystems.scoring;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ScoringMotorSubsystem implements Subsystem {

    public final SparkMax scoringMotor = new SparkMax(ScoringMotorConstants.kScoringMotorId, MotorType.kBrushed);

}
