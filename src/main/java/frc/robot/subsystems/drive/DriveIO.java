package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface DriveIO {

    public static class DriveIOInputs {
        public double leftPositionRad = 0.0;
        public double leftVelocityRadPerSec = 0.0;
        public double leftAppliedVolt = 0.0;
        public double[] leftCurrentAmps = new double[] {};
    
        
        public double rightPositionRad = 0.0;
        public double rightVelocityRadPerSec = 0.0;
        public double rightAppliedVolt = 0.0;
        public double[] rightCurrentAmps = new double[] {};
    }
    
}
