package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        public double leftPositionRad = 0.0;
        public double leftVelocityRadPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double[] leftCurrentAmps = new double[] {};

        public double rightPositionRad = 0.0;
        public double rightVelocityRadPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double[] rightCurrentAmps = new double[] {};
    }

    public default void configureMotorSettings(IdleMode mode) {

    }

    public default void updateInputs(DriveIOInputs inputs) {

    }

    public default void setVoltage(double leftVolts, double rightVolts) {

    }

    public default void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts,
            double rightFFVolts) {
    }

    public default void setDutyCycle(double leftOut, double rightOut) {
    }

    public default void stop() {
        setVoltage(0.0, 0.0);
    }

}
