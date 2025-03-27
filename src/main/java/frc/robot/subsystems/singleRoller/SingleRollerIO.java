package frc.robot.subsystems.singleRoller;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SingleRollerIO {
    @AutoLog
    public static class SingleRollerIOInputs {
        public double singleRollerPositionRad = 0.0;
        public double singleRollerVelocityRadPerSec = 0.0;
        public double singleRollerAppliedVolts = 0.0;
        public double singleRollerCurrentAmps = 0.0;
    }

    public default void updateInputs(SingleRollerIOInputs inputs) {

    }

    public default void runVolts(double volts) {

    }

    public default void resetPosition(double positionRotations) {
    }

    public default void setReference(double setpoint) {
    }

    public default void stop() {
        runVolts(0.0);
    }

}
