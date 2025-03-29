package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double PivotPositionRad = 0.0;
        public double PivotVelocityRadPerSec = 0.0;
        public double PivotAppliedVolts = 0.0;
        public double PivotCurrentAmps = 0.0;
    }

    public default void updateInputs(PivotIOInputs inputs) {

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
