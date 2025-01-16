package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIO {
    @AutoLog
    public static class CoralIOInputs {
        public double coralPositionRad = 0.0;
        public double coralVelocityRadPerSec = 0.0;
        public double coralAppliedVolts = 0.0;
        public double coralCurrentAmps = 0.0;
    }

    public default void updateInputs(CoralIOInputs inputs) {

    }

    public default void runVolts(double volts) {

    }

    public default void stop() {
        runVolts(0.0);
    }

}
