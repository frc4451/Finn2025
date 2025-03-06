package frc.robot.subsystems.servo;

import org.littletonrobotics.junction.AutoLog;

public interface ServoIO {

    @AutoLog
    public static class ServoIOInputs {
        public double servoPositionAngle = 0.0;

    }

    public default void updateInputs(ServoIOInputs inputs) {

    }

    public default void setAngle(double angle) {

    }

}
