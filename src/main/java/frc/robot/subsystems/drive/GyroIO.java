package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yaw = new Rotation2d();
    }

    public default void updateInputs(GyroIOInputs inputs) {

    }

}
