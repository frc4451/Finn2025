package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class GyroIOPigeon1 implements GyroIO {
    private final WPI_PigeonIMU pigeon = new WPI_PigeonIMU(DriveConstants.kPigeonId);

    public GyroIOPigeon1() {
        pigeon.reset();
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = pigeon.getRotation2d();
        inputs.connected = true;
    }
}