package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        SIM,
        REAL,
        REPLAY
    }

    public static final int XboxDriverControllerPort = 0;
}
