package frc.robot.subsystems.servo;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoJJ extends SubsystemBase {
    public Servo servo1 = new Servo(0);

    public Command setAngle(int ang) {
        return runOnce(() -> servo1.setAngle(ang));
    }
}
